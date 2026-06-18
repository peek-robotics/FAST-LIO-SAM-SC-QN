#include "isam_backend.h"

#include <chrono>
using std::chrono::high_resolution_clock;
using std::chrono::microseconds;
using std::chrono::duration_cast;

// ── Construction ───────────────────────────────────────────────────────────────

IsamBackend::IsamBackend(const BackendParams& params)
    : p_(params), isam_(makeIsam2())
{
}

// ── Private: shared ISAM2 construction ────────────────────────────────────────

std::shared_ptr<gtsam::ISAM2> IsamBackend::makeIsam2()
{
    gtsam::ISAM2Params params;
    params.relinearizeThreshold = 0.01;
    params.relinearizeSkip      = 1;
    return std::make_shared<gtsam::ISAM2>(params);
}

// ── Graph initialisation ───────────────────────────────────────────────────────

void IsamBackend::initGraph(const gtsam::Pose3& init_pose,
                             gtsam::noiseModel::Diagonal::shared_ptr prior_noise,
                             int first_key)
{
    staged_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(first_key, init_pose, prior_noise));
    staged_init_.insert(first_key, init_pose);
}

// ── Factor staging (odom thread only — no lock) ────────────────────────────────

void IsamBackend::stageOdomFactor(int prev_key, int curr_key,
                                   const gtsam::Pose3& prev_pose, const gtsam::Pose3& curr_pose,
                                   bool is_degenerate)
{
    const auto& rot = is_degenerate ? p_.odom_noise_rot_degen : p_.odom_noise_rot;
    const auto& pos = is_degenerate ? p_.odom_noise_pos_degen : p_.odom_noise_pos;
    auto v = (gtsam::Vector(6) << rot[0], rot[1], rot[2], pos[0], pos[1], pos[2]).finished();
    auto odom_noise = gtsam::noiseModel::Diagonal::Variances(v);
    staged_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
        prev_key, curr_key, prev_pose.between(curr_pose), odom_noise));
}

void IsamBackend::stageFactors(const gtsam::NonlinearFactorGraph& factors)
{
    staged_graph_.push_back(factors);
}

void IsamBackend::stageInitValue(int key, const gtsam::Pose3& pose)
{
    staged_init_.insert(key, pose);
}

// ── Loop closure (loop-timer thread, acquires lock) ────────────────────────────

bool IsamBackend::isLoopDuplicate(int src_bucket, int dst_bucket) const
{
    std::lock_guard<std::mutex> lk(graph_mutex_);
    return existing_loops_.count({src_bucket, dst_bucket}) > 0;
}

bool IsamBackend::tryAddLoop(int src_idx, int dst_idx, int src_bucket, int dst_bucket,
                              const Eigen::Matrix4d& icp_pose_between_eig,
                              const Eigen::Matrix4d& lio_src_eig,
                              const Eigen::Matrix4d& lio_dst_eig,
                              double icp_score)
{
    const gtsam::Pose3 icp_from    = poseEigToGtsamPose(icp_pose_between_eig * lio_src_eig);
    const gtsam::Pose3 icp_to      = poseEigToGtsamPose(lio_dst_eig);
    const gtsam::Pose3 lio_from    = poseEigToGtsamPose(lio_src_eig);
    const gtsam::Pose3 lio_to      = poseEigToGtsamPose(lio_dst_eig);
    const gtsam::Pose3 icp_between = icp_from.between(icp_to);
    const gtsam::Rot3  lio_rel_rot = lio_from.rotation().between(lio_to.rotation());

    // Wrap yaw difference to [−180°, 180°].
    double yaw_diff_deg = (icp_between.rotation().yaw() - lio_rel_rot.yaw()) * 180.0 / M_PI;
    while (yaw_diff_deg >  180.0) yaw_diff_deg -= 360.0;
    while (yaw_diff_deg < -180.0) yaw_diff_deg += 360.0;
    yaw_diff_deg = std::abs(yaw_diff_deg);

    ROS_INFO("[Loop] ICP yaw=%.1f deg LIO yaw=%.1f deg (diff=%.1f deg)",
             icp_between.rotation().yaw() * 180.0 / M_PI,
             lio_rel_rot.yaw() * 180.0 / M_PI,
             yaw_diff_deg);

    if (yaw_diff_deg > p_.loop_max_yaw_diff_deg)
    {
        ROS_WARN("[Loop] Rejected: ICP yaw disagrees with LIO by %.1f deg > %.1f deg gate -- likely false match",
                 yaw_diff_deg, p_.loop_max_yaw_diff_deg);
        return false;
    }

    // Position disagreement gate: reject if ICP-derived translation differs from
    // the LIO odometry chain by more than max_pos_diff_m.  This is the position
    // analogue of the yaw gate — prevents loops that would pull the graph far from
    // where GPS and odometry already agree.  Set max_pos_diff_m ≤ 0 to disable.
    if (p_.loop_max_pos_diff_m > 0.0)
    {
        const gtsam::Pose3  lio_between = lio_from.between(lio_to);
        const double pos_diff_m = (icp_between.translation() - lio_between.translation()).norm();
        if (pos_diff_m > p_.loop_max_pos_diff_m)
        {
            ROS_WARN("[Loop] Rejected: ICP pos disagrees with LIO by %.2f m > %.2f m gate — likely false match",
                     pos_diff_m, p_.loop_max_pos_diff_m);
            return false;
        }
    }

    // Rotation variance: floor × (1 + scale × yaw_diff_deg) so the factor softens as
    // ICP-vs-LIO disagreement grows without hard-rejecting potentially valid yaw corrections.
    const double rv = p_.loop_noise_floor_rot * (1.0 + p_.loop_noise_rot_scale * yaw_diff_deg);
    const double tv = std::max(icp_score, p_.loop_noise_floor_pos);
    auto v = (gtsam::Vector(6) << rv, rv, rv, tv, tv, tv).finished();
    auto loop_noise = gtsam::noiseModel::Diagonal::Variances(v);

    ROS_INFO("[Loop] Factor noise: rot=%.4f rad^2 (%.1f deg 1-sig)  pos=%.3f m^2 (%.2f m 1-sig)",
             rv, std::sqrt(rv) * 180.0 / M_PI, tv, std::sqrt(tv));

    {
        std::lock_guard<std::mutex> lk(graph_mutex_);
        pending_loops_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            src_idx, dst_idx, icp_between, loop_noise));
        loop_idx_pairs_.push_back({static_cast<size_t>(src_idx), static_cast<size_t>(dst_idx)});
        existing_loops_.insert({src_bucket, dst_bucket});
    }
    loops_accepted.fetch_add(1, std::memory_order_relaxed);
    return true;
}

// ── ISAM2 commit (odom thread, acquires lock internally) ──────────────────────

IsamBackend::UpdateResult IsamBackend::commit(bool structural_change, bool gps_added, int gps_lm_passes)
{
    // Take ownership of the staged factors (written only from this thread).
    gtsam::NonlinearFactorGraph kf_graph = std::move(staged_graph_);
    gtsam::Values               kf_init  = std::move(staged_init_);
    staged_graph_.resize(0);
    staged_init_.clear();

    std::lock_guard<std::mutex> lk(graph_mutex_);

    // Merge pending loop factors from the loop-timer thread.
    kf_graph.push_back(pending_loops_);
    const bool had_loop = !pending_loops_.empty();
    pending_loops_.resize(0);

    const bool structural = structural_change || had_loop;

    UpdateResult result;
    bool isam_ok = false;

    try
    {
        isam_->update(kf_graph, kf_init);
        isam_->update();
        if (structural)  // extra updates to ensure convergence after structural graph change
        {                // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
            isam_->update();
            isam_->update();
            isam_->update();
            isam_->update();
            isam_->update();
        }

        // PERF NOTE: committed_ grows unboundedly (O(N_kf) factors).
        // LM rebuilds iterate the full history — monitor via perf stats / lm_max_factors gate.
        committed_.push_back(kf_graph);
        isam_ok = true;

        // [C] Batch LM global refinement — triggered by GPS re-entry / startup.
        // Adaptive pass count: passes = clamp(floor(dist_gap / step), 1, lm_max_passes)
        // where step = lm_max_distance / lm_max_passes.
        if (gps_added)
            runLMRefinementLocked(gps_lm_passes, "GPS");

        // [D] Batch LM after loop closure — distributes the correction globally.
        if (had_loop && p_.loop_lm_passes > 0)
            runLMRefinementLocked(p_.loop_lm_passes, "Loop");
    }
    catch (const gtsam::IndeterminantLinearSystemException& e)
    {
        ROS_WARN("[ISAM2] IndeterminantLinearSystem -- discarding loop factor and rebuilding: %s", e.what());

        // Remove the bad loop pair so the bucket can be retried later.
        if (had_loop && !loop_idx_pairs_.empty())
        {
            const auto& bad = loop_idx_pairs_.back();
            const int bs = static_cast<int>(bad.first)  / p_.min_loop_kf_sep;
            const int bd = static_cast<int>(bad.second) / p_.min_loop_kf_sep;
            existing_loops_.erase({bs, bd});
            loop_idx_pairs_.pop_back();
            loops_accepted.fetch_sub(1, std::memory_order_relaxed);
            loops_rejected.fetch_add(1, std::memory_order_relaxed);
        }

        // Filter the merged kf_graph to exclude loop BetweenFactors.
        // Loop factors: 2 keys with |k0 − k1| > 1.  Odom/GPS/Prior: 1 key or adjacent keys.
        gtsam::NonlinearFactorGraph safe_pending;
        for (const auto& f : kf_graph)
        {
            if (!f) continue;
            const auto& keys = f->keys();
            const bool is_loop = (keys.size() == 2 &&
                                  std::abs(static_cast<long>(keys[0]) - static_cast<long>(keys[1])) > 1);
            if (!is_loop) safe_pending.push_back(f);
        }

        // Rebuild ISAM2 from the last known-good committed history + safe current factors.
        gtsam::NonlinearFactorGraph rebuild_graph = committed_;
        rebuild_graph.push_back(safe_pending);

        gtsam::Values rebuild_values = calculateEstimate();  // from old isam_
        for (const auto& kv : kf_init)
        {
            if (!rebuild_values.exists(kv.key))
                rebuild_values.insert(kv.key, kv.value);
        }

        isam_ = makeIsam2();
        try
        {
            isam_->update(rebuild_graph, rebuild_values);
            committed_.push_back(safe_pending);
            ROS_INFO("\033[1;32m[ISAM2] Rebuilt from %zu committed factors (loop discarded)\033[0m",
                     committed_.size());
        }
        catch (const gtsam::IndeterminantLinearSystemException& e2)
        {
            ROS_ERROR("[ISAM2] Rebuild also failed: %s -- resetting to empty ISAM2", e2.what());
            isam_ = makeIsam2();
            committed_.resize(0);
        }

        isam_ok = false;
    }
    (void)isam_ok;

    // Retrieve corrected estimate.
    const gtsam::Values new_esti = isam_->calculateEstimate();
    if (new_esti.empty())
    {
        ROS_ERROR_THROTTLE(2.0, "[ISAM2] calculateEstimate() returned empty -- keeping stale estimate");
        return result;
    }

    result.ok          = true;
    result.needs_vis   = structural;
    result.estimate    = std::move(new_esti);
    result.marginal_cov = isam_->marginalCovariance(result.estimate.size() - 1);
    return result;
}

// ── LM refinement ─────────────────────────────────────────────────────────────

bool IsamBackend::runLMRefinementLocked(int passes, const std::string& tag)
{
    // Force full relinearization so LM seeds from the globally best ISAM2 estimate.
    isam_->update(gtsam::NonlinearFactorGraph(), gtsam::Values(),
                  gtsam::FactorIndices(), boost::none, boost::none,
                  boost::none, /*force_relinearize=*/true);

    if (p_.max_lm_factors > 0 &&
        committed_.size() > static_cast<size_t>(p_.max_lm_factors))
    {
        ROS_WARN("[%s] Skipping batch LM: %zu factors > limit %d -- ISAM2 force-relinearize only",
                 tag.c_str(), committed_.size(), p_.max_lm_factors);
        return false;
    }

    ROS_INFO("[%s] Batch LM on %zu committed factors (passes=%d)...",
             tag.c_str(), committed_.size(), passes);

    gtsam::LevenbergMarquardtParams lm_params;
    lm_params.maxIterations    = static_cast<size_t>(std::max(p_.lm_max_iterations, 1));
    lm_params.relativeErrorTol = p_.lm_rel_tol;
    lm_params.absoluteErrorTol = p_.lm_abs_tol;

    gtsam::Values lm_result;
    bool any_ok = false;
    auto lm_t0  = high_resolution_clock::now();

    for (int pass = 0; pass < passes; ++pass)
    {
        // Pass 0: seed from ISAM2 (freshly force-relinearized above).
        // Pass 1+: reuse previous lm_result — strictly better warm start.
        gtsam::Values lm_init = (pass == 0)
            ? isam_->calculateEstimate()
            : std::move(lm_result);

        try
        {
            lm_result = gtsam::LevenbergMarquardtOptimizer(
                            committed_, lm_init, lm_params).optimize();
        }
        catch (const std::exception& ex)
        {
            ROS_WARN("[%s] LM pass %d/%d failed: %s -- stopping passes",
                     tag.c_str(), pass + 1, passes, ex.what());
            break;
        }

        auto lm_t1 = high_resolution_clock::now();
        ROS_INFO("[%s] LM pass %d/%d done in %.1f ms -- rebuilding ISAM2",
                 tag.c_str(), pass + 1, passes,
                 duration_cast<microseconds>(lm_t1 - lm_t0).count() / 1e3);
        lm_t0 = lm_t1;

        isam_ = makeIsam2();
        try
        {
            isam_->update(committed_, lm_result);
            ROS_INFO("\033[1;32m[%s] ISAM2 rebuilt from LM pass %d/%d successfully\033[0m",
                     tag.c_str(), pass + 1, passes);
            any_ok = true;
        }
        catch (const gtsam::IndeterminantLinearSystemException& e)
        {
            ROS_ERROR("[%s] ISAM2 rebuild LM pass %d/%d failed: %s -- stopping passes",
                      tag.c_str(), pass + 1, passes, e.what());
            break;
        }
    }
    return any_ok;
}

bool IsamBackend::runLMRefinement(int passes, const std::string& tag)
{
    std::lock_guard<std::mutex> lk(graph_mutex_);
    return runLMRefinementLocked(passes, tag);
}

gtsam::Values IsamBackend::calculateEstimate()
{
    return isam_->calculateEstimate();
}

Eigen::MatrixXd IsamBackend::getMarginalCovariance(size_t last_key)
{
    return isam_->marginalCovariance(last_key);
}

// ── Visualisation ──────────────────────────────────────────────────────────────

visualization_msgs::Marker IsamBackend::getLoopMarkers(const gtsam::Values& esti,
                                                        const std::string& map_frame) const
{
    visualization_msgs::Marker edges;
    edges.type               = 5u;  // LINE_LIST
    edges.scale.x            = 0.12f;
    edges.header.frame_id    = map_frame;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f; edges.color.g = 1.0f; edges.color.b = 1.0f; edges.color.a = 1.0f;

    for (const auto& pair : loop_idx_pairs_)
    {
        if (pair.first >= esti.size() || pair.second >= esti.size())
            continue;
        const gtsam::Pose3 pa = esti.at<gtsam::Pose3>(pair.first);
        const gtsam::Pose3 pb = esti.at<gtsam::Pose3>(pair.second);
        geometry_msgs::Point p, q;
        p.x = pa.translation().x(); p.y = pa.translation().y(); p.z = pa.translation().z();
        q.x = pb.translation().x(); q.y = pb.translation().y(); q.z = pb.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(q);
    }
    return edges;
}

bool IsamBackend::hasLoops() const
{
    return !loop_idx_pairs_.empty();
}
