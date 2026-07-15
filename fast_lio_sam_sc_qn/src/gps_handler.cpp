#include "gps_handler.h"

#include <algorithm>
#include <cmath>

#include <gtsam/linear/NoiseModel.h>  // noiseModel::Robust + mEstimator kernels (#2)

// ── Construction ───────────────────────────────────────────────────────────────

GpsHandler::GpsHandler(const GpsParams& p)
    : p_(p)
{
}

// ── State queries ──────────────────────────────────────────────────────────────

bool GpsHandler::firstReceived() const
{
    std::lock_guard<std::mutex> lk(gps_mutex_);
    return gps_first_received_;
}

double GpsHandler::latestGpsZ() const
{
    std::lock_guard<std::mutex> lk(gps_mutex_);
    return latest_gps_z_;
}

void GpsHandler::armReentry()
{
    gps_reentry_pending_ = true;
}

bool GpsHandler::isReadyToInit() const
{
    return gps_heading_received_;
}

bool GpsHandler::hasGpsConstraints() const
{
    return !gps_constraint_points_.empty();
}

GpsHandler::InitSnapshot GpsHandler::getInitSnapshot() const
{
    return {init_gps_x_, init_gps_y_, init_gps_z_, gps_initial_yaw_, init_lat_, init_lon_};
}

// ── Subscriber callbacks ───────────────────────────────────────────────────────

void GpsHandler::onNavSatFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
    latest_fix_status_.store(msg->status.status);
    {
        std::lock_guard<std::mutex> lk(fix_mutex_);
        sensor_msgs::NavSatFix f = *msg;
        f.header.stamp += ros::Duration(p_.time_offset);  // #6 keep fix time consistent with GPS odom
        fix_queue_.push_back(std::move(f));
    }
    // Cache raw WGS-84 lat/lon for metadata export.
    // Protected by gps_mutex_ to avoid a separate lock.
    std::lock_guard<std::mutex> lk(gps_mutex_);
    latest_lat_ = msg->latitude;
    latest_lon_ = msg->longitude;
}

void GpsHandler::onGpsOdom(const nav_msgs::OdometryConstPtr& msg)
{
    double gx, gy, gz;
    {
        std::lock_guard<std::mutex> lk(gps_mutex_);
        latest_gps_x_     = gx = msg->pose.pose.position.x;
        latest_gps_y_     = gy = msg->pose.pose.position.y;
        latest_gps_z_     = gz = msg->pose.pose.position.z;
        nav_msgs::Odometry m = *msg;
        m.header.stamp += ros::Duration(p_.time_offset);  // #6 align GPS stamp to the LIO clock
        gps_queue_.push_back(std::move(m));
        gps_first_received_ = true;
    }

    // When a heading topic is configured, headingCallback drives the joint stability
    // window and sets gps_heading_received_. Nothing more to do here for init.
    if (p_.has_heading_topic || gps_heading_received_)
        return;

    // GPS-only init path.
    if (!isReadyForInitSample())
        return;

    runPositionWindowCheck(gx, gy, gz);
}

void GpsHandler::onHeading(const sensor_msgs::ImuConstPtr& msg)
{
    const auto& q = msg->orientation;
    if (std::abs(q.w) < 1e-6 && std::abs(q.x) < 1e-6 &&
        std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6)
        return;

    const double yaw = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z).yaw();

    // Run the joint GPS+heading stability window until init is confirmed.
    if (!gps_heading_received_)
        runJointInitWindow(yaw);

    // Cache the latest heading for use in tryAddFactor.
    {
        std::lock_guard<std::mutex> lk(heading_mutex_);
        latest_heading_yaw_   = yaw;
        latest_heading_stamp_ = msg->header.stamp.toSec();
    }
}

// ── Init stability window helpers ──────────────────────────────────────────────

bool GpsHandler::isReadyForInitSample() const
{
    if (!p_.require_gbas_for_init)
        return true;

    if (latest_fix_status_.load() < sensor_msgs::NavSatStatus::STATUS_GBAS_FIX)
    {
        ROS_INFO_THROTTLE(5.0, "[Init] Waiting for GBAS_FIX for SLAM init (current status=%d)",
                          static_cast<int>(latest_fix_status_.load()));
        return false;
    }
    return true;
}

void GpsHandler::finalizeInit(double yaw, double x, double y, double z)
{
    gps_initial_yaw_      = yaw;
    init_gps_x_           = x;
    init_gps_y_           = y;
    init_gps_z_           = z;
    {
        std::lock_guard<std::mutex> lk(gps_mutex_);
        init_lat_ = latest_lat_;
        init_lon_ = latest_lon_;
    }
    gps_heading_received_ = true;
}

void GpsHandler::runJointInitWindow(double yaw)
{
    double gx, gy, gz;
    {
        std::lock_guard<std::mutex> lk(gps_mutex_);
        if (!gps_first_received_)
        {
            ROS_INFO_THROTTLE(5.0, "[Init] Waiting for first GPS position before collecting init window...");
            return;
        }
        if (!isReadyForInitSample())
            return;
        gx = latest_gps_x_; gy = latest_gps_y_; gz = latest_gps_z_;
    }

    init_window_.push_back({yaw, gx, gy, gz});
    init_yaw_buf_.push_back(yaw);
    while ((int)init_window_.size() > p_.stable_count)
    {
        init_window_.pop_front();
        init_yaw_buf_.pop_front();
    }

    if ((int)init_window_.size() < p_.stable_count)
    {
        ROS_INFO_THROTTLE(2.0, "[Init] Collecting joint GPS+heading window (%d / %d)...",
                          (int)init_window_.size(), p_.stable_count);
        return;
    }

    // Check circular yaw range.
    {
        const double ref = init_yaw_buf_.front();
        double min_d = 0.0, max_d = 0.0;
        for (const double y : init_yaw_buf_)
        {
            const double d = std::remainder(y - ref, 2.0 * M_PI);
            min_d = std::min(min_d, d);
            max_d = std::max(max_d, d);
        }
        const double yaw_range = max_d - min_d;
        if (yaw_range >= p_.stable_tol_yaw)
        {
            ROS_INFO_THROTTLE(2.0, "[Init] Yaw unstable (range=%.2f deg >= tol=%.2f deg) -- clearing window",
                              yaw_range * 180.0 / M_PI, p_.stable_tol_yaw * 180.0 / M_PI);
            init_window_.clear();
            init_yaw_buf_.clear();
            return;
        }
    }

    // Check GPS XY range.
    {
        double min_x = init_window_.front().x, max_x = min_x;
        double min_y = init_window_.front().y, max_y = min_y;
        for (const auto& s : init_window_)
        {
            min_x = std::min(min_x, s.x); max_x = std::max(max_x, s.x);
            min_y = std::min(min_y, s.y); max_y = std::max(max_y, s.y);
        }
        const double pos_range = std::max(max_x - min_x, max_y - min_y);
        if (pos_range >= p_.stable_tol_pos_m)
        {
            ROS_INFO_THROTTLE(2.0, "[Init] GPS position unstable (range=%.3f m >= tol=%.3f m) -- clearing window",
                              pos_range, p_.stable_tol_pos_m);
            init_window_.clear();
            init_yaw_buf_.clear();
            return;
        }
    }

    // Both stable — snapshot the last sample in the window.
    const auto& last = init_window_.back();
    finalizeInit(last.yaw, last.x, last.y, last.z);
    ROS_INFO("\033[1;32m[Init] Joint GPS+heading window stable over %d samples: "
             "yaw=%.1f deg  pos=(%.2f, %.2f, %.2f) -- SLAM init unblocked.\033[0m",
             p_.stable_count, last.yaw * 180.0 / M_PI, last.x, last.y, last.z);
}

bool GpsHandler::runPositionWindowCheck(double x, double y, double z)
{
    init_window_.push_back({std::numeric_limits<double>::quiet_NaN(), x, y, z});
    while ((int)init_window_.size() > p_.stable_count)
        init_window_.pop_front();

    if ((int)init_window_.size() < p_.stable_count)
    {
        ROS_INFO_THROTTLE(2.0, "[GPS] Collecting position stability window (%d / %d)...",
                          (int)init_window_.size(), p_.stable_count);
        return false;
    }

    double min_x = init_window_.front().x, max_x = min_x;
    double min_y = init_window_.front().y, max_y = min_y;
    for (const auto& s : init_window_)
    {
        min_x = std::min(min_x, s.x); max_x = std::max(max_x, s.x);
        min_y = std::min(min_y, s.y); max_y = std::max(max_y, s.y);
    }
    const double range = std::max(max_x - min_x, max_y - min_y);
    if (range >= p_.stable_tol_pos_m)
    {
        ROS_INFO_THROTTLE(2.0, "[GPS] Position unstable (range=%.3f m >= tol=%.3f m) -- clearing window",
                          range, p_.stable_tol_pos_m);
        init_window_.clear();
        return false;
    }

    finalizeInit(std::numeric_limits<double>::quiet_NaN(), x, y, z);
    ROS_INFO("\033[1;32m[Init] GPS position stable over %d samples: pos=(%.2f, %.2f, %.2f) -- SLAM init unblocked.\033[0m",
             p_.stable_count, x, y, z);
    return true;
}

// ── Per-keyframe GPS factor ────────────────────────────────────────────────────

const GpsFixTier* GpsHandler::resolveFixTier(double msg_time)
{
    if (!p_.use_fix_tiers)
        return &p_.default_tier;

    std::lock_guard<std::mutex> lk(fix_mutex_);

    // Prune stale fixes.
    while (!fix_queue_.empty() && fix_queue_.front().header.stamp.toSec() < msg_time - 1.0)
        fix_queue_.pop_front();

    // Find the NavSatFix closest in time within ±0.1 s.
    double best_dt = 0.1 + 1e-9;
    int8_t best_status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
    bool found = false;
    for (const auto& fix : fix_queue_)
    {
        const double dt = std::abs(fix.header.stamp.toSec() - msg_time);
        if (dt < best_dt) { best_dt = dt; best_status = fix.status.status; found = true; }
    }

    if (!found)
    {
        ROS_WARN_THROTTLE(5.0, "[GPS] No NavSatFix within +/-0.1 s of GPS odometry -- rejecting factor");
        return nullptr;
    }

    auto it = p_.fix_tiers.find(best_status);
    if (it == p_.fix_tiers.end())
    {
        ROS_WARN_THROTTLE(5.0, "[GPS] Fix status %d below minimum accepted tier (SBAS=1, GBAS=2) -- rejecting",
                          static_cast<int>(best_status));
        return nullptr;
    }

    ROS_DEBUG("[GPS] Tier matched: status=%d  cov_gate=%.3f  noise_floor=%.3f  cov_scale=%.1f",
              static_cast<int>(best_status), it->second.cov_gate,
              it->second.noise_floor, it->second.cov_scale);
    return &it->second;
}

GpsHandler::FactorResult GpsHandler::tryAddFactor(
    double kf_time, int node_idx,
    double traveled_dist,
    const Eigen::Matrix4d& node_pose_corrected,
    gtsam::NonlinearFactorGraph& graph_out)
{
    FactorResult result;
    const double current_z = node_pose_corrected(2, 3);

    std::lock_guard<std::mutex> lk(gps_mutex_);
    if (gps_queue_.empty() && !buffering_)
        return result;

    // Traveled-distance gate (bypass for the very first GPS factor).
    if (!first_gps_added_)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] First GPS factor -- bypassing traveled-dist gate to anchor map origin");
    }
    else if (traveled_dist < p_.min_traveled_dist)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: traveled dist %.2f m < %.1f m threshold",
                          traveled_dist, p_.min_traveled_dist);
        return result;
    }

    // Time-sync: GPS @ 5 Hz → max inter-message gap 0.2 s, so ±0.1 s keeps us within
    // half a GPS period of the keyframe and avoids applying a fix from a different pose.
    // In buffer mode the sync window is relaxed: we consume every message up to kf_time so
    // that all fixes arriving between keyframes are collected as candidates.
    static constexpr double kGpsSyncWindow = 0.1;
    while (!gps_queue_.empty())
    {
        const double msg_time = gps_queue_.front().header.stamp.toSec();
        if (buffering_)
        {
            if (msg_time < buffer_start_time_)                       { gps_queue_.pop_front(); continue; }  // pre-buffer
            if (msg_time > buffer_start_time_ + p_.best_fix_window)  break;  // window closed in GPS time
            if (msg_time > kf_time)                                   break;  // no pose yet for this msg
        }
        else
        {
            if (msg_time < kf_time - kGpsSyncWindow)
            {
                ROS_DEBUG_THROTTLE(5.0, "[GPS] Time-sync: dropping old message (%.3f s behind keyframe)",
                                   kf_time - msg_time);
                gps_queue_.pop_front();
                continue;
            }
            if (msg_time > kf_time + kGpsSyncWindow)
            {
                ROS_DEBUG_THROTTLE(5.0, "[GPS] No sync: nearest msg %.3f s ahead of keyframe",
                                   msg_time - kf_time);
                break;
            }
        }

        const nav_msgs::Odometry gps_msg = gps_queue_.front();
        gps_queue_.pop_front();

        // Resolve quality tier.
        const GpsFixTier* tier = resolveFixTier(gps_msg.header.stamp.toSec());
        if (!tier)
            continue;

        const float noise_x = static_cast<float>(gps_msg.pose.covariance[0]);
        const float noise_y = static_cast<float>(gps_msg.pose.covariance[7]);
        const float noise_z = static_cast<float>(gps_msg.pose.covariance[14]);

        if (std::abs(noise_x) > static_cast<float>(tier->cov_gate) ||
            std::abs(noise_y) > static_cast<float>(tier->cov_gate) ||
            std::abs(noise_z) > static_cast<float>(tier->cov_gate))
        {
            ROS_INFO_THROTTLE(5.0, "[GPS] Rejected: position cov (%.4f, %.4f, %.4f) > gate %.4f",
                              noise_x, noise_y, noise_z, tier->cov_gate);
            continue;
        }

        const float gps_x = static_cast<float>(gps_msg.pose.pose.position.x);
        const float gps_y = static_cast<float>(gps_msg.pose.pose.position.y);

        if (std::abs(gps_x) < 1e-6f && std::abs(gps_y) < 1e-6f)
        {
            ROS_WARN_THROTTLE(5.0, "[GPS] Rejected: position at origin (uninitialized)");
            continue;
        }

        int8_t fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
        for (const auto& kv : p_.fix_tiers)
            if (&kv.second == tier) { fix_status = kv.first; break; }

        const bool quality_upgrade = first_gps_added_ && fix_status > last_gps_fix_status_;

        // Spacing and re-entry checks (XY only).
        const float dist_from_last = std::hypot(gps_x - last_gps_point_.x, gps_y - last_gps_point_.y);
        if (first_gps_added_)
        {
            if (dist_from_last < static_cast<float>(p_.min_spacing) && !quality_upgrade)
            {
                ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: %.2f m from last GPS point (< %.1f m)",
                                  dist_from_last, p_.min_spacing);
                continue;
            }
            if (dist_from_last < static_cast<float>(p_.min_spacing) && quality_upgrade)
            {
                ROS_INFO("[GPS] Fix quality upgrade %d -> %d within %.2f m -- bypassing spacing gate",
                         static_cast<int>(last_gps_fix_status_), static_cast<int>(fix_status), dist_from_last);
            }

            if (dist_from_last > 2.0f * static_cast<float>(p_.min_spacing) && re_entry_skip_remaining_ == 0)
            {
                re_entry_skip_remaining_ = p_.re_entry_skip_count;
                last_gps_point_ = pcl::PointXYZ(gps_x, gps_y,
                                                 static_cast<float>(gps_msg.pose.pose.position.z));
                gps_reentry_pending_ = true;
                ROS_WARN("[GPS] Re-entry detected (%.1f m gap) -- discarding next %d fix(es)",
                         dist_from_last, re_entry_skip_remaining_);
            }
            if (re_entry_skip_remaining_ > 0)
            {
                re_entry_skip_remaining_--;
                ROS_INFO("[GPS] Re-entry: discarding fix (%d remaining)", re_entry_skip_remaining_);
                continue;
            }
        }

        bool   hdg_has = false;
        double hdg_yaw = 0.0;
        if (p_.has_heading_topic)
        {
            std::lock_guard<std::mutex> hlk(heading_mutex_);
            const double gps_time = gps_msg.header.stamp.toSec();
            if (latest_heading_stamp_ > 0.0 && std::abs(latest_heading_stamp_ - gps_time) <= 0.05)
            {
                hdg_has = true;
                hdg_yaw = latest_heading_yaw_;
            }
            else if (latest_heading_stamp_ > 0.0)
                ROS_INFO_THROTTLE(2.0, "[GPS] No heading: %.3f s from GPS fix (> 50 ms threshold)",
                                  std::abs(latest_heading_stamp_ - gps_time));
        }

        if (p_.best_fix_window > 0.0)
        {
            // Buffer mode: collect candidates over the window, then commit the best.
            if (!buffering_)
            {
                buffering_         = true;
                buffer_start_time_ = gps_msg.header.stamp.toSec();
                ROS_INFO("[GPS] Best-fix buffering started (window=%.1f s)...", p_.best_fix_window);
            }
            BufCandidate c;
            c.gps        = gps_msg;
            c.tier       = *tier;
            c.fix_status = fix_status;
            c.cov_trace  = static_cast<double>(noise_x) + static_cast<double>(noise_y);
            c.hdg_yaw    = hdg_yaw;
            c.has_hdg    = hdg_has;
            fix_buffer_.push_back(std::move(c));
            continue;
        }

        // Immediate accept (buffering disabled).
        // #4 Temporal interpolation: gps_msg was just popped, so gps_queue_.front() is the
        // next fix. If [gps_msg, next] brackets the keyframe time, linearly interpolate the
        // position + diagonal covariance to kf_time. This removes the up-to-±half-GPS-period
        // offset between the fix stamp and the keyframe stamp — a heading-dependent bias (not
        // random noise), which is why inflating covariance can't fix it. Falls back to the raw
        // fix when there is no forward bracket within max_interp_dt.
        nav_msgs::Odometry accept_msg = gps_msg;
        if (p_.interpolate && !gps_queue_.empty())
        {
            const double t0 = gps_msg.header.stamp.toSec();
            const double t1 = gps_queue_.front().header.stamp.toSec();
            if (t0 <= kf_time && kf_time <= t1 && (t1 - t0) > 1e-6 && (t1 - t0) <= p_.max_interp_dt)
            {
                const nav_msgs::Odometry& nxt = gps_queue_.front();
                const double a = (kf_time - t0) / (t1 - t0);
                auto& P = accept_msg.pose.pose.position;
                P.x += a * (nxt.pose.pose.position.x - P.x);
                P.y += a * (nxt.pose.pose.position.y - P.y);
                P.z += a * (nxt.pose.pose.position.z - P.z);
                auto& C = accept_msg.pose.covariance;
                C[0]  += a * (nxt.pose.covariance[0]  - C[0]);
                C[7]  += a * (nxt.pose.covariance[7]  - C[7]);
                C[14] += a * (nxt.pose.covariance[14] - C[14]);
                ROS_DEBUG("[GPS] Interpolated fix to keyframe: a=%.2f over dt=%.3f s", a, t1 - t0);
            }
        }
        return acceptFixIntoGraph(accept_msg, *tier, fix_status,
                                  node_idx, traveled_dist, current_z,
                                  node_pose_corrected,
                                  hdg_has, hdg_yaw, graph_out);
    }

    // Buffer window expired → flush the best candidate.
    // Placed after the while loop so that all queued fixes up to kf_time are
    // collected as candidates before we decide which one to commit.
    if (buffering_ && kf_time >= buffer_start_time_ + p_.best_fix_window)
    {
        if (!fix_buffer_.empty())
        {
            const BufCandidate& best = *std::min_element(
                fix_buffer_.begin(), fix_buffer_.end(),
                [](const BufCandidate& a, const BufCandidate& b){ return a.cov_trace < b.cov_trace; });
            ROS_INFO("[GPS] Best-fix buffer: %zu candidates, selected cov_trace=%.4f",
                     fix_buffer_.size(), best.cov_trace);

            // Heading fallback for buffered candidates: all candidates are processed in the flush
            // keyframe so latest_heading_stamp_ is the same for all of them. A candidate from the
            // START of the buffer may be up to best_fix_window seconds older than the heading, so
            // the 50 ms per-candidate window systematically fails. Re-check for the selected
            // candidate with an extended window (best_fix_window + 50 ms). GPS compass heading
            // changes slowly enough that this timing slack is negligible.
            bool   hdg_has = best.has_hdg;
            double hdg_yaw = best.hdg_yaw;
            if (!hdg_has && p_.has_heading_topic)
            {
                std::lock_guard<std::mutex> hlk(heading_mutex_);
                const double gap = std::abs(latest_heading_stamp_ - best.gps.header.stamp.toSec());
                if (latest_heading_stamp_ > 0.0 && gap <= p_.best_fix_window + 0.05)
                {
                    hdg_has = true;
                    hdg_yaw = latest_heading_yaw_;
                    ROS_DEBUG("[GPS] Heading fallback for buffered candidate: gap=%.3f s (window=%.2f s)",
                              gap, p_.best_fix_window + 0.05);
                }
            }

            result = acceptFixIntoGraph(best.gps, best.tier, best.fix_status,
                                        node_idx, traveled_dist, current_z,
                                        node_pose_corrected,
                                        hdg_has, hdg_yaw, graph_out);
        }
        else
        {
            ROS_WARN("[GPS] Best-fix buffer expired with no valid candidates");
        }
        buffering_ = false;
        fix_buffer_.clear();
    }
    return result;
}

// ── makeGpsNoise (#2 robust kernel) ───────────────────────────────────────────

gtsam::noiseModel::Base::shared_ptr GpsHandler::makeGpsNoise(const gtsam::Vector3& var) const
{
    const gtsam::noiseModel::Base::shared_ptr base =
        gtsam::noiseModel::Diagonal::Variances(var);
    if (p_.robust_kernel.empty() || p_.robust_kernel == "none")
        return base;

    namespace mE = gtsam::noiseModel::mEstimator;
    mE::Base::shared_ptr m;
    const double k = p_.robust_thresh;
    if      (p_.robust_kernel == "huber")  m = mE::Huber::Create(k);
    else if (p_.robust_kernel == "cauchy") m = mE::Cauchy::Create(k);
    else if (p_.robust_kernel == "gm")     m = mE::GemanMcClure::Create(k);
    else if (p_.robust_kernel == "dcs")    m = mE::DCS::Create(k);
    else if (p_.robust_kernel == "tukey")  m = mE::Tukey::Create(k);
    else
    {
        ROS_WARN_ONCE("[GPS] Unknown robust_kernel '%s' -- using plain Gaussian noise",
                      p_.robust_kernel.c_str());
        return base;
    }
    return gtsam::noiseModel::Robust::Create(m, base);
}

// ── acceptFixIntoGraph ────────────────────────────────────────────────────────

GpsHandler::FactorResult GpsHandler::acceptFixIntoGraph(
    const nav_msgs::Odometry& gps_msg, const GpsFixTier& tier, int8_t fix_status,
    int node_idx, double traveled_dist, double current_z,
    const Eigen::Matrix4d& node_pose_corrected,
    bool hdg_has, double hdg_yaw,
    gtsam::NonlinearFactorGraph& graph_out)
{
    FactorResult result;

    float noise_x = static_cast<float>(gps_msg.pose.covariance[0]);
    float noise_y = static_cast<float>(gps_msg.pose.covariance[7]);
    float noise_z = static_cast<float>(gps_msg.pose.covariance[14]);

    float gps_x = static_cast<float>(gps_msg.pose.pose.position.x);
    float gps_y = static_cast<float>(gps_msg.pose.pose.position.y);
    float gps_z = static_cast<float>(gps_msg.pose.pose.position.z);

    if (!p_.use_elevation)
    {
        gps_z   = static_cast<float>(current_z);
        noise_z = 0.01f;
    }

    // ── Course-over-ground heading ────────────────────────────────────────────
    // Derive a yaw constraint from the bearing vector between the previous accepted
    // GPS fix and this one. Only active when use_cog_heading is set and no IMU
    // heading has already been paired for this fix.
    double cog_noise_var = p_.cog_noise_floor;
    if (p_.use_cog_heading && first_gps_added_ && !hdg_has)
    {
        const float dx   = gps_x - last_gps_point_.x;
        const float dy   = gps_y - last_gps_point_.y;
        const float dist = std::sqrt(dx * dx + dy * dy);
        if (dist > 0.1f)  // safety floor; normally dist >= min_spacing
        {
            hdg_yaw = std::atan2(static_cast<double>(dy), static_cast<double>(dx));
            hdg_has = true;
            // Propagate GPS position uncertainty into bearing uncertainty:
            //   σ_bearing ≈ σ_xy / dist  →  var ≈ (noise_x + noise_y) / (2 * dist²)
            cog_noise_var = std::max(p_.cog_noise_floor,
                                     static_cast<double>(noise_x + noise_y) /
                                         (2.0 * static_cast<double>(dist) * static_cast<double>(dist)));
        }
    }

    last_gps_point_  = pcl::PointXYZ(gps_x, gps_y, gps_z);
    first_gps_added_ = true;
    last_gps_fix_status_ = fix_status;

    if (p_.use_ground_prior)
    {
        result.ground_z_updated = true;
        result.new_ground_z     = static_cast<double>(gps_z);
    }

    if (gps_reentry_pending_)
    {
        result.lm_dist_gap   = traveled_dist - last_gps_accepted_path_length_;
        result.triggers_lm   = true;
        gps_reentry_pending_ = false;
        ROS_INFO("[GPS] Post-outage/startup fix accepted -- LM global refinement will run (dist_gap=%.1f m)",
                 result.lm_dist_gap);
    }
    else if (p_.lm_every_factor)
    {
        result.lm_dist_gap = 0.0;
        result.triggers_lm = true;
    }
    last_gps_accepted_path_length_ = traveled_dist;

    const float noise_floor = static_cast<float>(tier.noise_floor);
    const float cov_scale   = static_cast<float>(std::max(tier.cov_scale, 1e-3));
    const gtsam::Vector3 gps_noise_vec(std::max(noise_x * cov_scale, noise_floor),
                                       std::max(noise_y * cov_scale, noise_floor),
                                       std::max(noise_z * cov_scale, noise_floor));
    // #2 robust kernel + #7 lever arm. Apply the lever arm as a single pre-correction of
    // the GPS measurement: corrected = gps - R_node * arm.  A plain GPSFactor on
    // (corrected) is then equivalent to  pose.t == gps - R_node*arm  i.e. the
    // GPSArmFactor constraint  pose.t + R*arm == gps .  Only uses the current node yaw,
    // so the linearization matches the prior custom factor for well-converged rotations.
    float eff_x = gps_x, eff_y = gps_y, eff_z = gps_z;
    if (p_.lever_arm.squaredNorm() > 1e-12)
    {
        const Eigen::Matrix3d R = node_pose_corrected.block<3, 3>(0, 0);
        const Eigen::Vector3d arm = p_.lever_arm;
        const Eigen::Vector3d world_arm = R * arm;
        eff_x = gps_x - static_cast<float>(world_arm.x());
        eff_y = gps_y - static_cast<float>(world_arm.y());
        eff_z = gps_z - static_cast<float>(world_arm.z());
    }
    const gtsam::noiseModel::Base::shared_ptr gps_noise = makeGpsNoise(gps_noise_vec);
    graph_out.add(gtsam::GPSFactor(node_idx,
                                   gtsam::Point3(eff_x, eff_y, eff_z),
                                   gps_noise));

    gps_constraint_points_.push_back(pcl::PointXYZ(gps_x, gps_y, gps_z));
    gps_constraint_noises_.push_back(Eigen::Vector3f(static_cast<float>(gps_noise_vec[0]),
                                                      static_cast<float>(gps_noise_vec[1]),
                                                      static_cast<float>(gps_noise_vec[2])));
    gps_constraint_fix_status_.push_back(fix_status);

    ROS_INFO("\033[1;36m[GPS] Factor @node %d  pos=(%.1f, %.1f, %.1f)"
             "  sig_raw=(%.3f, %.3f)  sig_eff=(%.3f, %.3f, %.3f)\033[0m",
             node_idx, gps_x, gps_y, gps_z, noise_x, noise_y,
             static_cast<float>(gps_noise_vec[0]),
             static_cast<float>(gps_noise_vec[1]),
             static_cast<float>(gps_noise_vec[2]));

    if ((p_.has_heading_topic || p_.use_cog_heading) && hdg_has)
    {
        const bool using_cog = p_.use_cog_heading && !p_.has_heading_topic;
        const double hdg_var = using_cog ? cog_noise_var : p_.heading_noise_floor;
        auto yaw_noise = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e6, 1e6, hdg_var, 1e6, 1e6, 1e6).finished());
        graph_out.add(gtsam::PriorFactor<gtsam::Pose3>(
            node_idx,
            gtsam::Pose3(gtsam::Rot3::Rz(hdg_yaw), gtsam::Point3(gps_x, gps_y, gps_z)),
            yaw_noise));
        ROS_INFO("\033[1;33m[GPS] %s heading factor at node %d yaw=%.1f deg (var=%.4f rad^2)\033[0m",
                 using_cog ? "COG" : "IMU", node_idx, hdg_yaw * 180.0 / M_PI, hdg_var);
    }
    else if (p_.has_heading_topic)
    {
        ROS_WARN("[GPS] Heading NOT paired at node %d -- yaw unconstrained at this GPS fix", node_idx);
    }

    result.factor_added = true;
    return result;
}

// ── Visualization ──────────────────────────────────────────────────────────────

visualization_msgs::MarkerArray GpsHandler::getGpsMarkers(const std::string& map_frame) const
{
    visualization_msgs::MarkerArray ma;

    // One sphere per accepted GPS constraint; diameter = 2σ per axis.
    for (size_t i = 0; i < gps_constraint_points_.size(); ++i)
    {
        const auto& pt = gps_constraint_points_[i];
        float sx = 0.5f, sy = 0.5f, sz = 0.5f;
        if (i < gps_constraint_noises_.size())
        {
            sx = std::max(0.1f, std::sqrt(std::max(gps_constraint_noises_[i].x(), 0.0f)));
            sy = std::max(0.1f, std::sqrt(std::max(gps_constraint_noises_[i].y(), 0.0f)));
            sz = std::max(0.1f, std::sqrt(std::max(gps_constraint_noises_[i].z(), 0.0f)));
        }
        visualization_msgs::Marker node;
        node.header.frame_id     = map_frame;
        node.ns                  = "gps_nodes";
        node.id                  = static_cast<int>(i);
        node.type                = visualization_msgs::Marker::SPHERE;
        node.action              = visualization_msgs::Marker::ADD;
        node.pose.orientation.w  = 1.0;
        node.pose.position.x     = pt.x;
        node.pose.position.y     = pt.y;
        node.pose.position.z     = pt.z;
        node.scale.x = 2.0f * sx;
        node.scale.y = 2.0f * sy;
        node.scale.z = 2.0f * sz;
        // GBAS (RTK): bright green  SBAS (WAAS): yellowish-green
        const bool is_gbas = (i < gps_constraint_fix_status_.size()) &&
                             (gps_constraint_fix_status_[i] == sensor_msgs::NavSatStatus::STATUS_GBAS_FIX);
        if (is_gbas)
            { node.color.r = 0.0f; node.color.g = 0.9f; node.color.b = 0.1f; node.color.a = 0.6f; }
        else
            { node.color.r = 0.9f; node.color.g = 0.9f; node.color.b = 0.0f; node.color.a = 0.5f; }
        ma.markers.push_back(node);
    }

    // Line strip connecting consecutive GPS constraint points.
    if (gps_constraint_points_.size() >= 2)
    {
        visualization_msgs::Marker edges;
        edges.header.frame_id    = map_frame;
        edges.ns                 = "gps_edges";
        edges.id                 = 0;
        edges.type               = visualization_msgs::Marker::LINE_STRIP;
        edges.action             = visualization_msgs::Marker::ADD;
        edges.pose.orientation.w = 1.0;
        edges.scale.x            = 0.15;
        edges.color.r = 0.1f; edges.color.g = 0.85f; edges.color.b = 0.05f; edges.color.a = 0.8f;
        for (const auto& pt : gps_constraint_points_)
        {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            edges.points.push_back(p);
        }
        ma.markers.push_back(edges);
    }

    return ma;
}
