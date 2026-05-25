#include "gps_handler.h"

#include <algorithm>
#include <cmath>

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

GpsHandler::HeadingSnapshot GpsHandler::consumeHeading()
{
    std::lock_guard<std::mutex> lk(heading_mutex_);
    HeadingSnapshot s{latest_heading_fresh_, latest_heading_yaw_, latest_heading_cov_, latest_heading_stamp_};
    latest_heading_fresh_ = false;
    return s;
}

// ── Subscriber callbacks ───────────────────────────────────────────────────────

void GpsHandler::onNavSatFix(const sensor_msgs::NavSatFixConstPtr& msg)
{
    latest_fix_status_.store(msg->status.status);
    {
        std::lock_guard<std::mutex> lk(fix_mutex_);
        fix_queue_.push_back(*msg);
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
        gps_queue_.push_back(*msg);
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

    // Always cache the latest yaw for continuous heading PriorFactors.
    {
        std::lock_guard<std::mutex> lk(heading_mutex_);
        latest_heading_yaw_   = yaw;
        latest_heading_cov_   = msg->orientation_covariance[8];
        latest_heading_stamp_ = msg->header.stamp.toSec();
        latest_heading_fresh_ = true;
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
            ROS_INFO_THROTTLE(2.0, "[Init] Yaw unstable (range=%.2f° >= tol=%.2f°) — clearing window",
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
            ROS_INFO_THROTTLE(2.0, "[Init] GPS position unstable (range=%.3f m >= tol=%.3f m) — clearing window",
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
             "yaw=%.1f°  pos=(%.2f, %.2f, %.2f) — SLAM init unblocked.\033[0m",
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
        ROS_INFO_THROTTLE(2.0, "[GPS] Position unstable (range=%.3f m >= tol=%.3f m) — clearing window",
                          range, p_.stable_tol_pos_m);
        init_window_.clear();
        return false;
    }

    finalizeInit(std::numeric_limits<double>::quiet_NaN(), x, y, z);
    ROS_INFO("\033[1;32m[Init] GPS position stable over %d samples: pos=(%.2f, %.2f, %.2f) — SLAM init unblocked.\033[0m",
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
        ROS_WARN_THROTTLE(5.0, "[GPS] No NavSatFix within ±0.1 s of GPS odometry — rejecting factor");
        return nullptr;
    }

    auto it = p_.fix_tiers.find(best_status);
    if (it == p_.fix_tiers.end())
    {
        ROS_WARN_THROTTLE(5.0, "[GPS] Fix status %d below minimum accepted tier (SBAS=1, GBAS=2) — rejecting",
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
    double traveled_dist, double current_z,
    gtsam::NonlinearFactorGraph& graph_out)
{
    FactorResult result;

    std::lock_guard<std::mutex> lk(gps_mutex_);
    if (gps_queue_.empty())
        return result;

    // Traveled-distance gate (bypass for the very first GPS factor).
    if (!first_gps_added_)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] First GPS factor — bypassing traveled-dist gate to anchor map origin");
    }
    else if (traveled_dist < p_.min_traveled_dist)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: traveled dist %.2f m < %.1f m threshold",
                          traveled_dist, p_.min_traveled_dist);
        return result;
    }

    // Time-sync: GPS @ 5 Hz → max inter-message gap 0.2 s, so ±0.1 s keeps us within
    // half a GPS period of the keyframe and avoids applying a fix from a different pose.
    static constexpr double kGpsSyncWindow = 0.1;
    while (!gps_queue_.empty())
    {
        const double msg_time = gps_queue_.front().header.stamp.toSec();
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

        const nav_msgs::Odometry gps_msg = gps_queue_.front();
        gps_queue_.pop_front();

        // Resolve quality tier.
        const GpsFixTier* tier = resolveFixTier(gps_msg.header.stamp.toSec());
        if (!tier)
            continue;

        const float noise_x = static_cast<float>(gps_msg.pose.covariance[0]);
        const float noise_y = static_cast<float>(gps_msg.pose.covariance[7]);
        float noise_z       = static_cast<float>(gps_msg.pose.covariance[14]);

        if (std::abs(noise_x) > static_cast<float>(tier->cov_gate) ||
            std::abs(noise_y) > static_cast<float>(tier->cov_gate) ||
            std::abs(noise_z) > static_cast<float>(tier->cov_gate))
        {
            ROS_INFO_THROTTLE(5.0, "[GPS] Rejected: position cov (%.4f, %.4f, %.4f) > gate %.4f",
                              noise_x, noise_y, noise_z, tier->cov_gate);
            continue;
        }

        float gps_x = static_cast<float>(gps_msg.pose.pose.position.x);
        float gps_y = static_cast<float>(gps_msg.pose.pose.position.y);
        float gps_z = static_cast<float>(gps_msg.pose.pose.position.z);

        if (!p_.use_elevation)
        {
            gps_z   = static_cast<float>(current_z);
            noise_z = 0.01f;
        }

        if (std::abs(gps_x) < 1e-6f && std::abs(gps_y) < 1e-6f)
        {
            ROS_WARN_THROTTLE(5.0, "[GPS] Rejected: position at origin (uninitialized)");
            continue;
        }

        // Spacing and re-entry checks.
        const float dist_from_last = std::hypot(gps_x - last_gps_point_.x, gps_y - last_gps_point_.y);
        if (first_gps_added_)
        {
            if (dist_from_last < static_cast<float>(p_.min_spacing))
            {
                ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: %.2f m from last GPS point (< %.1f m)",
                                  dist_from_last, p_.min_spacing);
                continue;
            }

            if (dist_from_last > 2.0f * static_cast<float>(p_.min_spacing) && re_entry_skip_remaining_ == 0)
            {
                re_entry_skip_remaining_ = p_.re_entry_skip_count;
                last_gps_point_ = pcl::PointXYZ(gps_x, gps_y, gps_z);
                gps_reentry_pending_ = true;
                ROS_WARN("[GPS] Re-entry detected (%.1f m gap) — discarding next %d fix(es)",
                         dist_from_last, re_entry_skip_remaining_);
            }
            if (re_entry_skip_remaining_ > 0)
            {
                re_entry_skip_remaining_--;
                ROS_INFO("[GPS] Re-entry: discarding fix (%d remaining)", re_entry_skip_remaining_);
                continue;
            }
        }

        // Accept the fix.
        last_gps_point_ = pcl::PointXYZ(gps_x, gps_y, gps_z);
        first_gps_added_ = true;

        if (p_.use_ground_prior)
        {
            result.ground_z_updated = true;
            result.new_ground_z     = static_cast<double>(gps_z);
        }

        // Trigger B+C global LM refinement on startup and after outage re-entry.
        if (gps_reentry_pending_)
        {
            result.lm_dist_gap   = traveled_dist - last_gps_accepted_path_length_;
            result.triggers_lm   = true;
            gps_reentry_pending_ = false;
            ROS_INFO("[GPS] Post-outage/startup fix accepted — LM global refinement will run (dist_gap=%.1f m)",
                     result.lm_dist_gap);
        }
        else if (p_.lm_every_factor)
        {
            result.lm_dist_gap = 0.0;
            result.triggers_lm = true;
        }
        last_gps_accepted_path_length_ = traveled_dist;

        // Build GPS position factor.
        const float noise_floor = static_cast<float>(tier->noise_floor);
        const float cov_scale   = static_cast<float>(std::max(tier->cov_scale, 1e-3));
        const gtsam::Vector3 gps_noise_vec(std::max(noise_x * cov_scale, noise_floor),
                                           std::max(noise_y * cov_scale, noise_floor),
                                           std::max(noise_z * cov_scale, noise_floor));
        graph_out.add(gtsam::GPSFactor(node_idx,
                                       gtsam::Point3(gps_x, gps_y, gps_z),
                                       gtsam::noiseModel::Diagonal::Variances(gps_noise_vec)));

        gps_constraint_points_.push_back(pcl::PointXYZ(gps_x, gps_y, gps_z));
        gps_constraint_noises_.push_back(Eigen::Vector3f(static_cast<float>(gps_noise_vec[0]),
                                                          static_cast<float>(gps_noise_vec[1]),
                                                          static_cast<float>(gps_noise_vec[2])));
        {
            int8_t fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
            for (const auto& kv : p_.fix_tiers)
                if (&kv.second == tier) { fix_status = kv.first; break; }
            gps_constraint_fix_status_.push_back(fix_status);
        }

        ROS_INFO("\033[1;36m[GPS] Factor @node %d  pos=(%.1f, %.1f, %.1f)"
                 "  σ_raw=(%.3f, %.3f)  σ_eff=(%.3f, %.3f, %.3f)\033[0m",
                 node_idx, gps_x, gps_y, gps_z, noise_x, noise_y,
                 static_cast<float>(gps_noise_vec[0]),
                 static_cast<float>(gps_noise_vec[1]),
                 static_cast<float>(gps_noise_vec[2]));

        // Optional GPS-derived heading factor (from robot_localization yaw covariance).
        // covariance[35] = pose.covariance[5*6+5] = yaw variance [rad²]
        const double heading_cov = gps_msg.pose.covariance[35];
        if (heading_cov <= 1e-9 || heading_cov >= p_.heading_cov_gate)
            ROS_INFO_THROTTLE(5.0, "[GPS] No heading factor at node %d: cov=%.4f (gate=%.4f)",
                              node_idx, heading_cov, p_.heading_cov_gate);
        if (heading_cov > 1e-9 && heading_cov < p_.heading_cov_gate)
        {
            const auto& quat = gps_msg.pose.pose.orientation;
            const double gps_yaw  = gtsam::Rot3::Quaternion(quat.w, quat.x, quat.y, quat.z).yaw();
            const double h_noise  = std::max(heading_cov, p_.heading_noise_floor);
            // Tight on yaw (Rz) only; very loose on roll, pitch, x, y, z.
            auto yaw_noise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e6, 1e6, h_noise, 1e6, 1e6, 1e6).finished());
            graph_out.add(gtsam::PriorFactor<gtsam::Pose3>(
                node_idx,
                gtsam::Pose3(gtsam::Rot3::Rz(gps_yaw), gtsam::Point3(gps_x, gps_y, gps_z)),
                yaw_noise));
            ROS_INFO("\033[1;33m[GPS] Heading factor at node %d yaw=%.1f° (cov=%.4f rad²)\033[0m",
                     node_idx, gps_yaw * 180.0 / M_PI, heading_cov);
        }

        result.factor_added = true;
        return result;
    }
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
            { node.color.r = 0.6f; node.color.g = 0.9f; node.color.b = 0.0f; node.color.a = 0.5f; }
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
