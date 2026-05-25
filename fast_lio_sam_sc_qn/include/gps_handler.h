#pragma once

#include <atomic>
#include <deque>
#include <limits>
#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Eigen>
#include <pcl/point_types.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <visualization_msgs/MarkerArray.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/PriorFactor.h>

/// Per-fix-type GPS quality parameters (covariance gate, noise floor, covariance scale).
struct GpsFixTier
{
    double cov_gate    = 0.1;  ///< [m²] per-axis covariance gate: reject if abs(cov) > this
    double noise_floor = 1.0;  ///< [m²] per-axis noise floor clamped into GTSAM factor
    double cov_scale   = 1.0;  ///< multiplicative inflate applied post-gate, post-floor
};

/// All GPS-related configuration, populated once by loadParams() in the main class.
struct GpsParams
{
    // Quality tiers
    GpsFixTier default_tier;
    bool use_fix_tiers         = false;
    std::map<int8_t, GpsFixTier> fix_tiers;
    bool require_gbas_for_init = false; ///< only allow GBAS_FIX for the init stability window

    // Covariance gates
    bool   use_elevation       = true;  ///< use GPS Z; false = substitute LIO Z

    // Spacing / traveled-distance gates
    double min_spacing         = 5.0;   ///< [m] min distance between accepted GPS factors
    double min_traveled_dist   = 5.0;   ///< [m] min path length before first GPS factor
    int    re_entry_skip_count = 2;     ///< post-outage fixes to discard

    // Init stability window
    bool   has_heading_topic   = false;
    int    stable_count        = 5;
    double stable_tol_yaw      = 0.05;  ///< [rad] max circular yaw range in window (~3°)
    double stable_tol_pos_m    = 1.0;   ///< [m]   max XY range in position window

    // Heading / yaw factors
    double heading_cov_gate      = 0.1;    ///< [rad²] max yaw cov accepted from GPS odom msg
    double heading_noise_floor   = 0.01;   ///< [rad²] noise floor for GPS-derived yaw factor
    double heading_factor_noise  = 0.0076; ///< [rad²] default IMU heading topic factor noise
    double heading_velocity_gate = 0.0;    ///< [m/s]  skip heading factor if |v_linear| exceeds this; 0 = disabled

    // LM triggers
    bool lm_every_factor = false;  ///< run LM on every accepted GPS factor, not just re-entry

    // Ground prior
    bool use_ground_prior = false;
};

///
/// GpsHandler — owns all GPS state and logic:
///   - GPS position queue + fix-type tier resolution
///   - Joint GPS+heading init stability window
///   - Per-keyframe GPS factor construction
///   - Continuous heading caching for PriorFactors
///   - GPS constraint visualization data
///
class GpsHandler
{
public:
    /// Result returned by tryAddFactor().
    struct FactorResult
    {
        bool   factor_added    = false;
        bool   triggers_lm     = false;
        double lm_dist_gap     = 0.0;
        bool   ground_z_updated = false;
        double new_ground_z    = 0.0;
    };

    /// Snapshot of GPS+heading captured at init time.
    struct InitSnapshot
    {
        double x   = 0.0;
        double y   = 0.0;
        double z   = 0.0;
        double yaw = std::numeric_limits<double>::quiet_NaN(); ///< NaN = no heading available
        double lat = std::numeric_limits<double>::quiet_NaN(); ///< WGS-84 latitude  [deg]
        double lon = std::numeric_limits<double>::quiet_NaN(); ///< WGS-84 longitude [deg]
    };

    /// Heading reading consumed once per keyframe.
    struct HeadingSnapshot
    {
        bool   fresh = false;
        double yaw   = 0.0;
        double cov   = 0.0;
        double stamp = 0.0;  ///< header.stamp of the source IMU message [s]
    };

    explicit GpsHandler(const GpsParams& p);

    /// Read-only access to loaded parameters.
    const GpsParams& params() const { return p_; }

    // ── Subscriber callbacks ─────────────────────────────────────────────────
    void onGpsOdom(const nav_msgs::OdometryConstPtr& msg);
    void onNavSatFix(const sensor_msgs::NavSatFixConstPtr& msg);
    void onHeading(const sensor_msgs::ImuConstPtr& msg);

    // ── Init readiness ────────────────────────────────────────────────────────
    bool isReadyToInit() const;
    InitSnapshot getInitSnapshot() const;

    // ── Per-keyframe GPS factor ───────────────────────────────────────────────
    /// Attempts to add GPS position (and optional GPS-derived yaw) factors into
    /// graph_out.  Returns metadata about what was added.
    FactorResult tryAddFactor(double kf_time, int node_idx,
                              double traveled_dist, double current_z,
                              gtsam::NonlinearFactorGraph& graph_out);

    // ── Continuous heading factor (IMU heading topic) ─────────────────────────
    /// Returns and clears the latest cached heading reading.
    HeadingSnapshot consumeHeading();

    // ── Visualization ─────────────────────────────────────────────────────────
    visualization_msgs::MarkerArray getGpsMarkers(const std::string& map_frame) const;
    bool hasGpsConstraints() const;

    // ── State queries ─────────────────────────────────────────────────────────
    double latestGpsZ() const;
    bool   firstReceived() const;
    void   armReentry(); ///< re-arm GPS re-entry flag (call after LIO jump recovery)

private:
    GpsParams p_;

    // GPS position queue
    std::deque<nav_msgs::Odometry> gps_queue_;
    mutable std::mutex gps_mutex_;

    // NavSatFix queue (for fix-type tier resolution)
    std::deque<sensor_msgs::NavSatFix> fix_queue_;
    mutable std::mutex fix_mutex_;
    std::atomic<int8_t> latest_fix_status_{-1};

    // Live GPS state
    bool   gps_first_received_ = false;
    double latest_gps_x_       = 0.0;
    double latest_gps_y_       = 0.0;
    double latest_gps_z_       = 0.0;
    double latest_lat_         = std::numeric_limits<double>::quiet_NaN();
    double latest_lon_         = std::numeric_limits<double>::quiet_NaN();

    // Init stability window
    struct InitSample { double yaw, x, y, z; };
    std::deque<InitSample> init_window_;
    std::deque<double>     init_yaw_buf_;
    bool   gps_heading_received_ = false;
    double gps_initial_yaw_      = std::numeric_limits<double>::quiet_NaN();
    double init_gps_x_           = 0.0;
    double init_gps_y_           = 0.0;
    double init_gps_z_           = 0.0;
    double init_lat_             = std::numeric_limits<double>::quiet_NaN();
    double init_lon_             = std::numeric_limits<double>::quiet_NaN();

    // Continuous heading state (IMU heading topic)
    mutable std::mutex heading_mutex_;
    double latest_heading_yaw_   = 0.0;
    double latest_heading_cov_   = 0.0;
    double latest_heading_stamp_ = 0.0;
    bool   latest_heading_fresh_ = false;

    // GPS factor bookkeeping (promoted from function-static locals in original code)
    bool          first_gps_added_               = false;
    pcl::PointXYZ last_gps_point_                = {0.f, 0.f, 0.f};
    int           re_entry_skip_remaining_       = 0;
    double        last_gps_accepted_path_length_ = 0.0;
    bool          gps_reentry_pending_           = true;

    // Visualization data
    std::vector<pcl::PointXYZ>   gps_constraint_points_;
    std::vector<Eigen::Vector3f> gps_constraint_noises_;
    std::vector<int8_t>          gps_constraint_fix_status_;  // STATUS_GBAS_FIX=2, STATUS_SBAS_FIX=1

    // ── Private helpers ───────────────────────────────────────────────────────
    /// Looks up the quality tier for a GPS odometry message.
    /// Caller MUST hold gps_mutex_ (because this also acquires fix_mutex_).
    const GpsFixTier* resolveFixTier(double msg_time);

    /// Returns false and logs if the current fix type is below the init gate.
    bool isReadyForInitSample() const;

    /// Joint GPS+heading stability window (used when heading_topic is configured).
    void runJointInitWindow(double yaw);

    /// GPS-only position stability window (used when no heading_topic).
    bool runPositionWindowCheck(double x, double y, double z);

    /// Snapshot the init values and set gps_heading_received_ = true.
    void finalizeInit(double yaw, double x, double y, double z);
};
