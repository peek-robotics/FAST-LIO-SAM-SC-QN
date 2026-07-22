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
    double heading_noise_floor   = 0.05;   ///< [rad²] noise floor for heading PriorFactor added alongside GPS factor

    // Course-over-ground heading (derived from consecutive accepted GPS XY positions)
    bool   use_cog_heading = false;
    double cog_noise_floor = 0.05; ///< [rad²] noise floor for COG heading PriorFactor (also used as floor when propagating from GPS XY noise)

    // Best-fix buffering
    double best_fix_window = 0.0;  ///< [s] after spacing gate passes, buffer fixes for this long then commit the best (0 = off)

    // LM triggers
    bool lm_every_factor = false;  ///< run LM on every accepted GPS factor, not just re-entry

    // Ground prior
    bool use_ground_prior = false;

    // Robust kernel on the GPS position factor (#2) — down-weights outlier fixes
    // (multipath under canopy, EKF glitches) instead of letting them yank a node.
    std::string robust_kernel = "none";  ///< none|huber|cauchy|gm|dcs|tukey
    double      robust_thresh = 1.345;   ///< kernel width in WHITENED (sigma) units, not metres

    // Temporal alignment (#4/#6)
    double time_offset   = 0.0;   ///< [s] added to GPS/fix stamps at ingestion to align them to the LIO clock
    bool   interpolate   = true;  ///< linearly interpolate GPS position/cov to the exact keyframe timestamp
    double max_interp_dt = 0.30;  ///< [s] max bracket gap for interpolation; beyond this, use the nearest fix

    // GPS lever arm: the GPS-reported point expressed in the SLAM body (node) frame [m].
    // Applied as a pre-correction of the measurement before adding a plain GPSFactor:
    //   corrected_xyz = gps_xyz - R_node * arm
    // Equivalently constrains  pose.t == gps - R*arm , i.e.  pose.t + R*arm == gps .
    // [0,0,0] on Grover (the SLAM node frame == the frame robot_localization reports GPS in).
    Eigen::Vector3d lever_arm = Eigen::Vector3d::Zero();
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
    /// graph_out.  node_pose_corrected is the current estimated pose of the new
    /// keyframe node (4x4 homogeneous) — used to rotate the lever arm into the
    /// world frame for the measurement pre-correction.  Returns metadata about
    /// what was added.
    FactorResult tryAddFactor(double kf_time, int node_idx,
                              double traveled_dist,
                              const Eigen::Matrix4d& node_pose_corrected,
                              gtsam::NonlinearFactorGraph& graph_out);

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
    bool   datum_set_            = false;  ///< PHASE 2: init WGS-84 datum captured -> GPS odom reprojected to true-north local ENU (onGpsOdom)

    // Latest cached heading from IMU heading topic
    mutable std::mutex heading_mutex_;
    double latest_heading_yaw_   = 0.0;
    double latest_heading_stamp_ = 0.0;

    // Best-fix buffer
    struct BufCandidate
    {
        nav_msgs::Odometry gps;
        GpsFixTier         tier;
        int8_t             fix_status = sensor_msgs::NavSatStatus::STATUS_FIX;
        double             cov_trace  = 0.0;   ///< noise_x + noise_y (lower = better)
        double             hdg_yaw    = 0.0;
        bool               has_hdg    = false;
    };
    bool                      buffering_         = false;
    double                    buffer_start_time_ = 0.0;
    std::vector<BufCandidate> fix_buffer_;

    // GPS factor bookkeeping (promoted from function-static locals in original code)
    bool          first_gps_added_               = false;
    pcl::PointXYZ last_gps_point_                = {0.f, 0.f, 0.f};
    int8_t        last_gps_fix_status_           = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
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

    /// PHASE 2: nearest raw NavSatFix lat/lon/alt to time `t` (±0.15 s), used to
    /// reproject GPS odom into the true-north local-ENU datum frame. Also prunes
    /// stale fixes. Caller holds gps_mutex_; locks fix_mutex_ (order as resolveFixTier).
    bool nearestFixLL(double t, double& lat, double& lon, double& alt);

    /// Build the GPS position-factor noise model: diagonal variances, optionally
    /// wrapped in a robust m-estimator per robust_kernel/robust_thresh (#2).
    gtsam::noiseModel::Base::shared_ptr makeGpsNoise(const gtsam::Vector3& var) const;

    /// Returns false and logs if the current fix type is below the init gate.
    bool isReadyForInitSample() const;

    /// Joint GPS+heading stability window (used when heading_topic is configured).
    void runJointInitWindow(double yaw);

    /// GPS-only position stability window (used when no heading_topic).
    bool runPositionWindowCheck(double x, double y, double z);

    /// Snapshot the init values and set gps_heading_received_ = true.
    void finalizeInit(double yaw, double x, double y, double z);

    /// Accept a single GPS fix (all gates already passed) into the factor graph.
    /// Updates all bookkeeping. Caller holds gps_mutex_.
    /// node_pose_corrected is the current estimated pose of the keyframe node
    /// (4x4 homogeneous, base_footprint frame) — used to rotate the lever arm into
    /// the world frame for the measurement pre-correction.
    FactorResult acceptFixIntoGraph(const nav_msgs::Odometry& gps_msg, const GpsFixTier& tier,
                                    int8_t fix_status, int node_idx,
                                    double traveled_dist, double current_z,
                                    const Eigen::Matrix4d& node_pose_corrected,
                                    bool hdg_has, double hdg_yaw,
                                    gtsam::NonlinearFactorGraph& graph_out);
};
