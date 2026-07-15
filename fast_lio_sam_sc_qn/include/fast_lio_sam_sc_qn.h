#ifndef FAST_LIO_SAM_SC_QN_MAIN_H
#define FAST_LIO_SAM_SC_QN_MAIN_H

#include <ctime>
#include <cmath>
#include <chrono>
#include <fstream>
#include <filesystem>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <grover_msgs/SrvInt16.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>
#include <voxel_slam/LIODiag.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>

#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"
#include "z_height_factor.h"
#include "gps_handler.h"
#include "isam_backend.h"

namespace fs = std::filesystem;
using namespace std::chrono;

typedef message_filters::sync_policies::ApproximateTime<
    nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

class FastLioSamScQn
{
public:
    explicit FastLioSamScQn(const ros::NodeHandle& n_private);
    ~FastLioSamScQn();

private:
    // ── Basic config ─────────────────────────────────────────────────────────
    std::string map_frame_, robot_frame_, package_path_, seq_name_;
    double keyframe_thr_, voxel_res_;
    bool publish_tf_, init_from_tf_;

    // ── LIO health ───────────────────────────────────────────────────────────
    double max_odom_jump_m_;
    double lio_cov_threshold_;
    bool reinit_on_jump_;
    int  reinit_skip_frames_               = 10;
    int  post_reinit_frames_remaining_     = 0;
    int  degrade_accept_max_               = 8;      ///< accept voxel_slam degrade_state <= this (downweighted); drop above. 8=High, drop Reset(16)
    bool bridge_after_reinit_              = false;  ///< first keyframe after a reinit: loosen its odom factor
    bool input_pcd_lidar_frame_            = false;  // true if input_pcd is already in LiDAR frame
    // Static robot_frame(base_footprint) -> lidar_frame extrinsic. voxel_slam odometry is in the
    // LiDAR frame, but the graph node/GPS/output frame is base_footprint. When clouds are stored
    // in the LiDAR frame we compose this at render time so map points land at their true position
    // (fixes the heading-dependent "doubled trunks"). The node pose itself stays base_footprint.
    std::string     lidar_frame_           = "livox_frame";
    Eigen::Matrix4d T_base_lidar_          = Eigen::Matrix4d::Identity();
    bool            lidar_extrinsic_ready_ = false;

    // ── Runtime state ────────────────────────────────────────────────────────
    bool is_initialized_      = false;
    bool first_odom_received_ = false;
    std::atomic<uint8_t> latest_diag_state_{1};
    bool loop_added_flag_vis_ = false;

    // ── Keyframes ────────────────────────────────────────────────────────────
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    mutable std::mutex   keyframes_mutex_;
    int current_keyframe_idx_ = 0;

    // ── Realtime pose ────────────────────────────────────────────────────────
    mutable std::mutex   realtime_pose_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_          = Eigen::Matrix4d::Identity();
    gtsam::Values   corrected_esti_;
    Eigen::MatrixXd pose_covariance_     = Eigen::MatrixXd::Identity(6, 6) * 1000.0;

    // ── Accumulated GPS path length (for min_traveled_dist gate) ─────────────
    double gps_total_path_length_ = 0.0;

    // ── Ground-plane Z prior ─────────────────────────────────────────────────
    bool   use_ground_prior_   = false;
    bool   ground_z_ready_     = false;
    double ground_z_ref_       = 0.0;
    double ground_prior_sigma_ = 1.0;
    gtsam::noiseModel::Base::shared_ptr ground_prior_noise_;

    // ── Init ─────────────────────────────────────────────────────────────────
    double init_prior_noise_z_ = 1.0;
    double init_prior_noise_yaw_unknown_ = 1.0;  ///< [rad^2] loose yaw prior when no GPS/heading yaw at init
    Eigen::Matrix4d tf_at_heading_pose_ = Eigen::Matrix4d::Identity();
    bool tf_at_heading_valid_  = false;
    bool heading_tf_captured_  = false;

    // ── Visualization ────────────────────────────────────────────────────────
    mutable std::mutex vis_mutex_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;

    // ── Save ─────────────────────────────────────────────────────────────────
    bool save_map_pcd_ = false;
    double init_lat_   = std::numeric_limits<double>::quiet_NaN();
    double init_lon_   = std::numeric_limits<double>::quiet_NaN();
    double init_alt_   = std::numeric_limits<double>::quiet_NaN();
    double init_x_     = std::numeric_limits<double>::quiet_NaN();
    double init_y_     = std::numeric_limits<double>::quiet_NaN();

    // ── Subsystems ───────────────────────────────────────────────────────────
    std::shared_ptr<LoopClosure> loop_closure_;
    GpsHandler  gps_handler_;
    IsamBackend isam_backend_;

    // ── ROS infrastructure ───────────────────────────────────────────────────
    ros::NodeHandle nh_;
    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener    tf_listener_;

    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, corrected_pcd_map_pub_, loop_detection_pub_;
    ros::Publisher realtime_pose_pub_, slam_odom_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    bool pub_debug_clouds_ = false;
    ros::Publisher gps_constraint_pub_;

    ros::Subscriber sub_save_flag_, sub_gps_, sub_gps_fix_, sub_heading_, sub_lio_diag_;
    ros::ServiceServer lm_refine_srv_, save_map_srv_;
    ros::ServiceClient to_ll_client_;
    ros::Timer loop_timer_, vis_timer_;

    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>>  sub_odom_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_;

    // ── Cloud sparsification ─────────────────────────────────────────────────
    bool              cloud_sparsify_en_  = false;
    double            cloud_sparsify_res_ = 0.3;
    int               cloud_sparsify_age_ = 30;
    std::thread       cloud_sparsify_thread_;
    std::atomic<bool> cloud_sparsify_stop_{false};

    // ── Performance monitoring ────────────────────────────────────────────────
    bool           show_perf_stats_       = false;
    static constexpr double perf_report_interval_ = 30.0;
    ros::WallTimer perf_timer_;
    ros::WallTime  perf_init_time_;
    mutable std::mutex perf_mutex_;
    double         perf_kf_time_sum_ms_   = 0.0;
    double         perf_kf_time_max_ms_   = 0.0;
    uint64_t       perf_kf_count_window_  = 0;
    double         perf_loop_time_sum_ms_ = 0.0;
    double         perf_loop_time_max_ms_ = 0.0;
    uint64_t       perf_loop_count_window_= 0;
    std::atomic<uint64_t> perf_frames_total_{0};
    std::atomic<uint64_t> perf_frames_dropped_{0};
    std::atomic<uint64_t> perf_gps_accepted_{0};

    // ── Constructor helpers ───────────────────────────────────────────────────
    static GpsParams     loadGpsParams(const ros::NodeHandle& nh);
    static BackendParams loadBackendParams(const ros::NodeHandle& nh);
    void loadParams(LoopClosureConfig& lc_config, double& loop_hz, double& vis_hz,
                    std::string& gps_topic, std::string& fix_topic, std::string& heading_topic);
    void setupRos(double loop_hz, double vis_hz,
                  const std::string& gps_topic, const std::string& fix_topic,
                  const std::string& heading_topic);
    void initComponents(const LoopClosureConfig& lc_config);

    // ── Callbacks ────────────────────────────────────────────────────────────
    void odomPcdCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                         const sensor_msgs::PointCloud2ConstPtr& pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr& msg);
    void loopTimerFunc(const ros::TimerEvent& event);
    void visTimerFunc(const ros::TimerEvent& event);
    void lioDiagCallback(const voxel_slam::LIODiagConstPtr& msg);
    bool lmRefineSrvCallback(grover_msgs::SrvInt16::Request&, grover_msgs::SrvInt16::Response&);
    bool saveMapSrvCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response&);
    void perfTimerFunc(const ros::WallTimerEvent& event);

    // ── odomPcdCallback decomposition ────────────────────────────────────────
    bool passLioHealthChecks(const nav_msgs::OdometryConstPtr& odom_msg,
                             const Eigen::Matrix4d& last_odom_tf);
    void publishRealtimePose(const nav_msgs::OdometryConstPtr& odom_msg,
                             const Eigen::Matrix4d& last_odom_tf);
    void tryInitialize();
    void processKeyframe(const nav_msgs::OdometryConstPtr& odom_msg);

    // ── Utilities ────────────────────────────────────────────────────────────
    void updateOdomsAndPaths(const PosePcd& pose_pcd_in);
    bool checkIfKeyframe(const PosePcd& a, const PosePcd& b) const;
    void cloudSparsifyThread();

    /// Lazily look up + cache the static robot_frame_->lidar_frame_ transform (idempotent).
    void ensureLidarExtrinsic(double wait_s, bool verbose);
    /// Pose to transform a stored keyframe cloud into the map: node pose, with the base->lidar
    /// extrinsic composed when clouds are in the LiDAR frame.
    Eigen::Matrix4d renderPose(const Eigen::Matrix4d& node_pose) const;

    // ── Save map helpers ──────────────────────────────────────────────────────
    /// Build a timestamped subdirectory under base_dir, write cloud.pcd and
    /// metadata.yaml there.  Returns the directory path on success or "" on failure.
    std::string saveMapPcd(const std::string& base_dir);
};

#endif // FAST_LIO_SAM_SC_QN_MAIN_H
