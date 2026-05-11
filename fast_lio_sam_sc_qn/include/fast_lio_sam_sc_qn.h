#ifndef FAST_LIO_SAM_SC_QN_MAIN_H
#define FAST_LIO_SAM_SC_QN_MAIN_H

///// common headers
#include <ctime>
#include <cmath>
#include <chrono> //time check
#include <set>
#include <vector>
#include <memory>
#include <deque>
#include <mutex>
#include <string>
#include <utility> // pair, make_pair
#include <tuple>
#include <filesystem>
#include <fstream>
#include <iostream>
///// ROS
#include <ros/ros.h>
#include <ros/package.h>              // get package_path
#include <rosbag/bag.h>               // save map
#include <tf/LinearMath/Quaternion.h> // to Quaternion_to_euler
#include <tf/LinearMath/Matrix3x3.h>  // to Quaternion_to_euler
#include <tf/transform_datatypes.h>   // createQuaternionFromRPY
#include <tf_conversions/tf_eigen.h>  // tf <-> eigen
#include <tf/transform_broadcaster.h> // broadcaster
#include <tf/transform_listener.h>   // TF lookup for init pose seeding
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
///// GTSAM
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/linear/linearExceptions.h>
///// coded headers
#include "loop_closure.h"
#include "pose_pcd.hpp"
#include "utilities.hpp"
#include "z_height_factor.h"

namespace fs = std::filesystem;
using namespace std::chrono;
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> odom_pcd_sync_pol;

////////////////////////////////////////////////////////////////////////////////////////////////////
class FastLioSamScQn
{
private:
    ///// basic params
    std::string map_frame_;
    std::string package_path_;
    std::string seq_name_;
    ///// LIO health / degeneracy guard
    double max_odom_jump_m_;       // drop frame if LIO delta > this (m)
    double lio_cov_threshold_;     // treat frame as degenerate if pos cov trace > this
    bool reinit_on_jump_;          // reset GTSAM on detected jump
    ///// shared data - odom and pcd
    std::mutex realtime_pose_mutex_, keyframes_mutex_;
    std::mutex graph_mutex_, vis_mutex_;
    Eigen::Matrix4d last_corrected_pose_ = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d odom_delta_ = Eigen::Matrix4d::Identity();
    PosePcd current_frame_;
    std::vector<PosePcd> keyframes_;
    int current_keyframe_idx_ = 0;
    ///// graph and values
    bool is_initialized_ = false;
    bool first_odom_received_ = false; // true after first real LIO frame; guards jump check bootstrap
    bool loop_added_flag_ = false;     // for opt: true when any structural factor (loop or GPS) was added
    bool loop_added_flag_vis_ = false; // for vis
    bool loop_closure_added_this_cycle_ = false; // true only when a loop BetweenFactor was added this odom cycle
    bool gps_added_flag_ = false;      // true when a GPS factor was added this cycle → triggers LM global refinement
    bool gps_reentry_pending_ = true;   // true at startup and after outage re-entry; cleared once B+C refinement fires
    int    max_lm_factors_    = 500;     // skip batch LM when committed_factors_ exceeds this (prevents O(N) stall)
    int    lm_max_iterations_ = 20;     // max iterations per LM solve
    int    lm_max_passes_     = 3;      // adaptive LM: ceiling on passes on GPS re-entry (scales with dist gap)
    double lm_max_distance_   = 50.0;   // [m] dist gap that maps to lm_max_passes_ (linear interpolation)
    double lm_dist_gap_       = 0.0;    // [m] odometric gap since last accepted GPS factor (set at re-entry)
    int    loop_lm_passes_    = 1;      // LM passes after each accepted loop closure (1 = single global refinement)
    double init_prior_noise_z_ = 1.0;   // [m²] Z variance for the node-0 PriorFactor; looser than XY so GPS can correct bad initial Z
    double last_gps_accepted_path_length_ = 0.0; // [m] path length snapshot when last GPS factor was accepted
    std::shared_ptr<gtsam::ISAM2> isam_handler_ = nullptr;
    gtsam::NonlinearFactorGraph gtsam_graph_;          // odom + GPS + re-init prior factors (safe)
    gtsam::NonlinearFactorGraph pending_loop_graph_;   // loop BetweenFactors waiting to be committed
    gtsam::NonlinearFactorGraph committed_factors_;    // full committed factor history for ISAM2 rebuild
    gtsam::Values init_esti_;
    gtsam::Values corrected_esti_;
    double keyframe_thr_;
    double voxel_res_;
    int sub_key_num_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_; // for vis
    std::set<std::pair<int, int>> existing_loop_factors_; // dedup: (src_bucket, dst_bucket) already constrained
    ///// TF
    std::string robot_frame_;
    bool publish_tf_;
    bool init_from_tf_;
    tf::TransformBroadcaster broadcaster_;
    tf::TransformListener tf_listener_;
    pcl::PointCloud<pcl::PointXYZ> odoms_, corrected_odoms_;
    nav_msgs::Path odom_path_, corrected_path_;
    bool global_map_vis_switch_ = true;
    ///// results
    bool save_map_bag_ = false, save_map_pcd_ = false, save_in_kitti_format_ = false;
    ///// ros
    ros::NodeHandle nh_;
    ros::Publisher corrected_odom_pub_, corrected_path_pub_, odom_pub_, path_pub_;
    ros::Publisher corrected_current_pcd_pub_, corrected_pcd_map_pub_, loop_detection_pub_;
    ros::Publisher realtime_pose_pub_;
    ros::Publisher slam_odom_pub_;
    ros::Publisher debug_src_pub_, debug_dst_pub_, debug_coarse_aligned_pub_, debug_fine_aligned_pub_;
    ros::Subscriber sub_save_flag_;
    ros::Timer loop_timer_, vis_timer_;
    // odom, pcd sync, and save flag subscribers
    std::shared_ptr<message_filters::Synchronizer<odom_pcd_sync_pol>> sub_odom_pcd_sync_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> sub_odom_ = nullptr;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::PointCloud2>> sub_pcd_ = nullptr;
    ///// Loop closure
    std::shared_ptr<LoopClosure> loop_closure_;
    ///// GPS
    std::deque<nav_msgs::Odometry> gps_queue_;
    std::mutex gps_mutex_;
    Eigen::MatrixXd pose_covariance_ = Eigen::MatrixXd::Identity(6, 6) * 1000.0;
    double gps_cov_threshold_;
    bool use_slam_cov_gate_;
    double gps_cov_gate_;
    double gps_noise_floor_;
    double gps_cov_scale_ = 1.0;  // multiplicative inflate applied post-gate, post-floor
    double gps_min_spacing_;
    double gps_min_traveled_dist_;
    double gps_total_path_length_ = 0.0; // accumulated odometric path length for traveled_dist gate
    int gps_re_entry_skip_count_;
    bool use_gps_elevation_;
    double gps_heading_cov_gate_;
    double gps_heading_noise_floor_;
    ros::Subscriber sub_gps_;
    ///// Loop closure quality gates (orchard / repeating-feature resilience)
    int min_loop_keyframe_separation_; // minimum keyframe index gap between query and candidate
    double loop_noise_floor_rot_;      // [rad^2] minimum variance for loop closure rotation DOF
    double loop_noise_rot_scale_;      // scale factor: rot variance = loop_noise_floor_rot_ * loop_noise_rot_scale_ * (1 + yaw_diff_deg)
    double loop_noise_floor_pos_;      // [m^2]   minimum variance for loop closure position DOF
    double loop_max_yaw_diff_deg_;     // [deg]   reject loop if ICP yaw vs LIO yaw exceeds this
    // GPS heading initialisation
    ros::Subscriber sub_heading_;
    bool gps_heading_received_ = false;
    bool gps_heading_wait_forever_ = false; // true when heading_topic is configured
    ///// Ground-plane Z prior (option 2)
    bool   use_ground_prior_   = false;     // add a soft ZHeightFactor every keyframe
    bool   ground_z_ready_     = false;     // true once z_ref set from first GPS fix
    double ground_z_ref_       = 0.0;       // [m] world-Z reference; updated from every accepted GPS fix
    double ground_prior_sigma_ = 1.0;       // [m] 1σ tolerance; controls how hard the prior fights LIO Z drift
    gtsam::noiseModel::Base::shared_ptr ground_prior_noise_;  // precomputed from sigma
    bool gps_first_received_   = false;     // true after first message on the GPS position topic
    double gps_initial_yaw_ = std::numeric_limits<double>::quiet_NaN(); // [rad] NaN = not yet set
    double gps_heading_init_timeout_;   // [s] fallback timeout (only used if wait_forever=false)
    ros::Time node_start_time_;
    // TF pose captured at heading-received time — used as the authoritative init translation
    // so the SLAM starts from the robot's current map position rather than the stale constructor-time seed.
    Eigen::Matrix4d tf_at_heading_pose_ = Eigen::Matrix4d::Identity();
    bool tf_at_heading_valid_ = false;
    // Continuous heading yaw factors
    std::mutex heading_mutex_;              // protects latest_heading_yaw_ / latest_heading_fresh_
    double latest_heading_yaw_   = 0.0;    // most recent yaw from heading IMU topic [rad]
    double latest_heading_cov_   = 0.0;    // orientation_covariance[8] from the same message [rad²]
    bool   latest_heading_fresh_ = false;  // true when a new heading has arrived since last keyframe consumed it
    double heading_factor_noise_ = 0.0076; // [rad²] noise floor for heading PriorFactor (~5° 1-sigma); actual IMU cov used when larger
    // GPS visualization
    ros::Publisher gps_constraint_pub_;
    std::vector<pcl::PointXYZ>    gps_constraint_points_; // XY positions of accepted GPS factors
    std::vector<Eigen::Vector2f>   gps_constraint_noises_; // actual (post-inflation) variance [m²] per factor (x, y)

public:
    explicit FastLioSamScQn(const ros::NodeHandle &n_private);
    ~FastLioSamScQn();

private:
    // methods
    void updateOdomsAndPaths(const PosePcd &pose_pcd_in);
    bool checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd);
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values &corrected_esti_in);
    visualization_msgs::MarkerArray getGpsMarkers();
    // cb
    void odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                         const sensor_msgs::PointCloud2ConstPtr &pcd_msg);
    void saveFlagCallback(const std_msgs::String::ConstPtr &msg);
    void loopTimerFunc(const ros::TimerEvent &event);
    void visTimerFunc(const ros::TimerEvent &event);
    void gpsCallback(const nav_msgs::OdometryConstPtr &gps_msg);
    bool addGPSFactor(const double current_time, const int node_idx,
                      const double traveled_dist, const double current_z);
    void headingCallback(const sensor_msgs::ImuConstPtr &imu_msg);
};


#endif
