#include "fast_lio_sam_sc_qn.h"

FastLioSamScQn::FastLioSamScQn(const ros::NodeHandle &n_private):
    nh_(n_private)
{
    ////// ROS params
    double loop_update_hz, vis_hz;
    LoopClosureConfig lc_config;
    auto &gc = lc_config.gicp_config_;
    auto &qc = lc_config.quatro_config_;
    /* basic */
    nh_.param<std::string>("/basic/map_frame", map_frame_, "map");
    nh_.param<std::string>("/basic/robot_frame", robot_frame_, "base_footprint");
    nh_.param<bool>("/basic/publish_tf", publish_tf_, true);
    nh_.param<bool>("/basic/init_from_tf", init_from_tf_, true);
    nh_.param<double>("/basic/max_odom_jump_m", max_odom_jump_m_, 3.0);
    nh_.param<double>("/basic/lio_cov_threshold", lio_cov_threshold_, 1.0);
    nh_.param<bool>("/basic/reinit_on_jump", reinit_on_jump_, true);
    nh_.param<double>("/basic/loop_update_hz", loop_update_hz, 1.0);
    nh_.param<double>("/basic/vis_hz", vis_hz, 0.5);
    nh_.param<double>("/save_voxel_resolution", voxel_res_, 0.3);
    nh_.param<double>("/quatro_nano_gicp_voxel_resolution", lc_config.voxel_res_, 0.3);
    /* keyframe */
    nh_.param<double>("/keyframe/keyframe_threshold", keyframe_thr_, 1.0);
    nh_.param<int>("/keyframe/nusubmap_keyframes", lc_config.num_submap_keyframes_, 5);
    nh_.param<bool>("/keyframe/enable_submap_matching", lc_config.enable_submap_matching_, false);
    /* ScanContext */
    nh_.param<double>("/scancontext_max_correspondence_distance",
                      lc_config.scancontext_max_correspondence_distance_,
                      35.0);
    /* nano (GICP config) */
    nh_.param<int>("/nano_gicp/thread_number", gc.nano_thread_number_, 0);
    nh_.param<double>("/nano_gicp/icp_score_threshold", gc.icp_score_thr_, 10.0);
    nh_.param<int>("/nano_gicp/correspondences_number", gc.nano_correspondences_number_, 15);
    nh_.param<double>("/nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 0.01);
    nh_.param<int>("/nano_gicp/max_iter", gc.nano_max_iter_, 32);
    nh_.param<double>("/nano_gicp/transformation_epsilon", gc.transformation_epsilon_, 0.01);
    nh_.param<double>("/nano_gicp/euclidean_fitness_epsilon", gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("/nano_gicp/ransac/max_iter", gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
    /* quatro (Quatro config) */
    nh_.param<bool>("/quatro/enable", lc_config.enable_quatro_, false);
    nh_.param<bool>("/quatro/optimize_matching", qc.use_optimized_matching_, true);
    nh_.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_, 30.0);
    nh_.param<int>("/quatro/max_nucorrespondences", qc.quatro_max_num_corres_, 200);
    nh_.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_, 0.3);
    nh_.param<double>("/quatro/fpfh_radius", qc.fpfh_radius_, 0.5);
    nh_.param<bool>("/quatro/estimating_scale", qc.estimat_scale_, false);
    nh_.param<double>("/quatro/noise_bound", qc.noise_bound_, 0.3);
    nh_.param<double>("/quatro/rotation/gnc_factor", qc.rot_gnc_factor_, 1.4);
    nh_.param<double>("/quatro/rotation/rot_cost_diff_threshold", qc.rot_cost_diff_thr_, 0.0001);
    nh_.param<int>("/quatro/rotation/numax_iter", qc.quatro_max_iter_, 50);
    /* results */
    nh_.param<bool>("/result/save_map_bag", save_map_bag_, false);
    nh_.param<bool>("/result/save_map_pcd", save_map_pcd_, false);
    nh_.param<bool>("/result/save_in_kitti_format", save_in_kitti_format_, false);
    nh_.param<std::string>("/result/seq_name", seq_name_, "");
    /* GPS */
    std::string gps_topic;
    nh_.param<std::string>("/gps/topic", gps_topic, "/gps/odometry");
    nh_.param<double>("/gps/cov_threshold", gps_cov_threshold_, 25.0);
    nh_.param<bool>("/gps/use_slam_cov_gate", use_slam_cov_gate_, false);
    nh_.param<bool>("/gps/use_elevation", use_gps_elevation_, true);
    nh_.param<double>("/gps/cov_gate", gps_cov_gate_, 0.01);
    nh_.param<double>("/gps/noise_floor", gps_noise_floor_, 1.0);
    nh_.param<double>("/gps/cov_scale", gps_cov_scale_, 1.0);
    nh_.param<double>("/gps/min_spacing", gps_min_spacing_, 5.0);
    nh_.param<double>("/gps/min_traveled_dist", gps_min_traveled_dist_, 5.0);
    nh_.param<int>("/gps/re_entry_skip_count", gps_re_entry_skip_count_, 2);
    nh_.param<double>("/gps/heading_cov_gate", gps_heading_cov_gate_, 0.1);
    nh_.param<double>("/gps/heading_noise_floor", gps_heading_noise_floor_, 0.01);
    nh_.param<double>("/gps/heading_factor_noise", heading_factor_noise_, 0.0076);
    nh_.param<int>("/gps/lm_max_factors", max_lm_factors_, 500);
    nh_.param<int>("/gps/lm_max_iterations", lm_max_iterations_, 20);
    nh_.param<int>("/gps/lm_max_passes", lm_max_passes_, 3);
    nh_.param<double>("/gps/lm_max_distance", lm_max_distance_, 50.0);
    /* Loop closure quality gates (orchard / repeating-feature resilience) */
    nh_.param<int>("/loop_closure/min_keyframe_separation", min_loop_keyframe_separation_, 50);
    nh_.param<double>("/loop_closure/noise_floor_rot", loop_noise_floor_rot_, 0.01);
    nh_.param<double>("/loop_closure/noise_rot_scale", loop_noise_rot_scale_, 1.0);
    nh_.param<double>("/loop_closure/noise_floor_pos", loop_noise_floor_pos_, 1.0);
    nh_.param<double>("/loop_closure/max_yaw_diff_deg", loop_max_yaw_diff_deg_, 30.0);
    nh_.param<int>("/loop_closure/lm_passes", loop_lm_passes_, 1);
    nh_.param<double>("/basic/init_prior_noise_z", init_prior_noise_z_, 1.0);
    /* Ground-plane Z prior */
    nh_.param<bool>("/gps/use_ground_prior", use_ground_prior_, false);
    nh_.param<double>("/gps/ground_prior_sigma", ground_prior_sigma_, 1.0);
    if (use_ground_prior_)
    {
        ground_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(1, std::max(ground_prior_sigma_, 0.01));
        ROS_INFO("[ZPrior] Ground Z prior enabled: sigma=%.2f m", ground_prior_sigma_);
    }
    /* GPS heading initialisation */
    std::string heading_topic;
    nh_.param<std::string>("/gps/heading_topic", heading_topic, "");
    nh_.param<double>("/gps/heading_init_timeout", gps_heading_init_timeout_, 30.0);
    gps_heading_wait_forever_ = !heading_topic.empty(); // if topic given, wait indefinitely
    node_start_time_ = ros::Time::now();
    /* Seed initial corrected pose from TF if available */
    if (init_from_tf_)
    {
        if (tf_listener_.waitForTransform(map_frame_, robot_frame_, ros::Time(0), ros::Duration(2.0)))
        {
            tf::StampedTransform tf_stamped;
            tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), tf_stamped);
            Eigen::Affine3d tf_eigen;
            tf::transformTFToEigen(tf_stamped, tf_eigen);
            last_corrected_pose_ = tf_eigen.matrix();
            ROS_INFO("[Init] Seeded initial pose from TF %s -> %s: t=(%.2f, %.2f, %.2f)",
                     map_frame_.c_str(), robot_frame_.c_str(),
                     tf_stamped.getOrigin().x(), tf_stamped.getOrigin().y(), tf_stamped.getOrigin().z());
        }
        else
            ROS_WARN("[Init] TF %s -> %s not available within 2 s — starting at origin",
                     map_frame_.c_str(), robot_frame_.c_str());
    }
    /* Initialization of loop closure */
    loop_closure_ = std::make_shared<LoopClosure>(lc_config);
    /* Initialization of GTSAM */
    gtsam::ISAM2Params isam_params_;
    isam_params_.relinearizeThreshold = 0.01;
    isam_params_.relinearizeSkip = 1;
    isam_handler_ = std::make_shared<gtsam::ISAM2>(isam_params_);
    /* ROS things */
    odom_path_.header.frame_id = map_frame_;
    corrected_path_.header.frame_id = map_frame_;
    package_path_ = ros::package::getPath("fast_lio_sam_sc_qn");
    /* topics */
    std::string t_input_odom, t_input_pcd, t_save_dir;
    std::string t_odom, t_path, t_corrected_odom, t_corrected_path;
    std::string t_corrected_map, t_corrected_pcd, t_loop, t_pose, t_slam_odom;
    nh_.param<std::string>("/topics/input_odom",        t_input_odom,     "/Odometry");
    nh_.param<std::string>("/topics/input_pcd",         t_input_pcd,      "/cloud_registered");
    nh_.param<std::string>("/topics/save_dir",          t_save_dir,       "/save_dir");
    nh_.param<std::string>("/topics/odom",              t_odom,           "/ori_odom");
    nh_.param<std::string>("/topics/path",              t_path,           "/ori_path");
    nh_.param<std::string>("/topics/corrected_odom",    t_corrected_odom, "/corrected_odom");
    nh_.param<std::string>("/topics/corrected_path",    t_corrected_path, "/corrected_path");
    nh_.param<std::string>("/topics/corrected_map",     t_corrected_map,  "/corrected_map");
    nh_.param<std::string>("/topics/corrected_pcd",     t_corrected_pcd,  "/corrected_current_pcd");
    nh_.param<std::string>("/topics/loop_detection",    t_loop,           "/loop_detection");
    nh_.param<std::string>("/topics/pose_stamped",      t_pose,           "/pose_stamped");
    nh_.param<std::string>("/topics/slam_odom",         t_slam_odom,      "/odom/slam");
    /* publishers */
    odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(t_odom, 10, true);
    path_pub_ = nh_.advertise<nav_msgs::Path>(t_path, 10, true);
    corrected_odom_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_odom, 10, true);
    corrected_path_pub_ = nh_.advertise<nav_msgs::Path>(t_corrected_path, 10, true);
    corrected_pcd_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_map, 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_pcd, 10, true);
    loop_detection_pub_ = nh_.advertise<visualization_msgs::Marker>(t_loop, 10, true);
    realtime_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(t_pose, 10);
    slam_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(t_slam_odom, 10);
    std::string t_gps_constraints;
    nh_.param<std::string>("/topics/gps_constraints", t_gps_constraints, "/fast_lio_sam/gps_constraints");
    gps_constraint_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(t_gps_constraints, 10, true);
    ROS_INFO("[GPS] Constraint markers topic: %s  re_entry_skip_count=%d  cov_gate=%.1f  min_spacing=%.1f m",
             gps_constraint_pub_.getTopic().c_str(), gps_re_entry_skip_count_,
             gps_cov_gate_, gps_min_spacing_);
    debug_src_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/src", 10, true);
    debug_dst_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/dst", 10, true);
    debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/coarse_aligned_quatro", 10, true);
    debug_fine_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/fine_aligned_nano_gicp", 10, true);
    /* subscribers */
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, t_input_odom, 10);
    sub_pcd_ = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, t_input_pcd, 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(boost::bind(&FastLioSamScQn::odomPcdCallback, this, _1, _2));
    sub_save_flag_ = nh_.subscribe(t_save_dir, 1, &FastLioSamScQn::saveFlagCallback, this);
    sub_gps_ = nh_.subscribe(gps_topic, 200, &FastLioSamScQn::gpsCallback, this,
                              ros::TransportHints().tcpNoDelay());
    if (!heading_topic.empty())
        sub_heading_ = nh_.subscribe(heading_topic, 10, &FastLioSamScQn::headingCallback, this,
                                     ros::TransportHints().tcpNoDelay());
    else
    {
        gps_heading_received_ = true;  // no heading topic configured: skip wait
        gps_initial_yaw_ = std::numeric_limits<double>::quiet_NaN(); // mark as not set
    }
    /* Timers */
    loop_timer_ = nh_.createTimer(ros::Duration(1 / loop_update_hz), &FastLioSamScQn::loopTimerFunc, this);
    vis_timer_ = nh_.createTimer(ros::Duration(1 / vis_hz), &FastLioSamScQn::visTimerFunc, this);
    ROS_INFO("Main class, starting node...");
}

void FastLioSamScQn::odomPcdCallback(const nav_msgs::OdometryConstPtr &odom_msg,
                                     const sensor_msgs::PointCloud2ConstPtr &pcd_msg)
{
    Eigen::Matrix4d last_odom_tf;
    last_odom_tf = current_frame_.pose_eig_;                              // to calculate delta
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_); // to be checked if keyframe or not

    //// 0. LIO health checks — delta magnitude gate and covariance gate
    {
        // T1: inter-frame position jump check (catches LIO resets / degenerate frames)
        // Skip on the very first callback: last_odom_tf is Identity (default) while LIO may
        // have been running for tens of seconds (waiting for GPS heading), so the delta would
        // be the robot's entire LIO-world displacement — guaranteed false positive.
        const Eigen::Vector3d frame_delta = (last_odom_tf.inverse() * current_frame_.pose_eig_).block<3,1>(0,3);
        const double jump_dist = frame_delta.norm();
        if (first_odom_received_ && jump_dist > max_odom_jump_m_)
        {
            ROS_WARN_THROTTLE(1.0, "[LIO] Jump detected: %.2f m (threshold %.2f m) — dropping frame",
                              jump_dist, max_odom_jump_m_);
            current_frame_.pose_eig_ = last_odom_tf; // freeze so next delta is clean
            if (reinit_on_jump_ && is_initialized_)
            {
                ROS_WARN("[LIO] Resetting GTSAM backend after jump — will re-seed on next frame");
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                is_initialized_ = false;
                odom_delta_ = Eigen::Matrix4d::Identity();
                // last_corrected_pose_ is kept so the next init_from_tf lookup still has context
            }
            return;
        }

        // T2: FAST-LIO covariance gate — position covariance trace from incoming odom
        first_odom_received_ = true;
        if (lio_cov_threshold_ > 0.0)
        {
            // covariance[0,7,14] = x,y,z variances in row-major 6x6 matrix
            const double pos_cov_trace = odom_msg->pose.covariance[0]
                                       + odom_msg->pose.covariance[7]
                                       + odom_msg->pose.covariance[14];
            if (pos_cov_trace > lio_cov_threshold_)
                ROS_WARN_THROTTLE(2.0, "[LIO] Degenerate frame (cov trace=%.4f > %.4f) — will still integrate but not keyframe",
                                  pos_cov_trace, lio_cov_threshold_);
            current_frame_.is_degenerate_ = (pos_cov_trace > lio_cov_threshold_);
        }
    }
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    {
        //// 1. realtime pose = last corrected odom * delta (last -> current)
        std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
        odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
        current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
        realtime_pose_pub_.publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        {
            nav_msgs::Odometry slam_odom;
            slam_odom.header.stamp = odom_msg->header.stamp;
            slam_odom.header.frame_id = map_frame_;
            slam_odom.child_frame_id = robot_frame_;
            const auto &ps = poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_);
            slam_odom.pose.pose = ps.pose;
            slam_odom.twist = odom_msg->twist;  // pass-through LIO body-frame velocity
            slam_odom_pub_.publish(slam_odom);
        }
        if (publish_tf_)
            broadcaster_.sendTransform(tf::StampedTransform(poseEigToROSTf(current_frame_.pose_corrected_eig_),
                                                            ros::Time::now(),
                                                            map_frame_,
                                                            robot_frame_));
    }
    corrected_current_pcd_pub_.publish(pclToPclRos(transformPcd(current_frame_.pcd_, current_frame_.pose_corrected_eig_), map_frame_));

    if (!is_initialized_) //// init only once
    {
        // Wait for GPS heading unless timeout has elapsed
        if (!gps_heading_received_)
        {
            if (gps_heading_wait_forever_)
            {
                ROS_INFO_THROTTLE(5.0, "[Init] Waiting for GPS heading on %s...",
                                  sub_heading_.getTopic().c_str());
                return;
            }
            const double waited = (ros::Time::now() - node_start_time_).toSec();
            if (waited < gps_heading_init_timeout_)
            {
                ROS_INFO_THROTTLE(5.0, "[Init] Waiting for GPS heading (%.0f / %.0f s)...",
                                  waited, gps_heading_init_timeout_);
                return;
            }
            ROS_WARN("[Init] GPS heading timeout (%.0f s) — initialising with LIO yaw.", gps_heading_init_timeout_);
            gps_heading_received_ = true; // don't block further
        }

        // Wait for first GPS position fix so the initial GPS anchor is available immediately
        // after init. Without this the first keyframe may be inserted before robot_localization
        // has published any fix, causing the GPS factor to not anchor the map until much later.
        if (!gps_first_received_)
        {
            ROS_INFO_THROTTLE(5.0, "[Init] Waiting for first GPS position message on %s...",
                              sub_gps_.getTopic().c_str());
            return;
        }

        // Build initial pose. If we have a fresh TF captured at heading-received time, use that
        // as the authoritative translation (robot_localization is well-settled by then). Otherwise
        // fall back to the accumulated corrected pose from the constructor-time TF seed.
        const Eigen::Matrix4d &init_tf_source =
            (init_from_tf_ && tf_at_heading_valid_) ? tf_at_heading_pose_
                                                    : current_frame_.pose_corrected_eig_;
        gtsam::Pose3 init_pose = poseEigToGtsamPose(init_tf_source);
        if (!std::isnan(gps_initial_yaw_))
        {
            const gtsam::Rot3 lio_rot = init_pose.rotation();
            // Keep LIO roll & pitch, replace yaw with GPS heading
            // RzRyRx convention: args are (roll, pitch, yaw) — Rz applied last
            const gtsam::Rot3 gps_rot = gtsam::Rot3::RzRyRx(
                lio_rot.roll(), lio_rot.pitch(), gps_initial_yaw_);
            init_pose = gtsam::Pose3(gps_rot, init_pose.translation());
            ROS_INFO("\033[1;32m[Init] Using GPS heading yaw=%.1f° for GTSAM prior.\033[0m",
                     gps_initial_yaw_ * 180.0 / M_PI);
        }
        // Write init_pose back to the frame so that keyframes_, last_corrected_pose_,
        // and the GTSAM prior are ALL consistent (same yaw — GPS or TF, not old LIO yaw).
        current_frame_.pose_corrected_eig_ = gtsamPoseToPoseEig(init_pose);
        // Reset odom_delta so accumulated motion during heading-wait doesn't corrupt
        // the corrected pose on the very first post-init callback.
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            odom_delta_ = Eigen::Matrix4d::Identity();
            last_corrected_pose_ = current_frame_.pose_corrected_eig_;
        }
        // others
        keyframes_.push_back(current_frame_);
        updateOdomsAndPaths(current_frame_);
        // graph
        // rx, ry, rz tight (1e-4 rad²); x, y tight (1e-2 m²); z configurable — looser default
        // lets GPS correct a bad initial altitude before it gets baked in.
        auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, init_prior_noise_z_).finished();
        gtsam::noiseModel::Diagonal::shared_ptr prior_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(0, init_pose, prior_noise));
        init_esti_.insert(current_keyframe_idx_, init_pose);
        current_keyframe_idx_++;
        // ScanContext
        loop_closure_->updateScancontext(current_frame_.pcd_);
        is_initialized_ = true;
        ROS_INFO("[Init] GTSAM graph initialised. Node ready.");
    }
    else
    {
        //// 2. check if keyframe
        high_resolution_clock::time_point t2 = high_resolution_clock::now();
        if (!current_frame_.is_degenerate_ && checkIfKeyframe(current_frame_, keyframes_.back()))
        {
            // 2-2. if so, save
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                keyframes_.push_back(current_frame_);
            }
            // 2-3. if so, add to graph
            // Use tighter noise for healthy frames, looser for degenerate ones (T2)
            gtsam::noiseModel::Diagonal::shared_ptr odom_noise;
            if (current_frame_.is_degenerate_)
            {
                auto loose_var = (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-1, 1e-1, 1e-1).finished();
                odom_noise = gtsam::noiseModel::Diagonal::Variances(loose_var);
            }
            else
            {
                auto variance_vector = (gtsam::Vector(6) << 1e-4, 1e-4, 1e-4, 1e-2, 1e-2, 1e-2).finished();
                odom_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
            }
            gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[current_keyframe_idx_ - 1].pose_corrected_eig_);
            gtsam::Pose3 pose_to = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);
            // Accumulate odometric path length (step distance between consecutive keyframes).
            // Using Euclidean displacement front->back was wrong for looping paths where
            // the robot returns close to its start position.
            if (keyframes_.size() >= 2)
            {
                gps_total_path_length_ += (keyframes_.back().pose_corrected_eig_.block<3, 1>(0, 3) -
                                           keyframes_[keyframes_.size() - 2].pose_corrected_eig_.block<3, 1>(0, 3)).norm();
            }
            const double traveled_dist = gps_total_path_length_;
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);
                gtsam_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(current_keyframe_idx_ - 1,
                                                                    current_keyframe_idx_,
                                                                    pose_from.between(pose_to),
                                                                    odom_noise));
                init_esti_.insert(current_keyframe_idx_, pose_to);
                if (addGPSFactor(current_frame_.timestamp_, current_keyframe_idx_,
                                 traveled_dist, current_frame_.pose_corrected_eig_(2, 3)))
                {
                    loop_added_flag_ = true;
                    loop_added_flag_vis_ = true; // trigger corrected map rebuild in vis timer
                    // gps_added_flag_ is set inside addGPSFactor only after outage re-entry / startup
                }

                // Heading yaw PriorFactor — added independently of GPS position quality.
                // The heading IMU topic is gated upstream (only publishes confident fixes), so
                // we trust it unconditionally with a fixed noise of heading_factor_noise_ [rad²].
                // We consume at most one heading measurement per keyframe (fresh flag cleared here).
                {
                    std::lock_guard<std::mutex> hlk(heading_mutex_);
                    if (latest_heading_fresh_)
                    {
                        const double hyaw = latest_heading_yaw_;
                        // Use IMU-reported yaw covariance when available and larger than the noise floor.
                        // orientation_covariance[8] = [2][2] yaw variance [rad²]; 0 means not provided.
                        const double hcov = (latest_heading_cov_ > 1e-9)
                                                ? std::max(latest_heading_cov_, heading_factor_noise_)
                                                : heading_factor_noise_;
                        latest_heading_fresh_ = false;
                        // Position DOFs set loose (1e6) — only yaw (Rz index in Pose3) is constrained.
                        // Use the current LIO position so the factor mean is sane.
                        const gtsam::Point3 pos = pose_to.translation();
                        auto yaw_noise = gtsam::noiseModel::Diagonal::Variances(
                            (gtsam::Vector(6) << 1e6, 1e6, hcov, 1e6, 1e6, 1e6).finished());
                        gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                            current_keyframe_idx_,
                            gtsam::Pose3(gtsam::Rot3::Rz(hyaw), pos),
                            yaw_noise));
                        loop_added_flag_     = true;
                        loop_added_flag_vis_ = true;
                        ROS_INFO("\033[1;33m[Heading] Yaw PriorFactor at node %d yaw=%.1f\u00b0 (cov=%.4f rad\u00b2)\033[0m",
                                 current_keyframe_idx_, hyaw * 180.0 / M_PI, hcov);
                    }
                }

                // Ground Z prior: one soft ZHeightFactor per keyframe to anchor the vertical
                // axis between GPS fixes. Reference is updated from every accepted GPS Z so the
                // prior always reflects the most recent GPS-anchored ground level.
                if (use_ground_prior_ && ground_z_ready_)
                {
                    gtsam_graph_.add(ZHeightFactor(current_keyframe_idx_,
                                                   ground_z_ref_,
                                                   ground_prior_noise_));
                }
            }
            current_keyframe_idx_++;
            // 2-4. if so, update ScanContext
            loop_closure_->updateScancontext(current_frame_.pcd_);

            //// 3. vis
            high_resolution_clock::time_point t3 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(vis_mutex_);
                updateOdomsAndPaths(current_frame_);
            }

            //// 4. optimize with graph
            high_resolution_clock::time_point t4 = high_resolution_clock::now();
            // m_corrected_esti = gtsam::LevenbergMarquardtOptimizer(m_gtsam_graph, init_esti_).optimize(); // cf. isam.update vs values.LM.optimize
            {
                std::lock_guard<std::mutex> lock(graph_mutex_);

                // Merge pending loop factors into the main graph for this update cycle
                gtsam_graph_.push_back(pending_loop_graph_);
                const bool had_loop_this_cycle = !pending_loop_graph_.empty();
                pending_loop_graph_.resize(0);
                loop_closure_added_this_cycle_ = had_loop_this_cycle;

                bool isam_ok = false;
                try
                {
                    isam_handler_->update(gtsam_graph_, init_esti_);
                    isam_handler_->update();
                    if (loop_added_flag_) // extra updates to ensure convergence after structural graph change
                    {             // https://github.com/TixiaoShan/LIO-SAM/issues/5#issuecomment-653752936
                        isam_handler_->update();
                        isam_handler_->update();
                        isam_handler_->update();
                        isam_handler_->update();
                        isam_handler_->update();
                    }

                    // [B] Force full relinearization when a GPS factor was added.
                    // Normally ISAM2 skips nodes whose error change is below relinearizeThreshold
                    // (0.01). GPS corrections propagate poorly in a long odom chain because most
                    // upstream nodes appear converged. ISAM2UpdateParams::force_relinearize=true
                    // forces every variable to relinearize in this one extra pass, distributing
                    // the GPS correction globally before the LM step reads corrected_esti_.
                    if (gps_added_flag_)
                    {
                        // Pass empty graphs + force_relinearize=true so every variable relinearizes
                        isam_handler_->update(gtsam::NonlinearFactorGraph(), gtsam::Values(),
                                              gtsam::FactorIndices(), boost::none, boost::none,
                                              boost::none, /*force_relinearize=*/true);
                        ROS_INFO("[GPS] Forced full ISAM2 relinearization after GPS factor");
                    }

                    committed_factors_.push_back(gtsam_graph_);
                    gtsam_graph_.resize(0);
                    init_esti_.clear();
                    isam_ok = true;

                    // [C] Batch LM global refinement — triggered by GPS re-entry/startup.
                    // Adaptive pass count scales linearly with odometric gap since last accepted GPS:
                    //   step    = lm_max_distance_ / lm_max_passes_   (metres per extra pass)
                    //   passes  = clamp(floor(dist_gap / step), 1, lm_max_passes_)
                    // e.g. max_passes=10, max_dist=50m → step=5m: 5m→1, 10m→2, …, 50m+→10.
                    // Periodic LM (1-pass background smoothing) runs in its own section below,
                    // outside the keyframe block, so it fires even when the robot is stationary.

                    if (gps_added_flag_)
                    {
                        // Adaptive pass count scales linearly with dist_gap.
                        int lm_passes = 1;
                        if (lm_max_passes_ > 1 && lm_max_distance_ > 0.0)
                        {
                            const double step = lm_max_distance_ / static_cast<double>(lm_max_passes_);
                            lm_passes = std::max(1, std::min(lm_max_passes_,
                                             static_cast<int>(std::floor(lm_dist_gap_ / step))));
                        }
                        ROS_INFO("[GPS] Running LM global refinement on %zu committed factors "
                                 "(limit=%d, passes=%d, dist_gap=%.1fm)...",
                                 committed_factors_.size(), max_lm_factors_, lm_passes, lm_dist_gap_);
                        high_resolution_clock::time_point lm_t0 = high_resolution_clock::now();

                        if (max_lm_factors_ > 0 &&
                            committed_factors_.size() > static_cast<size_t>(max_lm_factors_))
                        {
                            // Graph is too large for batch LM — force-relinearize only (step B already ran).
                            // Set lm_max_factors: 0 in config to disable the cap.
                            ROS_WARN("[GPS] Skipping batch LM: %zu factors > limit %d — ISAM2 force-relinearize only",
                                     committed_factors_.size(), max_lm_factors_);
                        }
                        else
                        {
                            gtsam::LevenbergMarquardtParams lm_params;
                            lm_params.maxIterations  = static_cast<size_t>(std::max(lm_max_iterations_, 1));
                            lm_params.relativeErrorTol = 1e-4;
                            lm_params.absoluteErrorTol = 1e-4;

                            gtsam::Values lm_result;  // declared outside loop: reused as warm start on passes 1+
                            for (int pass = 0; pass < lm_passes; ++pass)
                            {
                                // Pass 0: seed from ISAM2 (already improved by step B force-relinearize).
                                // Pass 1+: reuse previous lm_result — it's a strictly better warm start
                                // than asking ISAM2 again, and avoids a full Values copy per pass.
                                gtsam::Values lm_init = (pass == 0)
                                    ? isam_handler_->calculateEstimate()
                                    : std::move(lm_result);
                                bool lm_ok = false;

                                try
                                {
                                    lm_result = gtsam::LevenbergMarquardtOptimizer(
                                                    committed_factors_, lm_init, lm_params).optimize();
                                    lm_ok = true;
                                }
                                catch (const std::exception &lm_ex)
                                {
                                    ROS_WARN("[GPS] LM pass %d/%d failed: %s — stopping passes", pass + 1, lm_passes, lm_ex.what());
                                    break;
                                }

                                if (lm_ok)
                                {
                                    high_resolution_clock::time_point lm_t1 = high_resolution_clock::now();
                                    ROS_INFO("[GPS] LM pass %d/%d done in %.1f ms — rebuilding ISAM2 from LM solution",
                                             pass + 1, lm_passes,
                                             duration_cast<microseconds>(lm_t1 - lm_t0).count() / 1e3);
                                    lm_t0 = lm_t1; // reset timer for next pass

                                    // Rebuild ISAM2 seeded from the LM-optimized values so the Bayes
                                    // tree linearization point is globally consistent.
                                    gtsam::ISAM2Params rebuild_params;
                                    rebuild_params.relinearizeThreshold = 0.01;
                                    rebuild_params.relinearizeSkip = 1;
                                    isam_handler_ = std::make_shared<gtsam::ISAM2>(rebuild_params);
                                    try
                                    {
                                        isam_handler_->update(committed_factors_, lm_result);
                                        ROS_INFO("[GPS] ISAM2 rebuilt from LM pass %d/%d successfully", pass + 1, lm_passes);
                                    }
                                    catch (const gtsam::IndeterminantLinearSystemException &e3)
                                    {
                                        ROS_ERROR("[GPS] ISAM2 rebuild from LM pass %d/%d failed: %s — stopping passes",
                                                  pass + 1, lm_passes, e3.what());
                                        break;
                                    }
                                }
                            } // for pass
                        }
                        gps_added_flag_ = false;
                    }

                    // [D] Batch LM global refinement after loop closure.
                    // A loop BetweenFactor connects far-apart nodes; ISAM2's Bayes tree (5 extra
                    // updates above) propagates the correction locally but doesn't guarantee global
                    // convergence. One LM pass over the full committed graph distributes it globally.
                    if (loop_closure_added_this_cycle_ && loop_lm_passes_ > 0)
                    {
                        // Force full relinearization so LM seeds from the globally best ISAM2 estimate
                        isam_handler_->update(gtsam::NonlinearFactorGraph(), gtsam::Values(),
                                              gtsam::FactorIndices(), boost::none, boost::none,
                                              boost::none, /*force_relinearize=*/true);

                        ROS_INFO("[Loop] Running LM global refinement on %zu committed factors "
                                 "(limit=%d, passes=%d)...",
                                 committed_factors_.size(), max_lm_factors_, loop_lm_passes_);
                        high_resolution_clock::time_point lm_t0 = high_resolution_clock::now();

                        if (max_lm_factors_ > 0 &&
                            committed_factors_.size() > static_cast<size_t>(max_lm_factors_))
                        {
                            ROS_WARN("[Loop] Skipping batch LM: %zu factors > limit %d — ISAM2 force-relinearize only",
                                     committed_factors_.size(), max_lm_factors_);
                        }
                        else
                        {
                            gtsam::LevenbergMarquardtParams lm_params;
                            lm_params.maxIterations  = static_cast<size_t>(std::max(lm_max_iterations_, 1));
                            lm_params.relativeErrorTol = 1e-4;
                            lm_params.absoluteErrorTol = 1e-4;

                            gtsam::Values lm_result;
                            for (int pass = 0; pass < loop_lm_passes_; ++pass)
                            {
                                gtsam::Values lm_init = (pass == 0)
                                    ? isam_handler_->calculateEstimate()
                                    : std::move(lm_result);
                                bool lm_ok = false;

                                try
                                {
                                    lm_result = gtsam::LevenbergMarquardtOptimizer(
                                                    committed_factors_, lm_init, lm_params).optimize();
                                    lm_ok = true;
                                }
                                catch (const std::exception &lm_ex)
                                {
                                    ROS_WARN("[Loop] LM pass %d/%d failed: %s — stopping passes",
                                             pass + 1, loop_lm_passes_, lm_ex.what());
                                    break;
                                }

                                if (lm_ok)
                                {
                                    high_resolution_clock::time_point lm_t1 = high_resolution_clock::now();
                                    ROS_INFO("[Loop] LM pass %d/%d done in %.1f ms — rebuilding ISAM2 from LM solution",
                                             pass + 1, loop_lm_passes_,
                                             duration_cast<microseconds>(lm_t1 - lm_t0).count() / 1e3);
                                    lm_t0 = lm_t1;

                                    gtsam::ISAM2Params rebuild_params;
                                    rebuild_params.relinearizeThreshold = 0.01;
                                    rebuild_params.relinearizeSkip = 1;
                                    isam_handler_ = std::make_shared<gtsam::ISAM2>(rebuild_params);
                                    try
                                    {
                                        isam_handler_->update(committed_factors_, lm_result);
                                        ROS_INFO("[Loop] ISAM2 rebuilt from LM pass %d/%d successfully",
                                                 pass + 1, loop_lm_passes_);
                                    }
                                    catch (const gtsam::IndeterminantLinearSystemException &e3)
                                    {
                                        ROS_ERROR("[Loop] ISAM2 rebuild from LM pass %d/%d failed: %s — stopping passes",
                                                  pass + 1, loop_lm_passes_, e3.what());
                                        break;
                                    }
                                }
                            } // for pass
                        }
                    }
                }
                catch (const gtsam::IndeterminantLinearSystemException &e)
                {
                    ROS_WARN("[ISAM2] IndeterminantLinearSystem: %s", e.what());
                    ROS_WARN("[ISAM2] Loop factors caused indeterminate system — discarding and rebuilding");

                    // Remove the bad loop pair from tracking so the bucket can be retried later.
                    // Only do this if a loop closure (not just GPS) was added this cycle — loop_idx_pairs_
                    // accumulates ALL past closures, so .back() would be wrong if only GPS fired.
                    if (loop_closure_added_this_cycle_ && !loop_idx_pairs_.empty())
                    {
                        const auto &bad = loop_idx_pairs_.back();
                        const int bs = static_cast<int>(bad.first)  / min_loop_keyframe_separation_;
                        const int bd = static_cast<int>(bad.second) / min_loop_keyframe_separation_;
                        existing_loop_factors_.erase({bs, bd});
                        loop_idx_pairs_.pop_back();
                    }
                    loop_added_flag_             = false;
                    loop_added_flag_vis_          = false;
                    loop_closure_added_this_cycle_ = false;
                    gps_added_flag_              = false;

                    // Rebuild ISAM2 from the last known-good committed factor history.
                    // The loop factor that caused the failure was merged into gtsam_graph_;
                    // we filter it out by key-distance and rebuild with only safe factors
                    // (odom BetweenFactors + GPS + re-init Priors).

                    gtsam::ISAM2Params rebuild_params;
                    rebuild_params.relinearizeThreshold = 0.01;
                    rebuild_params.relinearizeSkip = 1;
                    isam_handler_ = std::make_shared<gtsam::ISAM2>(rebuild_params);

                    // Extract only safe (non-loop) factors from the merged gtsam_graph_.
                    // Loop BetweenFactors have 2 keys with |k0-k1| >= min_loop_keyframe_separation_.
                    // Odom BetweenFactors have adjacent keys (diff == 1).
                    // GPS factors have 1 key. PriorFactors have 1 key.
                    // This preserves the odom edge for the current keyframe in the rebuild
                    // so the chain 0..N is intact and marginalCovariance(N) works in step 5.
                    gtsam::NonlinearFactorGraph safe_pending;
                    for (const auto &f : gtsam_graph_)
                    {
                        if (!f) continue;
                        const auto &keys = f->keys();
                        bool is_loop = (keys.size() == 2 &&
                                        std::abs(static_cast<long>(keys[0]) - static_cast<long>(keys[1]))
                                            > 1);
                        if (!is_loop) safe_pending.push_back(f);
                    }

                    // Rebuild graph = all committed factors + safe pending (odom + GPS only)
                    gtsam::NonlinearFactorGraph rebuild_graph = committed_factors_;
                    rebuild_graph.push_back(safe_pending);

                    // Initial values: corrected_esti_ for all existing nodes, init_esti_ for new node N
                    gtsam::Values rebuild_values = corrected_esti_;
                    for (const auto &kv : init_esti_)
                    {
                        if (!rebuild_values.exists(kv.key))
                            rebuild_values.insert(kv.key, kv.value);
                    }

                    try
                    {
                        isam_handler_->update(rebuild_graph, rebuild_values);
                        committed_factors_.push_back(safe_pending); // safe_pending is now committed
                        ROS_INFO("[ISAM2] Rebuild from %zu committed factors succeeded",
                                 committed_factors_.size());
                    }
                    catch (const gtsam::IndeterminantLinearSystemException &e2)
                    {
                        ROS_ERROR("[ISAM2] Rebuild also failed: %s — resetting to empty ISAM2", e2.what());
                        isam_handler_ = std::make_shared<gtsam::ISAM2>(rebuild_params);
                        committed_factors_.resize(0);
                    }

                    gtsam_graph_.resize(0);
                    init_esti_.clear();
                    loop_closure_added_this_cycle_ = false;
                    isam_ok = false; // corrected_esti_ will be refreshed in step 5 from rebuilt ISAM2
                }
                (void)isam_ok; // suppress unused-variable warning
            }

            //// 5. handle corrected results
            // get corrected poses and reset odom delta (for realtime pose pub)
            high_resolution_clock::time_point t5 = high_resolution_clock::now();
            {
                std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
                const gtsam::Values new_esti = isam_handler_->calculateEstimate();
                if (new_esti.empty())
                {
                    // Double-rebuild failure left an empty ISAM2 — keep stale corrected_esti_
                    // and last_corrected_pose_ to avoid underflow crash in .at<> / marginalCovariance.
                    ROS_ERROR_THROTTLE(2.0, "[ISAM2] calculateEstimate() returned empty — keeping stale estimate");
                }
                else
                {
                    corrected_esti_ = std::move(new_esti);
                    last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size() - 1));
                    odom_delta_ = Eigen::Matrix4d::Identity();
                    pose_covariance_ = isam_handler_->marginalCovariance(corrected_esti_.size() - 1);
                }
            }
            // correct poses in keyframes
            if (loop_added_flag_)
            {
                std::lock_guard<std::mutex> lock(keyframes_mutex_);
                for (size_t i = 0; i < corrected_esti_.size(); ++i)
                {
                    keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(i));
                }
                loop_added_flag_ = false;
            }
            high_resolution_clock::time_point t6 = high_resolution_clock::now();

            ROS_DEBUG("real: %.1f, key_add: %.1f, vis: %.1f, opt: %.1f, res: %.1f, tot: %.1fms",
                     duration_cast<microseconds>(t2 - t1).count() / 1e3,
                     duration_cast<microseconds>(t3 - t2).count() / 1e3,
                     duration_cast<microseconds>(t4 - t3).count() / 1e3,
                     duration_cast<microseconds>(t5 - t4).count() / 1e3,
                     duration_cast<microseconds>(t6 - t5).count() / 1e3,
                     duration_cast<microseconds>(t6 - t1).count() / 1e3);
        }
    }
    return;
}

void FastLioSamScQn::loopTimerFunc(const ros::TimerEvent &event)
{
    if (!is_initialized_ || keyframes_.empty())
        return;

    auto &latest_keyframe = keyframes_.back();
    if (latest_keyframe.processed_)
    {
        return;
    }
    latest_keyframe.processed_ = true;

    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    const int closest_keyframe_idx = loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
    {
        return;
    }
    // Reject candidates too close in keyframe index — in repetitive environments (orchards)
    // adjacent rows produce near-identical ScanContext descriptors. Requiring a minimum
    // index separation ensures the robot has actually traveled far enough for this to be
    // a genuine revisit rather than an inter-row confusion.
    if (std::abs(latest_keyframe.idx_ - closest_keyframe_idx) < min_loop_keyframe_separation_)
    {
        ROS_INFO_THROTTLE(5.0, "[Loop] Candidate kf %d too close to current kf %d (gap %d < %d) — skipping",
                  closest_keyframe_idx, latest_keyframe.idx_,
                  std::abs(latest_keyframe.idx_ - closest_keyframe_idx),
                  min_loop_keyframe_separation_);
        return;
    }
    // Deduplication: bucket both ends into bins of size min_loop_keyframe_separation_ so
    // consecutive src keyframes (100, 101, 102) that would all match the same dst region
    // don't each add an independent, slightly-inconsistent BetweenFactor that oscillates.
    const int src_bucket = latest_keyframe.idx_ / min_loop_keyframe_separation_;
    const int dst_bucket = closest_keyframe_idx / min_loop_keyframe_separation_;
    if (existing_loop_factors_.count({src_bucket, dst_bucket}))
    {
        ROS_DEBUG("[Loop] Duplicate suppressed: src bucket %d / dst bucket %d already constrained",
                  src_bucket, dst_bucket);
        return;
    }

    const RegistrationOutput &reg_output = loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);
    if (reg_output.is_valid_)
    {
        ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f (kf %d -> %d, gap %d)\033[0m",
                 reg_output.score_, latest_keyframe.idx_, closest_keyframe_idx,
                 std::abs(latest_keyframe.idx_ - closest_keyframe_idx));
        const auto &score = reg_output.score_;
        gtsam::Pose3 pose_from = poseEigToGtsamPose(reg_output.pose_between_eig_ * latest_keyframe.pose_corrected_eig_); // IMPORTANT: take care of the order
        gtsam::Pose3 pose_to = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);

        // Use the full ICP Pose3 (rotation + translation) for the BetweenFactor.
        // Previously we substituted LIO rotation to guard against bad ICP yaw in repetitive
        // environments, but that made loop closures position-only: yaw drift accumulated in
        // lio_relative_rot and was baked into every factor, preventing any yaw correction.
        //
        // Now we use ICP rotation directly with a noise model that:
        //   - Scales with yaw_diff_deg so large ICP-vs-LIO disagreements get looser rotation weight
        //   - Has a floor of loop_noise_floor_rot_ (default 0.01 rad²)
        //   - Remains tighter than GPS position noise so loops still strongly correct position
        //
        // The yaw gate (max_yaw_diff_deg_) is widened to 30° (was 5°) so genuine yaw corrections
        // are not blocked. Quatro pre-alignment makes large wrong-rotation false positives rare;
        // the ICP score gate (icp_score_threshold) is the primary quality filter.
        const gtsam::Pose3 lio_from = poseEigToGtsamPose(latest_keyframe.pose_corrected_eig_);
        const gtsam::Pose3 lio_to   = poseEigToGtsamPose(keyframes_[closest_keyframe_idx].pose_corrected_eig_);
        const gtsam::Rot3  lio_relative_rot = lio_from.rotation().between(lio_to.rotation());
        const gtsam::Pose3 icp_between = pose_from.between(pose_to);
        // Wrap yaw difference to [-180, 180]
        double yaw_diff_deg = (icp_between.rotation().yaw() - lio_relative_rot.yaw()) * 180.0 / M_PI;
        while (yaw_diff_deg >  180.0) yaw_diff_deg -= 360.0;
        while (yaw_diff_deg < -180.0) yaw_diff_deg += 360.0;
        yaw_diff_deg = std::abs(yaw_diff_deg);
        ROS_INFO("[Loop] ICP yaw=%.1f° LIO yaw=%.1f° (diff=%.1f°)",
                 icp_between.rotation().yaw() * 180.0 / M_PI,
                 lio_relative_rot.yaw() * 180.0 / M_PI,
                 yaw_diff_deg);
        if (yaw_diff_deg > loop_max_yaw_diff_deg_)
        {
            ROS_WARN("[Loop] Rejected: ICP yaw disagrees with LIO by %.1f° > %.1f° gate — likely false match",
                     yaw_diff_deg, loop_max_yaw_diff_deg_);
            return;
        }

        // Rotation variance: floor * (1 + scale * yaw_diff_deg) so the factor softens as
        // ICP-vs-LIO disagreement grows, without hard-rejecting potentially valid yaw corrections.
        // Translation variance: max(icp_score, noise_floor_pos)
        const double rv = loop_noise_floor_rot_ * (1.0 + loop_noise_rot_scale_ * yaw_diff_deg);
        const double tv = std::max(score, loop_noise_floor_pos_);
        auto variance_vector = (gtsam::Vector(6) << rv, rv, rv, tv, tv, tv).finished();
        gtsam::noiseModel::Diagonal::shared_ptr loop_noise = gtsam::noiseModel::Diagonal::Variances(variance_vector);
        ROS_INFO("[Loop] Factor noise: rot=%.4f rad² (%.1f° 1σ)  pos=%.3f m² (%.2f m 1σ)",
                 rv, std::sqrt(rv) * 180.0 / M_PI, tv, std::sqrt(tv));
        {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            pending_loop_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(latest_keyframe.idx_,
                                                                       closest_keyframe_idx,
                                                                       icp_between,
                                                                       loop_noise));
        }
        loop_idx_pairs_.push_back({latest_keyframe.idx_, closest_keyframe_idx}); // for vis
        existing_loop_factors_.insert({src_bucket, dst_bucket});
        loop_added_flag_vis_ = true;
        loop_added_flag_ = true;
    }
    else
    {
        ROS_WARN("Loop closure rejected. Score: %.3f", reg_output.score_);
    }
    high_resolution_clock::time_point t2 = high_resolution_clock::now();

    debug_src_pub_.publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    debug_dst_pub_.publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    debug_fine_aligned_pub_.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    debug_coarse_aligned_pub_.publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

    ROS_INFO("loop: %.1f", duration_cast<microseconds>(t2 - t1).count() / 1e3);
    return;
}

void FastLioSamScQn::visTimerFunc(const ros::TimerEvent &event)
{
    if (!is_initialized_)
    {
        return;
    }

    high_resolution_clock::time_point tv1 = high_resolution_clock::now();
    //// 1. if loop closed, correct vis data
    if (loop_added_flag_vis_)
    // copy and ready
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::Path corrected_path;
        {
            std::lock_guard<std::mutex> lock(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        // correct pose and path
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            gtsam::Pose3 pose_ = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(pose_.translation().x(), pose_.translation().y(), pose_.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(pose_, map_frame_));
        }
        // update vis of loop constraints
        if (!loop_idx_pairs_.empty())
        {
            loop_detection_pub_.publish(getLoopMarkers(corrected_esti_copied));
        }
        // update with corrected data
        {
            std::lock_guard<std::mutex> lock(vis_mutex_);
            corrected_odoms_ = corrected_odoms;
            corrected_path_.poses = corrected_path.poses;
        }
        loop_added_flag_vis_ = false;
        global_map_vis_switch_ = true; // republish corrected map with updated keyframe poses
    }
    // GPS constraint markers publish unconditionally whenever there are accepted GPS factors
    if (!gps_constraint_points_.empty())
    {
        gps_constraint_pub_.publish(getGpsMarkers());
    }
    //// 2. publish odoms, paths
    {
        std::lock_guard<std::mutex> lock(vis_mutex_);
        odom_pub_.publish(pclToPclRos(odoms_, map_frame_));
        path_pub_.publish(odom_path_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_path_);
    }

    //// 3. global map
    if (global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() > 0) // save time, only once
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_.publish(pclToPclRos(*voxelized_map, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() == 0)
    {
        global_map_vis_switch_ = true;
    }
    high_resolution_clock::time_point tv2 = high_resolution_clock::now();
    ROS_DEBUG("vis: %.1fms", duration_cast<microseconds>(tv2 - tv1).count() / 1e3);
    return;
}

void FastLioSamScQn::saveFlagCallback(const std_msgs::String::ConstPtr &msg)
{
    std::string save_dir = msg->data != "" ? msg->data : package_path_;

    // save scans as individual pcd files and poses in KITTI format
    // Delete the scans folder if it exists and create a new one
    std::string seq_directory = save_dir + "/" + seq_name_;
    std::string scans_directory = seq_directory + "/scans";
    if (save_in_kitti_format_)
    {
        ROS_INFO("\033[32;1mScans are saved in %s, following the KITTI and TUM format\033[0m", scans_directory.c_str());
        if (fs::exists(seq_directory))
        {
            fs::remove_all(seq_directory);
        }
        fs::create_directories(scans_directory);

        std::ofstream kitti_pose_file(seq_directory + "/poses_kitti.txt");
        std::ofstream tum_pose_file(seq_directory + "/poses_tum.txt");
        tum_pose_file << "#timestamp x y z qx qy qz qw\n";
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                // Save the point cloud
                std::stringstream ss_;
                ss_ << scans_directory << "/" << std::setw(6) << std::setfill('0') << i << ".pcd";
                ROS_INFO("Saving %s...", ss_.str().c_str());
                pcl::io::savePCDFileASCII<PointType>(ss_.str(), keyframes_[i].pcd_);

                // Save the pose in KITTI format
                const auto &pose_ = keyframes_[i].pose_corrected_eig_;
                kitti_pose_file << pose_(0, 0) << " " << pose_(0, 1) << " " << pose_(0, 2) << " "
                                << pose_(0, 3) << " " << pose_(1, 0) << " " << pose_(1, 1) << " "
                                << pose_(1, 2) << " " << pose_(1, 3) << " " << pose_(2, 0) << " "
                                << pose_(2, 1) << " " << pose_(2, 2) << " " << pose_(2, 3) << "\n";

                const auto &lidar_optim_pose_ = poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_);
                tum_pose_file << std::fixed << std::setprecision(8) << keyframes_[i].timestamp_
                              << " " << lidar_optim_pose_.pose.position.x << " "
                              << lidar_optim_pose_.pose.position.y << " "
                              << lidar_optim_pose_.pose.position.z << " "
                              << lidar_optim_pose_.pose.orientation.x << " "
                              << lidar_optim_pose_.pose.orientation.y << " "
                              << lidar_optim_pose_.pose.orientation.z << " "
                              << lidar_optim_pose_.pose.orientation.w << "\n";
            }
        }
        kitti_pose_file.close();
        tum_pose_file.close();
        ROS_INFO("\033[32;1mScans and poses saved in .pcd and KITTI format\033[0m");
    }

    if (save_map_bag_)
    {
        rosbag::Bag bag;
        bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    }

    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(seq_directory + "/" + seq_name_ + "_map.pcd", *voxelized_map);
        ROS_INFO("\033[32;1mAccumulated map cloud saved in .pcd format\033[0m");
    }
}

FastLioSamScQn::~FastLioSamScQn()
{
    // save map
    if (save_map_bag_)
    {
        rosbag::Bag bag;
        bag.open(package_path_ + "/result.bag", rosbag::bagmode::Write);
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                ros::Time time;
                time.fromSec(keyframes_[i].timestamp_);
                bag.write("/keyframe_pcd", time, pclToPclRos(keyframes_[i].pcd_, map_frame_));
                bag.write("/keyframe_pose", time, poseEigToPoseStamped(keyframes_[i].pose_corrected_eig_));
            }
        }
        bag.close();
        ROS_INFO("\033[36;1mResult saved in .bag format!!!\033[0m");
    }
    if (save_map_pcd_)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size()); // it's an approximated size
        {
            std::lock_guard<std::mutex> lock(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
            {
                *corrected_map += transformPcd(keyframes_[i].pcd_, keyframes_[i].pose_corrected_eig_);
            }
        }
        const auto &voxelized_map = voxelizePcd(corrected_map, voxel_res_);
        pcl::io::savePCDFileASCII<PointType>(package_path_ + "/result.pcd", *voxelized_map);
        ROS_INFO("\033[32;1mResult saved in .pcd format!!!\033[0m");
    }
}

void FastLioSamScQn::headingCallback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    const auto &q = imu_msg->orientation;
    // Accept only if the quaternion is valid (non-zero)
    if (std::abs(q.w) < 1e-6 && std::abs(q.x) < 1e-6 &&
        std::abs(q.y) < 1e-6 && std::abs(q.z) < 1e-6)
        return;
    const double yaw = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z).yaw();

    // One-time initialisation block — only runs on the first valid message received AFTER
    // the first GPS position message has arrived. This ensures the heading and GPS position
    // are contemporaneous: if heading arrives before any GPS fix the map would be rotated
    // correctly but anchored at the wrong position origin.
    if (!gps_heading_received_)
    {
        if (!gps_first_received_)
        {
            ROS_INFO_THROTTLE(5.0, "[GPS Heading] Waiting for first GPS position before accepting heading init...");
            // Fall through to cache yaw for continuous factors, but don't unblock init yet.
        }
        else
        {
            gps_initial_yaw_      = yaw;
            gps_heading_received_ = true;
            ROS_INFO("\033[1;32m[GPS Heading] Received initial yaw=%.1f° — SLAM init unblocked.\033[0m",
                     yaw * 180.0 / M_PI);
            // Re-read TF right now so the SLAM starts from the robot's *current* map position
            // rather than the (potentially stale or unavailable) constructor-time TF seed.
            if (init_from_tf_)
            {
                try
                {
                    tf::StampedTransform tf_stamped;
                    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), tf_stamped);
                    Eigen::Affine3d tf_eigen;
                    tf::transformTFToEigen(tf_stamped, tf_eigen);
                    tf_at_heading_pose_ = tf_eigen.matrix();
                    tf_at_heading_valid_ = true;
                    ROS_INFO("[Init] TF at heading time %s -> %s: t=(%.2f, %.2f, %.2f)",
                             map_frame_.c_str(), robot_frame_.c_str(),
                             tf_stamped.getOrigin().x(), tf_stamped.getOrigin().y(), tf_stamped.getOrigin().z());
                }
                catch (const tf::TransformException &ex)
                {
                    ROS_WARN("[Init] TF lookup failed at heading time (%s) — using accumulated LIO position", ex.what());
                }
            }
        } // else: gps_first_received_
    } // if (!gps_heading_received_)

    // Always cache the latest yaw so odomPcdCallback can add continuous heading PriorFactors.
    // orientation_covariance is a 3x3 row-major matrix; [8] is the [2][2] yaw variance [rad²].
    // heading_mutex_ is only ever acquired after graph_mutex_ (in odomPcdCallback), never before,
    // so there is no deadlock risk with this callback which only acquires heading_mutex_.
    {
        std::lock_guard<std::mutex> lock(heading_mutex_);
        latest_heading_yaw_   = yaw;
        latest_heading_cov_   = imu_msg->orientation_covariance[8]; // [rad²], 0 if not provided
        latest_heading_fresh_ = true;
    }
}

void FastLioSamScQn::gpsCallback(const nav_msgs::OdometryConstPtr &gps_msg)
{
    std::lock_guard<std::mutex> lock(gps_mutex_);
    gps_first_received_ = true;
    gps_queue_.push_back(*gps_msg);
}

bool FastLioSamScQn::addGPSFactor(const double current_time, const int node_idx,
                                   const double traveled_dist, const double current_z)
{
    std::lock_guard<std::mutex> lock(gps_mutex_);
    if (gps_queue_.empty()) return false;

    // Track whether the first GPS factor has been added yet.
    // The first factor anchors the map origin and is allowed through immediately;
    // subsequent factors are subject to the normal gates.
    static bool first_gps_added = false;

    // Wait for the system to settle: require minimum traveled distance (same as LIO-SAM).
    // Bypass for the very first GPS factor so the map is anchored at startup.
    if (!first_gps_added)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] First GPS factor — bypassing traveled-dist gate to anchor map origin");
    }
    else if (traveled_dist < gps_min_traveled_dist_)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: traveled dist %.2f m < %.1f m threshold", traveled_dist, gps_min_traveled_dist_);
        return false;
    }

    // Optional: skip GPS when LiDAR SLAM covariance is already small
    if (use_slam_cov_gate_ &&
        pose_covariance_(3, 3) < gps_cov_threshold_ && pose_covariance_(4, 4) < gps_cov_threshold_)
    {
        ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: SLAM cov (%.4f, %.4f) below threshold %.4f",
                          pose_covariance_(3, 3), pose_covariance_(4, 4), gps_cov_threshold_);
        return false;
    }

    // Time-sync: find GPS message within ±0.2 s of current keyframe timestamp
    static pcl::PointXYZ last_gps_point(0.0f, 0.0f, 0.0f);
    while (!gps_queue_.empty())
    {
        const double msg_time = gps_queue_.front().header.stamp.toSec();
        if (msg_time < current_time - 0.2)
        {
            gps_queue_.pop_front(); // too old
            continue;
        }
        if (msg_time > current_time + 0.2)
        {
            ROS_INFO_THROTTLE(5.0, "[GPS] No sync: nearest msg %.3f s ahead of keyframe", msg_time - current_time);
            break; // too new, nothing to use yet
        }

        const nav_msgs::Odometry gps_msg = gps_queue_.front();
        gps_queue_.pop_front();

        const float noise_x = static_cast<float>(gps_msg.pose.covariance[0]);
        const float noise_y = static_cast<float>(gps_msg.pose.covariance[7]);
        float noise_z       = static_cast<float>(gps_msg.pose.covariance[14]);

        // Skip if any position covariance diagonal element exceeds the gate
        if (std::abs(noise_x) > static_cast<float>(gps_cov_gate_) ||
            std::abs(noise_y) > static_cast<float>(gps_cov_gate_) ||
            std::abs(noise_z) > static_cast<float>(gps_cov_gate_))
        {
            ROS_INFO_THROTTLE(5.0, "[GPS] Rejected: position cov (%.4f, %.4f, %.4f) > gate %.4f",
                              noise_x, noise_y, noise_z, gps_cov_gate_);
            continue;
        }

        float gps_x = static_cast<float>(gps_msg.pose.pose.position.x);
        float gps_y = static_cast<float>(gps_msg.pose.pose.position.y);
        float gps_z = static_cast<float>(gps_msg.pose.pose.position.z);

        // Optionally ignore GPS elevation and use LiDAR z instead
        if (!use_gps_elevation_)
        {
            gps_z   = static_cast<float>(current_z);
            noise_z = 0.01f;
        }

        // Skip uninitialized GPS (at world origin)
        if (std::abs(gps_x) < 1e-6f && std::abs(gps_y) < 1e-6f)
        {
            ROS_WARN_THROTTLE(5.0, "[GPS] Rejected: position at origin (uninitialized)");
            continue;
        }

        // Skip if too close to the last GPS-constrained point (avoid redundant factors).
        // For the very first GPS factor, last_gps_point is (0,0,0) which is not a real
        // position — bypass the spacing and re-entry checks so the first anchor is always added.
        const float dist_from_last = std::hypot(gps_x - last_gps_point.x, gps_y - last_gps_point.y);
        static int re_entry_skip_remaining = 0;
        if (first_gps_added)
        {
            if (dist_from_last < static_cast<float>(gps_min_spacing_))
            {
                ROS_INFO_THROTTLE(5.0, "[GPS] Skipping: %.2f m from last GPS point (< %.1f m)",
                                  dist_from_last, gps_min_spacing_);
                continue;
            }

            // Re-entry heuristic: if the gap since the last accepted GPS is >2x the spacing
            // threshold, we may have just exited a GPS-denied area (canyon, tunnel, etc.) where
            // the first fixes are noisy. Discard the first N valid fixes before accepting one.
            // Anchor last_gps_point to the re-entry location so subsequent calls measure distance
            // from there, not from the origin — otherwise the counter resets every other call.
            if (dist_from_last > 2.0f * static_cast<float>(gps_min_spacing_) && re_entry_skip_remaining == 0)
            {
                re_entry_skip_remaining = gps_re_entry_skip_count_;
                last_gps_point = pcl::PointXYZ(gps_x, gps_y, gps_z); // anchor to re-entry position
                gps_reentry_pending_ = true; // B+C global refinement will fire on first accepted fix
                ROS_WARN("[GPS] Re-entry detected (%.1f m gap) — discarding next %d fix(es)",
                         dist_from_last, re_entry_skip_remaining);
            }
            if (re_entry_skip_remaining > 0)
            {
                re_entry_skip_remaining--;
                ROS_INFO("[GPS] Re-entry: discarding fix (%d remaining)", re_entry_skip_remaining);
                continue;
            }
        }

        last_gps_point = pcl::PointXYZ(gps_x, gps_y, gps_z);
        first_gps_added = true;
        // Keep the ground Z reference current: updated on every accepted GPS fix so the
        // ZHeightFactor anchors the next GPS-dark segment at the most recent GPS Z level.
        if (use_ground_prior_)
        {
            ground_z_ref_   = static_cast<double>(gps_z);
            ground_z_ready_ = true;
        }

        // Trigger B+C global refinement on startup and after outage re-entry.
        // gps_reentry_pending_ starts true (catches the very first GPS anchor) and is
        // re-armed when a re-entry gap is detected above. Clear it here so only the first
        // accepted fix after each outage fires the expensive LM pass, not every subsequent fix.
        if (gps_reentry_pending_)
        {
            lm_dist_gap_         = traveled_dist - last_gps_accepted_path_length_;
            gps_added_flag_      = true;
            gps_reentry_pending_ = false;
            ROS_INFO("[GPS] Post-outage / startup fix accepted — LM global refinement will run (dist_gap=%.1fm)", lm_dist_gap_);
        }
        last_gps_accepted_path_length_ = traveled_dist;  // update every accepted factor

        // Scale raw covariance first, then clamp to noise floor.
        // Order: noise_scaled = noise_raw * cov_scale, then max(noise_scaled, noise_floor).
        // cov_scale > 1 softens GPS factors (useful when reported covariance is over-optimistic).
        const float noise_floor = static_cast<float>(gps_noise_floor_);
        const float cov_scale   = static_cast<float>(std::max(gps_cov_scale_, 1e-3));  // guard against zero/negative
        const gtsam::Vector3 gps_noise_vec(std::max(noise_x * cov_scale, noise_floor),
                                           std::max(noise_y * cov_scale, noise_floor),
                                           std::max(noise_z * cov_scale, noise_floor));
        gtsam::noiseModel::Diagonal::shared_ptr gps_noise =
            gtsam::noiseModel::Diagonal::Variances(gps_noise_vec);
        gtsam_graph_.add(gtsam::GPSFactor(node_idx,
                                          gtsam::Point3(gps_x, gps_y, gps_z),
                                          gps_noise));
        gps_constraint_points_.push_back(pcl::PointXYZ(gps_x, gps_y, gps_z));
        gps_constraint_noises_.push_back(Eigen::Vector2f(static_cast<float>(gps_noise_vec[0]),
                                                          static_cast<float>(gps_noise_vec[1])));
        // Publish immediately so RViz subscribers see the marker without waiting for the vis timer.
        gps_constraint_pub_.publish(getGpsMarkers());
        ROS_INFO("\033[1;36m[GPS] Position factor at node %d (%.2f, %.2f, %.2f) noise_raw(%.3f,%.3f) noise_actual(%.3f,%.3f,%.3f)\033[0m",
                 node_idx, gps_x, gps_y, gps_z, noise_x, noise_y,
                 static_cast<float>(gps_noise_vec[0]), static_cast<float>(gps_noise_vec[1]), static_cast<float>(gps_noise_vec[2]));

        // --- Heading / yaw factor ---
        // robot_localization fuses GPS heading into the orientation of the odometry message.
        // Covariance[35] = pose.covariance[5*6+5] = yaw variance [rad^2].
        const double heading_cov = gps_msg.pose.covariance[35];
        if (heading_cov <= 1e-9 || heading_cov >= gps_heading_cov_gate_)
            ROS_INFO_THROTTLE(5.0, "[GPS] No heading factor at node %d: cov=%.4f (gate=%.4f)",
                              node_idx, heading_cov, gps_heading_cov_gate_);
        if (heading_cov > 1e-9 && heading_cov < gps_heading_cov_gate_)
        {
            const auto &q = gps_msg.pose.pose.orientation;
            const double gps_yaw = gtsam::Rot3::Quaternion(q.w, q.x, q.y, q.z).yaw();
            const double head_noise = std::max(heading_cov, gps_heading_noise_floor_);
            // Tight on yaw only; very loose on roll, pitch, x, y, z
            auto yaw_noise = gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(6) << 1e6, 1e6, head_noise, 1e6, 1e6, 1e6).finished());
            gtsam_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
                node_idx,
                gtsam::Pose3(gtsam::Rot3::Rz(gps_yaw), gtsam::Point3(gps_x, gps_y, gps_z)),
                yaw_noise));
            ROS_INFO("\033[1;33m[GPS] Heading factor at node %d yaw=%.1f° (cov=%.4f rad²)\033[0m",
                     node_idx, gps_yaw * 180.0 / M_PI, heading_cov);
        }

        return true;
    }
    return false;
}

void FastLioSamScQn::updateOdomsAndPaths(const PosePcd &pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0, 3),
                               pose_pcd_in.pose_eig_(1, 3),
                               pose_pcd_in.pose_eig_(2, 3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0, 3),
                                         pose_pcd_in.pose_corrected_eig_(1, 3),
                                         pose_pcd_in.pose_corrected_eig_(2, 3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
    return;
}

visualization_msgs::Marker FastLioSamScQn::getLoopMarkers(const gtsam::Values &corrected_esti_in)
{
    visualization_msgs::Marker edges;
    edges.type = 5u;
    edges.scale.x = 0.12f;
    edges.header.frame_id = map_frame_;
    edges.pose.orientation.w = 1.0f;
    edges.color.r = 1.0f;
    edges.color.g = 1.0f;
    edges.color.b = 1.0f;
    edges.color.a = 1.0f;
    for (size_t i = 0; i < loop_idx_pairs_.size(); ++i)
    {
        if (loop_idx_pairs_[i].first >= corrected_esti_in.size() ||
            loop_idx_pairs_[i].second >= corrected_esti_in.size())
        {
            continue;
        }
        gtsam::Pose3 pose = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].first);
        gtsam::Pose3 pose2 = corrected_esti_in.at<gtsam::Pose3>(loop_idx_pairs_[i].second);
        geometry_msgs::Point p, p2;
        p.x = pose.translation().x();
        p.y = pose.translation().y();
        p.z = pose.translation().z();
        p2.x = pose2.translation().x();
        p2.y = pose2.translation().y();
        p2.z = pose2.translation().z();
        edges.points.push_back(p);
        edges.points.push_back(p2);
    }
    return edges;
}

visualization_msgs::MarkerArray FastLioSamScQn::getGpsMarkers()
{
    visualization_msgs::MarkerArray ma;

    // Nodes: one sphere per accepted GPS constraint, radius = 1σ of actual noise (sqrt of variance).
    // Larger sphere = GPS trusted less. Uses individual Marker objects so each can have its own scale.
    for (size_t i = 0; i < gps_constraint_points_.size(); ++i)
    {
        const auto &pt = gps_constraint_points_[i];
        // sigma_xy is the average of x and y 1σ values; used as the sphere radius.
        float sigma = 0.5f;
        if (i < gps_constraint_noises_.size())
        {
            const float sx = std::sqrt(std::max(gps_constraint_noises_[i].x(), 0.0f));
            const float sy = std::sqrt(std::max(gps_constraint_noises_[i].y(), 0.0f));
            sigma = std::max(0.1f, 0.5f * (sx + sy));  // average 1σ, min 0.1 m
        }
        visualization_msgs::Marker node;
        node.header.frame_id = map_frame_;
        node.ns = "gps_nodes";
        node.id = static_cast<int>(i);
        node.type = visualization_msgs::Marker::SPHERE;
        node.action = visualization_msgs::Marker::ADD;
        node.pose.orientation.w = 1.0;
        node.pose.position.x = pt.x;
        node.pose.position.y = pt.y;
        node.pose.position.z = pt.z;
        node.scale.x = node.scale.y = node.scale.z = 2.0f * sigma;  // diameter = 2σ
        node.color.r = 0.0f; node.color.g = 1.0f; node.color.b = 0.4f; node.color.a = 0.5f;
        ma.markers.push_back(node);
    }

    // Edges: line strip connecting consecutive GPS constraint points
    if (gps_constraint_points_.size() >= 2)
    {
        visualization_msgs::Marker edges;
        edges.header.frame_id = map_frame_;
        edges.ns = "gps_edges";
        edges.id = 0;  // single marker within its own namespace
        edges.type = visualization_msgs::Marker::LINE_STRIP;
        edges.action = visualization_msgs::Marker::ADD;
        edges.pose.orientation.w = 1.0;
        edges.scale.x = 0.15;
        edges.color.r = 0.0f; edges.color.g = 0.8f; edges.color.b = 0.2f; edges.color.a = 0.8f;
        for (const auto &pt : gps_constraint_points_)
        {
            geometry_msgs::Point p;
            p.x = pt.x; p.y = pt.y; p.z = pt.z;
            edges.points.push_back(p);
        }
        ma.markers.push_back(edges);
    }

    return ma;
}

bool FastLioSamScQn::checkIfKeyframe(const PosePcd &pose_pcd_in, const PosePcd &latest_pose_pcd)
{
    return keyframe_thr_ < (latest_pose_pcd.pose_corrected_eig_.block<3, 1>(0, 3) - pose_pcd_in.pose_corrected_eig_.block<3, 1>(0, 3)).norm();
}
