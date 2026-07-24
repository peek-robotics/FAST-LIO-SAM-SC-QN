#include "fast_lio_sam_sc_qn.h"

#include <iomanip>
#include <sstream>

#include <robot_localization/ToLL.h>

// ── Constructor ────────────────────────────────────────────────────────────────

FastLioSamScQn::FastLioSamScQn(const ros::NodeHandle& n_private)
    : nh_(n_private),
      gps_handler_(loadGpsParams(n_private)),
      isam_backend_(loadBackendParams(n_private))
{
    LoopClosureConfig lc_config;
    double loop_hz, vis_hz;
    std::string gps_topic, fix_topic, heading_topic;

    loadParams(lc_config, loop_hz, vis_hz, gps_topic, fix_topic, heading_topic);
    setupRos(loop_hz, vis_hz, gps_topic, fix_topic, heading_topic);
    initComponents(lc_config);

    ROS_INFO("[Init] Node starting -- map: %s  robot: %s  loop: %.1f Hz  vis: %.1f Hz",
             map_frame_.c_str(), robot_frame_.c_str(), loop_hz, vis_hz);
}

// ── loadGpsParams ─────────────────────────────────────────────────────────────

GpsParams FastLioSamScQn::loadGpsParams(const ros::NodeHandle& nh)
{
    GpsParams p;
    nh.param<double>("/gps/cov_gate",    p.default_tier.cov_gate,    0.01);
    nh.param<double>("/gps/noise_floor", p.default_tier.noise_floor, 1.0);
    nh.param<double>("/gps/cov_scale",   p.default_tier.cov_scale,   1.0);

    std::string fix_topic;
    nh.param<std::string>("/gps/fix_topic", fix_topic, "");
    if (!fix_topic.empty())
    {
        p.use_fix_tiers         = true;
        p.require_gbas_for_init = true;
        GpsFixTier gbas, sbas;
        nh.param<double>("/gps/fix_tiers/gbas/cov_gate",    gbas.cov_gate,    0.01);
        nh.param<double>("/gps/fix_tiers/gbas/noise_floor", gbas.noise_floor, 0.2);
        nh.param<double>("/gps/fix_tiers/gbas/cov_scale",   gbas.cov_scale,   10.0);
        nh.param<double>("/gps/fix_tiers/sbas/cov_gate",    sbas.cov_gate,    0.1);
        nh.param<double>("/gps/fix_tiers/sbas/noise_floor", sbas.noise_floor, 1.0);
        nh.param<double>("/gps/fix_tiers/sbas/cov_scale",   sbas.cov_scale,   100.0);
        p.fix_tiers[sensor_msgs::NavSatStatus::STATUS_GBAS_FIX] = gbas;
        p.fix_tiers[sensor_msgs::NavSatStatus::STATUS_SBAS_FIX] = sbas;
        ROS_INFO("[GPS] Fix-type quality tiers enabled (%s) -- SLAM init requires GBAS_FIX", fix_topic.c_str());
        ROS_INFO("[GPS]   GBAS(2): cov_gate=%.3f  noise_floor=%.3f  cov_scale=%.1f",
                 gbas.cov_gate, gbas.noise_floor, gbas.cov_scale);
        ROS_INFO("[GPS]   SBAS(1): cov_gate=%.3f  noise_floor=%.3f  cov_scale=%.1f",
                 sbas.cov_gate, sbas.noise_floor, sbas.cov_scale);
    }

    nh.param<bool>("/gps/use_elevation",         p.use_elevation,       true);
    nh.param<double>("/gps/min_spacing",         p.min_spacing,         5.0);
    nh.param<double>("/gps/min_traveled_dist",   p.min_traveled_dist,   5.0);
    nh.param<int>("/gps/re_entry_skip_count",    p.re_entry_skip_count, 2);
    nh.param<double>("/gps/heading_noise_floor",    p.heading_noise_floor,   0.05);
    nh.param<bool>("/gps/use_cog_heading",           p.use_cog_heading,       false);
    nh.param<double>("/gps/cog_noise_floor",         p.cog_noise_floor,       0.05);
    nh.param<double>("/gps/best_fix_window",        p.best_fix_window,       0.0);
    nh.param<bool>("/gps/lm_every_factor",       p.lm_every_factor,     false);
    nh.param<bool>("/gps/use_ground_prior",      p.use_ground_prior,    false);

    // #2 Robust kernel on the GPS position factor.
    nh.param<std::string>("/gps/robust_kernel",  p.robust_kernel, std::string("none"));
    nh.param<double>("/gps/robust_threshold",    p.robust_thresh, 1.345);
    // #4/#6 Temporal alignment of GPS to the keyframe (LIO) clock.
    nh.param<double>("/gps/time_offset",         p.time_offset,   0.0);
    nh.param<bool>("/gps/interpolate",           p.interpolate,   true);
    nh.param<double>("/gps/max_interp_dt",       p.max_interp_dt, 0.30);
    // #7 GPS lever arm [x,y,z] of the GPS-reported point in the SLAM body (node) frame.
    // Applied as a measurement pre-correction (corrected = gps - R_node*arm) before
    // adding a plain GPSFactor. [0,0,0] means node frame == GPS frame (Grover default).
    std::vector<double> lever_arm;
    if (nh.getParam("/gps/lever_arm", lever_arm) && lever_arm.size() == 3)
    {
        p.lever_arm = Eigen::Vector3d(lever_arm[0], lever_arm[1], lever_arm[2]);
        ROS_INFO("[GPS] Lever arm set: (%.3f, %.3f, %.3f) m -- applying as measurement pre-correction",
                 lever_arm[0], lever_arm[1], lever_arm[2]);
    }
    if (p.robust_kernel != "none")
        ROS_INFO("[GPS] Robust GPS kernel: %s (threshold=%.3f whitened)",
                 p.robust_kernel.c_str(), p.robust_thresh);
    if (p.time_offset != 0.0)
        ROS_INFO("[GPS] GPS->LIO time offset: %.4f s", p.time_offset);

    std::string heading_topic;
    nh.param<std::string>("/gps/heading_topic", heading_topic, "");
    p.has_heading_topic = !heading_topic.empty();

    nh.param<int>("/gps/heading_init_stable_count", p.stable_count,      5);
    nh.param<double>("/gps/heading_init_stable_tol", p.stable_tol_yaw,   0.05);
    nh.param<double>("/gps/gps_init_stable_tol_m",  p.stable_tol_pos_m, 1.0);

    return p;
}

// ── loadBackendParams ─────────────────────────────────────────────────────────

BackendParams FastLioSamScQn::loadBackendParams(const ros::NodeHandle& nh)
{
    BackendParams p;
    nh.param<int>("/gps/lm_max_factors",     p.max_lm_factors,    500);
    nh.param<int>(   "/gps/lm_max_iterations",  p.lm_max_iterations, 200);
    nh.param<double>("/gps/lm_rel_tol",          p.lm_rel_tol,        1e-5);
    nh.param<double>("/gps/lm_abs_tol",          p.lm_abs_tol,        1e-5);
    nh.param<int>("/gps/lm_max_passes",      p.lm_max_passes,     3);
    nh.param<double>("/gps/lm_max_distance", p.lm_max_distance,   50.0);
    nh.param<int>("/loop_closure/lm_passes", p.loop_lm_passes,    1);
    nh.param<int>("/loop_closure/min_keyframe_separation", p.min_loop_kf_sep,         50);
    nh.param<double>("/loop_closure/noise_floor_rot",      p.loop_noise_floor_rot,    0.01);
    nh.param<double>("/loop_closure/noise_rot_scale",      p.loop_noise_rot_scale,    1.0);
    nh.param<double>("/loop_closure/noise_floor_pos",      p.loop_noise_floor_pos,    1.0);
    nh.param<double>("/loop_closure/max_yaw_diff_deg",     p.loop_max_yaw_diff_deg,   30.0);
    nh.param<double>("/loop_closure/max_pos_diff_m",       p.loop_max_pos_diff_m,     5.0);

    // Per-axis odom noise — helper to load a 3-element XmlRpc array.
    // XmlRpc may parse YAML values as TypeInt or TypeDouble; handle both and
    // fall back to defaults on any type mismatch or missing key.
    auto load3 = [&](const std::string& key, std::array<double,3>& arr,
                     const std::array<double,3>& def)
    {
        XmlRpc::XmlRpcValue xv;
        if (!nh.getParam(key, xv) ||
            xv.getType() != XmlRpc::XmlRpcValue::TypeArray || xv.size() != 3)
        {
            arr = def;
            return;
        }
        try
        {
            for (int i = 0; i < 3; ++i)
            {
                if (xv[i].getType() == XmlRpc::XmlRpcValue::TypeInt)
                    arr[i] = static_cast<double>(static_cast<int>(xv[i]));
                else
                    arr[i] = static_cast<double>(xv[i]);
            }
        }
        catch (...)
        {
            arr = def;
        }
    };
    load3("/backend/odom_noise_rot",       p.odom_noise_rot,       {1e-4, 1e-4, 1e-4});
    load3("/backend/odom_noise_pos",       p.odom_noise_pos,       {1e-2, 1e-2, 1e-2});
    load3("/backend/odom_noise_rot_degen", p.odom_noise_rot_degen, {1e-2, 1e-2, 1e-2});
    load3("/backend/odom_noise_pos_degen", p.odom_noise_pos_degen, {1e-1, 1e-1, 1e-1});
    return p;
}

// ── loadParams ────────────────────────────────────────────────────────────────

void FastLioSamScQn::loadParams(LoopClosureConfig& lc_config, double& loop_hz, double& vis_hz,
                                 std::string& gps_topic, std::string& fix_topic,
                                 std::string& heading_topic)
{
    auto& gc = lc_config.gicp_config_;
    auto& qc = lc_config.quatro_config_;

    /* basic */
    nh_.param<std::string>("/basic/map_frame",   map_frame_,    "map");
    nh_.param<std::string>("/basic/robot_frame", robot_frame_,  "base_footprint");
    nh_.param<bool>("/basic/publish_tf",         publish_tf_,   true);
    nh_.param<bool>("/basic/init_from_tf",       init_from_tf_, true);
    nh_.param<bool>("/basic/input_pcd_lidar_frame", input_pcd_lidar_frame_, false);
    nh_.param<std::string>("/basic/lidar_frame", lidar_frame_, std::string("livox_frame"));
    nh_.param<double>("/basic/max_odom_jump_m",  max_odom_jump_m_, 3.0);
    nh_.param<double>("/basic/lio_cov_threshold",lio_cov_threshold_, 1.0);
    nh_.param<bool>("/basic/reinit_on_jump",     reinit_on_jump_, true);
    nh_.param<int>("/basic/reinit_skip_frames",  reinit_skip_frames_, 10);
    nh_.param<int>("/basic/degrade_accept_max",  degrade_accept_max_, 8);
    nh_.param<double>("/basic/loop_update_hz",   loop_hz, 1.0);
    nh_.param<double>("/basic/vis_hz",           vis_hz, 0.5);
    nh_.param<double>("/save_voxel_resolution",  voxel_res_, 0.3);
    nh_.param<double>("/quatro_nano_gicp_voxel_resolution", lc_config.voxel_res_, 0.3);
    /* keyframe */
    nh_.param<double>("/keyframe/keyframe_threshold",         keyframe_thr_, 1.0);
    nh_.param<int>("/keyframe/num_submap_keyframes",          lc_config.num_submap_keyframes_, 5);
    nh_.param<bool>("/keyframe/enable_submap_matching",       lc_config.enable_submap_matching_, false);
    /* ScanContext */
    nh_.param<double>("/scancontext_dist_threshold",
                      lc_config.scancontext_dist_thres_, 0.2);
    nh_.param<double>("/scancontext_max_radius",
                      lc_config.scancontext_max_radius_, 80.0);
    nh_.param<double>("/scancontext_max_correspondence_distance",
                      lc_config.scancontext_max_correspondence_distance_, 35.0);
    nh_.param<double>("/scancontext_query_fov_deg",
                      lc_config.scancontext_query_fov_deg_, 360.0);
    /* nano (GICP) */
    nh_.param<int>("/nano_gicp/thread_number",              gc.nano_thread_number_, 0);
    nh_.param<double>("/nano_gicp/icp_score_threshold",     gc.icp_score_thr_, 10.0);
    nh_.param<int>("/nano_gicp/correspondences_number",     gc.nano_correspondences_number_, 15);
    nh_.param<double>("/nano_gicp/max_correspondence_distance", gc.max_corr_dist_, 2.0);
    nh_.param<int>("/nano_gicp/max_iter",                   gc.nano_max_iter_, 32);
    nh_.param<double>("/nano_gicp/transformation_epsilon",  gc.transformation_epsilon_, 0.01);
    nh_.param<double>("/nano_gicp/euclidean_fitness_epsilon",gc.euclidean_fitness_epsilon_, 0.01);
    nh_.param<int>("/nano_gicp/ransac/max_iter",            gc.nano_ransac_max_iter_, 5);
    nh_.param<double>("/nano_gicp/ransac/outlier_rejection_threshold", gc.ransac_outlier_rejection_threshold_, 1.0);
    nh_.param<double>("/nano_gicp/min_overlap_ratio",  gc.min_overlap_ratio_,  0.0);
    nh_.param<double>("/nano_gicp/overlap_distance",   gc.overlap_dist_,       0.5);
    /* quatro */
    nh_.param<bool>("/quatro/enable",               lc_config.enable_quatro_,          false);
    nh_.param<bool>("/quatro/optimize_matching",    qc.use_optimized_matching_,         true);
    nh_.param<double>("/quatro/distance_threshold", qc.quatro_distance_threshold_,      30.0);
    nh_.param<int>("/quatro/max_correspondences",   qc.quatro_max_num_corres_,          200);
    nh_.param<double>("/quatro/fpfh_normal_radius", qc.fpfh_normal_radius_,             0.3);
    nh_.param<double>("/quatro/fpfh_radius",        qc.fpfh_radius_,                   0.5);
    nh_.param<bool>("/quatro/estimating_scale",     qc.estimat_scale_,                 false);
    nh_.param<double>("/quatro/noise_bound",        qc.noise_bound_,                   0.3);
    nh_.param<double>("/quatro/rotation/gnc_factor",            qc.rot_gnc_factor_,    1.4);
    nh_.param<double>("/quatro/rotation/rot_cost_diff_threshold",qc.rot_cost_diff_thr_, 0.0001);
    nh_.param<int>("/quatro/rotation/num_max_iter",             qc.quatro_max_iter_,   50);
    /* results */
    nh_.param<bool>("/result/save_map_pcd",        save_map_pcd_,         false);
    nh_.param<std::string>("/result/seq_name",     seq_name_,             "");
    /* GPS topics */
    nh_.param<std::string>("/gps/topic",           gps_topic,     "/gps/odometry");
    nh_.param<std::string>("/gps/fix_topic",       fix_topic,     "");
    nh_.param<std::string>("/gps/heading_topic",   heading_topic, "");
    /* ground prior */
    nh_.param<bool>("/gps/use_ground_prior",       use_ground_prior_,   false);
    nh_.param<double>("/gps/ground_prior_sigma",   ground_prior_sigma_, 1.0);
    if (use_ground_prior_)
    {
        ground_prior_noise_ = gtsam::noiseModel::Isotropic::Sigma(1, std::max(ground_prior_sigma_, 0.01));
        ROS_INFO("[ZPrior] Ground Z prior enabled: sigma=%.2f m", ground_prior_sigma_);
    }
    /* misc */
    nh_.param<double>("/basic/init_prior_noise_z", init_prior_noise_z_, 1.0);
    nh_.param<double>("/basic/init_prior_noise_yaw_unknown", init_prior_noise_yaw_unknown_, 1.0);
}

// ── setupRos ──────────────────────────────────────────────────────────────────

void FastLioSamScQn::setupRos(double loop_hz, double vis_hz,
                               const std::string& gps_topic, const std::string& fix_topic,
                               const std::string& heading_topic)
{
    odom_path_.header.frame_id      = map_frame_;
    corrected_path_.header.frame_id = map_frame_;

    // Output directory: param override > /data/slam (if /data exists) > /tmp/slam
    std::string save_base;
    std::error_code ec;
    nh_.param<std::string>("/result/save_dir", save_base, "");
    if (save_base.empty())
    {
        if (fs::is_directory("/data", ec))
            save_base = "/data/slam";
        else
            save_base = "/tmp/slam";
    }
    fs::create_directories(save_base, ec);
    package_path_ = save_base;

    /* topic names */
    std::string t_input_odom, t_input_pcd, t_save_dir;
    std::string t_odom, t_path, t_corrected_odom, t_corrected_path;
    std::string t_corrected_map, t_corrected_pcd, t_loop, t_pose, t_slam_odom, t_gps;
    nh_.param<std::string>("/topics/input_odom",      t_input_odom,     "/Odometry");
    nh_.param<std::string>("/topics/input_pcd",       t_input_pcd,      "/cloud_registered");
    nh_.param<std::string>("/topics/save_dir",        t_save_dir,       "/save_dir");
    nh_.param<std::string>("/topics/odom",            t_odom,           "/ori_odom");
    nh_.param<std::string>("/topics/path",            t_path,           "/ori_path");
    nh_.param<std::string>("/topics/corrected_odom",  t_corrected_odom, "/corrected_odom");
    nh_.param<std::string>("/topics/corrected_path",  t_corrected_path, "/corrected_path");
    nh_.param<std::string>("/topics/corrected_map",   t_corrected_map,  "/corrected_map");
    nh_.param<std::string>("/topics/corrected_pcd",   t_corrected_pcd,  "/corrected_current_pcd");
    nh_.param<std::string>("/topics/loop_detection",  t_loop,           "/loop_detection");
    nh_.param<std::string>("/topics/pose_stamped",    t_pose,           "/pose_stamped");
    nh_.param<std::string>("/topics/slam_odom",       t_slam_odom,      "/odom/slam");
    nh_.param<std::string>("/topics/gps_constraints", t_gps, "/fast_lio_sam/gps_constraints");

    /* publishers */
    odom_pub_                  = nh_.advertise<sensor_msgs::PointCloud2>(t_odom, 10, true);
    path_pub_                  = nh_.advertise<nav_msgs::Path>(t_path, 10, true);
    corrected_odom_pub_        = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_odom, 10, true);
    corrected_path_pub_        = nh_.advertise<nav_msgs::Path>(t_corrected_path, 10, true);
    corrected_pcd_map_pub_     = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_map, 10, true);
    corrected_current_pcd_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(t_corrected_pcd, 10, true);
    loop_detection_pub_        = nh_.advertise<visualization_msgs::Marker>(t_loop, 10, true);
    realtime_pose_pub_         = nh_.advertise<geometry_msgs::PoseStamped>(t_pose, 10);
    slam_odom_pub_             = nh_.advertise<nav_msgs::Odometry>(t_slam_odom, 10);
    gps_constraint_pub_        = nh_.advertise<visualization_msgs::MarkerArray>(t_gps, 10, true);
    nh_.param<bool>("/basic/pub_debug_clouds", pub_debug_clouds_, false);
    if (pub_debug_clouds_)
    {
        debug_src_pub_            = nh_.advertise<sensor_msgs::PointCloud2>("debug/src",            10, true);
        debug_dst_pub_            = nh_.advertise<sensor_msgs::PointCloud2>("debug/dst",            10, true);
        debug_coarse_aligned_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("debug/coarse_aligned", 10, true);
        debug_fine_aligned_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("debug/fine_aligned",   10, true);
        ROS_INFO("[SLAM] Debug cloud publishers enabled under %s/debug/", nh_.getNamespace().c_str());
    }

    /* subscribers */
    sub_odom_ = std::make_shared<message_filters::Subscriber<nav_msgs::Odometry>>(nh_, t_input_odom, 10);
    sub_pcd_  = std::make_shared<message_filters::Subscriber<sensor_msgs::PointCloud2>>(nh_, t_input_pcd, 10);
    sub_odom_pcd_sync_ = std::make_shared<message_filters::Synchronizer<odom_pcd_sync_pol>>(
        odom_pcd_sync_pol(10), *sub_odom_, *sub_pcd_);
    sub_odom_pcd_sync_->registerCallback(
        boost::bind(&FastLioSamScQn::odomPcdCallback, this, _1, _2));

    sub_save_flag_ = nh_.subscribe(t_save_dir, 1, &FastLioSamScQn::saveFlagCallback, this);

    lm_refine_srv_ = nh_.advertiseService("run_lm_refinement",
                                           &FastLioSamScQn::lmRefineSrvCallback, this);  // grover_msgs/SrvInt16: data = passes (0 = use config default)
    ROS_INFO("[SLAM] LM refinement service ready at %s/run_lm_refinement", nh_.getNamespace().c_str());

    save_map_srv_ = nh_.advertiseService("save_map",
                                          &FastLioSamScQn::saveMapSrvCallback, this);
    ROS_INFO("[SLAM] Save-map service ready at %s/save_map", nh_.getNamespace().c_str());

    // Latched signal: the save dir, published after every saveMapPcd (any trigger
    // -- service, save-flag, GBA finish, shutdown). map_export subscribes and
    // auto-georeferences it to LAZ. Latched so a late subscriber still sees the
    // most recent save.
    map_saved_pub_ = nh_.advertise<std_msgs::String>("map_saved", 1, /*latch=*/true);

    // /toLL client (robot_localization) — used at save time to convert the
    // map origin (0,0,0) to WGS-84 lat/lon/alt for metadata. Created here so
    // it persists; the service need not be up at startup.
    to_ll_client_ = nh_.serviceClient<robot_localization::ToLL>("/toLL");

    sub_gps_ = nh_.subscribe(gps_topic, 200, &GpsHandler::onGpsOdom, &gps_handler_,
                              ros::TransportHints().tcpNoDelay());
    if (!fix_topic.empty())
        sub_gps_fix_ = nh_.subscribe(fix_topic, 200, &GpsHandler::onNavSatFix, &gps_handler_,
                                     ros::TransportHints().tcpNoDelay());
    if (!heading_topic.empty())
        sub_heading_ = nh_.subscribe(heading_topic, 10, &GpsHandler::onHeading, &gps_handler_,
                                     ros::TransportHints().tcpNoDelay());

    std::string t_lio_diag;
    nh_.param<std::string>("/topics/lio_diag", t_lio_diag, "/lidar_3d/voxel_slam/lio_diag");
    sub_lio_diag_ = nh_.subscribe(t_lio_diag, 10,
                                   &FastLioSamScQn::lioDiagCallback, this,
                                   ros::TransportHints().tcpNoDelay());

    /* timers */
    loop_timer_ = nh_.createTimer(ros::Duration(1.0 / loop_hz), &FastLioSamScQn::loopTimerFunc, this);
    vis_timer_  = nh_.createTimer(ros::Duration(1.0 / vis_hz),  &FastLioSamScQn::visTimerFunc,  this);

    nh_.param<bool>("/basic/show_perf_stats", show_perf_stats_, false);
    if (show_perf_stats_)
    {
        perf_init_time_ = ros::WallTime::now();
        perf_timer_ = nh_.createWallTimer(ros::WallDuration(perf_report_interval_),
                                           &FastLioSamScQn::perfTimerFunc, this);
        ROS_INFO("[Perf] Performance stats enabled (%.0f s interval)", perf_report_interval_);
    }

    nh_.param<bool>("/basic/cloud_sparsify", cloud_sparsify_en_, false);
    if (cloud_sparsify_en_)
    {
        nh_.param<double>("/basic/cloud_sparsify_res", cloud_sparsify_res_, voxel_res_);
        nh_.param<int>("/basic/cloud_sparsify_age",    cloud_sparsify_age_, 30);
        cloud_sparsify_thread_ = std::thread(&FastLioSamScQn::cloudSparsifyThread, this);
        ROS_INFO("[Init] Cloud sparsification enabled: res=%.2f m  age=%d kf",
                 cloud_sparsify_res_, cloud_sparsify_age_);
    }
}

// ── initComponents ────────────────────────────────────────────────────────────

void FastLioSamScQn::initComponents(const LoopClosureConfig& lc_config)
{
    loop_closure_ = std::make_shared<LoopClosure>(lc_config);

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
            ROS_WARN("[Init] TF %s -> %s not available within 2 s -- starting at origin",
                     map_frame_.c_str(), robot_frame_.c_str());
    }

    // Cache the static base->lidar extrinsic used to render LiDAR-frame clouds into the map.
    ensureLidarExtrinsic(2.0, true);
    // Loop-closure builds submaps by transforming stored LiDAR-frame keyframe clouds by their
    // base-frame node poses, so it needs the same base->lidar composition renderPose() uses.
    if (input_pcd_lidar_frame_ && lidar_extrinsic_ready_)
        loop_closure_->setRenderExtrinsic(T_base_lidar_);
}

void FastLioSamScQn::ensureLidarExtrinsic(double wait_s, bool verbose)
{
    if (lidar_extrinsic_ready_ || !input_pcd_lidar_frame_)
        return;
    if (!tf_listener_.waitForTransform(robot_frame_, lidar_frame_, ros::Time(0), ros::Duration(wait_s)))
    {
        if (verbose)
            ROS_WARN("[Init] base->lidar TF %s -> %s not available within %.1f s -- clouds rendered "
                     "WITHOUT lever arm (map may show heading-dependent doubling)",
                     robot_frame_.c_str(), lidar_frame_.c_str(), wait_s);
        return;
    }
    tf::StampedTransform tf_bl;
    tf_listener_.lookupTransform(robot_frame_, lidar_frame_, ros::Time(0), tf_bl);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf_bl, e);
    T_base_lidar_ = e.matrix();
    lidar_extrinsic_ready_ = true;
    ROS_INFO("[Init] base->lidar extrinsic %s -> %s: t=(%.3f, %.3f, %.3f) -- rendering clouds with lever arm",
             robot_frame_.c_str(), lidar_frame_.c_str(),
             T_base_lidar_(0, 3), T_base_lidar_(1, 3), T_base_lidar_(2, 3));
}

Eigen::Matrix4d FastLioSamScQn::renderPose(const Eigen::Matrix4d& node_pose) const
{
    // Keyframe clouds are stored in the LiDAR frame (input_pcd_lidar_frame_), but the graph node
    // pose is base_footprint. Compose the static base->lidar extrinsic so points land at their true
    // map position. When the extrinsic is unavailable, fall back to the node pose (previous behaviour).
    if (input_pcd_lidar_frame_ && lidar_extrinsic_ready_)
        return node_pose * T_base_lidar_;
    return node_pose;
}

// ── Destructor ────────────────────────────────────────────────────────────────

FastLioSamScQn::~FastLioSamScQn()
{
    if (cloud_sparsify_en_)
    {
        cloud_sparsify_stop_.store(true);
        cloud_sparsify_thread_.join();
    }
    if (save_map_pcd_)
        saveMapPcd(package_path_);
}

// ── odomPcdCallback ───────────────────────────────────────────────────────────

void FastLioSamScQn::odomPcdCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                                      const sensor_msgs::PointCloud2ConstPtr& pcd_msg)
{
    const Eigen::Matrix4d last_odom_tf = current_frame_.pose_eig_;
    current_frame_ = PosePcd(*odom_msg, *pcd_msg, current_keyframe_idx_, input_pcd_lidar_frame_);

    // voxel_slam odometry is the LiDAR-frame pose (its odom child_frame is base_link = lidar_frame),
    // but the graph node / GPS / output frame is robot_frame (base_footprint). Convert the incoming
    // pose livox->base once here so every downstream consumer (odom delta, BetweenFactors, init,
    // realtime pose, TF, GPS comparison) is consistently base_footprint. renderPose() then composes
    // the base->lidar extrinsic back only when placing the LiDAR-frame cloud. Without this the node
    // sits in a livox/base blend and renderPose adds a full lever arm -> heading-flip map ghosting
    // (~2x the forward offset). last_odom_tf above is the previous (already-converted) base pose, so
    // the frame-to-frame delta stays consistent.
    if (input_pcd_lidar_frame_ && lidar_extrinsic_ready_)
    {
        const Eigen::Matrix4d T_lidar_base = T_base_lidar_.inverse();
        current_frame_.pose_eig_           = current_frame_.pose_eig_ * T_lidar_base;
        current_frame_.pose_corrected_eig_ = current_frame_.pose_corrected_eig_ * T_lidar_base;
    }
    else if (input_pcd_lidar_frame_)
    {
        ROS_WARN_ONCE("[LIO] base<-lidar extrinsic unavailable; node pose left in LiDAR frame "
                      "(GPS/node frame mismatch, expect map ghosting until TF is found)");
    }

    perf_frames_total_.fetch_add(1, std::memory_order_relaxed);

    if (!passLioHealthChecks(odom_msg, last_odom_tf))
        return;

    const high_resolution_clock::time_point t1 = high_resolution_clock::now();

    publishRealtimePose(odom_msg, last_odom_tf);

    if (!is_initialized_)
    {
        tryInitialize();
        return;
    }

    if (post_reinit_frames_remaining_ > 0)
    {
        --post_reinit_frames_remaining_;
        ROS_DEBUG_THROTTLE(1.0, "[LIO] Post-reinit warmup: %d frames remaining", post_reinit_frames_remaining_);
        return;
    }

    // Degenerate frames are still allowed to become keyframes (with loosened odom noise) so the
    // graph has no gap during a degeneracy stretch; only keyframe spacing gates entry here.
    if (!checkIfKeyframe(current_frame_, keyframes_.back()))
        return;

    const high_resolution_clock::time_point t2 = high_resolution_clock::now();
    processKeyframe(odom_msg);
    const high_resolution_clock::time_point t3 = high_resolution_clock::now();

    ROS_DEBUG("[KF] realtime: %.1f  process: %.1f ms",
              duration_cast<microseconds>(t2 - t1).count() / 1e3,
              duration_cast<microseconds>(t3 - t2).count() / 1e3);

    if (show_perf_stats_)
    {
        const double kf_ms = duration_cast<microseconds>(t3 - t1).count() / 1e3;
        std::lock_guard<std::mutex> lk(perf_mutex_);
        perf_kf_time_sum_ms_ += kf_ms;
        perf_kf_time_max_ms_  = std::max(perf_kf_time_max_ms_, kf_ms);
        ++perf_kf_count_window_;
    }
}

// ── passLioHealthChecks ───────────────────────────────────────────────────────

bool FastLioSamScQn::passLioHealthChecks(const nav_msgs::OdometryConstPtr& odom_msg,
                                          const Eigen::Matrix4d& last_odom_tf)
{
    // Gate 0: voxel_slam degeneracy (degrade_state: Ok=1, Low=2, Medium=4, High=8, Reset=16).
    // Accept Low/Medium/High (<= degrade_accept_max_) but flag them so the odom factor is trusted
    // less (loose _degen noise); drop only Reset -- and anything degenerate before init, since node 0
    // anchors the whole map. Gate 1 (jump) still reinits genuinely broken frames below.
    const uint8_t diag = latest_diag_state_.load();
    const bool pre_init_dirty = (!is_initialized_ && diag > 1);
    if (diag > degrade_accept_max_ || pre_init_dirty)
    {
        ROS_WARN_THROTTLE(1.0, "[LIO] Dropping frame: voxel_slam degrade_state=%u (accept_max=%d%s)",
                          static_cast<unsigned>(diag), degrade_accept_max_,
                          pre_init_dirty ? ", pre-init requires clean" : "");
        perf_frames_dropped_.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    const bool diag_degenerate = (diag > 1);  // Low/Medium/High -> accept but downweight

    // Gate 1: inter-frame position jump
    const Eigen::Vector3d frame_delta = (last_odom_tf.inverse() * current_frame_.pose_eig_).block<3,1>(0,3);
    const double jump_dist = frame_delta.norm();
    if (first_odom_received_ && jump_dist > max_odom_jump_m_)
    {
        ROS_WARN_THROTTLE(1.0, "[LIO] Jump detected: %.2f m (threshold %.2f m) -- dropping frame",
                          jump_dist, max_odom_jump_m_);
        perf_frames_dropped_.fetch_add(1, std::memory_order_relaxed);

        if (reinit_on_jump_ && is_initialized_)
        {
            {
                std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
                odom_delta_ = Eigen::Matrix4d::Identity();
            }
            if (init_from_tf_)
            {
                try
                {
                    tf::StampedTransform tf_stamped;
                    tf_listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), tf_stamped);
                    Eigen::Affine3d tf_eigen;
                    tf::transformTFToEigen(tf_stamped, tf_eigen);
                    Eigen::Matrix4d anchor = tf_eigen.matrix();
                    if (gps_handler_.firstReceived())
                        anchor(2, 3) = gps_handler_.latestGpsZ();
                    {
                        std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
                        last_corrected_pose_ = anchor;
                    }
                    ROS_INFO("[LIO] Re-anchor from TF %s -> %s: t=(%.2f, %.2f, %.2f) -- map preserved (%zu kf)",
                             map_frame_.c_str(), robot_frame_.c_str(),
                             anchor(0,3), anchor(1,3), anchor(2,3), keyframes_.size());
                }
                catch (const tf::TransformException& ex)
                {
                    ROS_WARN("[LIO] TF lookup failed for re-anchor (%s) -- keeping last corrected pose", ex.what());
                }
            }
            gps_handler_.armReentry();
            post_reinit_frames_remaining_ = reinit_skip_frames_;
            // The odom between-factor that will bridge the last pre-reset keyframe to the
            // first post-reset keyframe spans un-tracked motion during the outage. Flag it so
            // it is staged with loose (degenerate) noise instead of tight odom noise — otherwise
            // the graph honours a tight-but-wrong transform and distorts poses around the reset.
            bridge_after_reinit_ = true;
        }
        else if (!reinit_on_jump_)
        {
            current_frame_.pose_eig_ = last_odom_tf;
        }
        return false;
    }

    // Gate 2: FAST-LIO covariance. Combined with the diag signal from Gate 0 so a frame flagged
    // degenerate by either source gets the loose odom noise (works even if lio_cov_threshold_ <= 0).
    first_odom_received_ = true;
    bool cov_degenerate = false;
    if (lio_cov_threshold_ > 0.0)
    {
        const double pos_cov_trace = odom_msg->pose.covariance[0]
                                   + odom_msg->pose.covariance[7]
                                   + odom_msg->pose.covariance[14];
        if (pos_cov_trace > lio_cov_threshold_)
            ROS_WARN_THROTTLE(2.0, "[LIO] Degenerate frame (cov trace=%.4f > %.4f)",
                              pos_cov_trace, lio_cov_threshold_);
        cov_degenerate = (pos_cov_trace > lio_cov_threshold_);
    }
    current_frame_.is_degenerate_ = diag_degenerate || cov_degenerate;
    return true;
}

// ── publishRealtimePose ───────────────────────────────────────────────────────

void FastLioSamScQn::publishRealtimePose(const nav_msgs::OdometryConstPtr& odom_msg,
                                         const Eigen::Matrix4d& last_odom_tf)
{
    std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
    // Accumulate incremental LIO motion since the last corrected keyframe.
    // odom_delta_ is reset to Identity after each keyframe commit.
    odom_delta_ = odom_delta_ * last_odom_tf.inverse() * current_frame_.pose_eig_;
    current_frame_.pose_corrected_eig_ = last_corrected_pose_ * odom_delta_;
    if (is_initialized_)
    {
        realtime_pose_pub_.publish(poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_));
        {
            nav_msgs::Odometry slam_odom;
            slam_odom.header.stamp    = odom_msg->header.stamp;
            slam_odom.header.frame_id = map_frame_;
            slam_odom.child_frame_id  = robot_frame_;
            slam_odom.pose.pose       = poseEigToPoseStamped(current_frame_.pose_corrected_eig_, map_frame_).pose;
            slam_odom.twist           = odom_msg->twist;

            // Populate pose covariance from the ISAM2 marginal of the last committed keyframe.
            // pose_covariance_ is in GTSAM body-frame tangent space ordering: [Rx,Ry,Rz, tx,ty,tz].
            // Step 1: rotate body-frame covariance into the world (map) frame — J = diag(R, R).
            // Step 2: reorder blocks from GTSAM [R,T] to ROS [T,R] convention.
            {
                const Eigen::Matrix3d R = current_frame_.pose_corrected_eig_.block<3,3>(0,0);
                Eigen::Matrix<double,6,6> J = Eigen::Matrix<double,6,6>::Zero();
                J.block<3,3>(0,0) = R;
                J.block<3,3>(3,3) = R;
                const Eigen::Matrix<double,6,6> wc = J * pose_covariance_ * J.transpose();
                // Swap [R,T] → [T,R] blocks for ROS nav_msgs/Odometry convention.
                Eigen::Matrix<double,6,6> ros_cov;
                ros_cov.block<3,3>(0,0) = wc.block<3,3>(3,3);  // T-T
                ros_cov.block<3,3>(0,3) = wc.block<3,3>(3,0);  // T-R
                ros_cov.block<3,3>(3,0) = wc.block<3,3>(0,3);  // R-T
                ros_cov.block<3,3>(3,3) = wc.block<3,3>(0,0);  // R-R
                Eigen::Map<Eigen::Matrix<double,6,6,Eigen::RowMajor>>(
                    slam_odom.pose.covariance.data()) = ros_cov;
            }
            slam_odom_pub_.publish(slam_odom);
        }
        if (publish_tf_)
            broadcaster_.sendTransform(tf::StampedTransform(
                poseEigToROSTf(current_frame_.pose_corrected_eig_),
                ros::Time::now(), map_frame_, robot_frame_));
    }
    if (is_initialized_ && corrected_current_pcd_pub_.getNumSubscribers() > 0)
        corrected_current_pcd_pub_.publish(
            pclToPclRos(transformPcd(current_frame_.pcd_, renderPose(current_frame_.pose_corrected_eig_)), map_frame_));
}

// ── tryInitialize ─────────────────────────────────────────────────────────────

void FastLioSamScQn::tryInitialize()
{
    if (!gps_handler_.isReadyToInit())
    {
        // Log which signal we're waiting for (heading topic or GPS only).
        if (!sub_heading_.getTopic().empty())
            ROS_INFO_THROTTLE(5.0, "[Init] Waiting for joint GPS+heading stability window (%s)...",
                              sub_heading_.getTopic().c_str());
        else
            ROS_INFO_THROTTLE(5.0, "[Init] Waiting for stable GPS position (%s)...",
                              sub_gps_.getTopic().c_str());
        return;
    }

    // Capture TF exactly once, at the moment the stability window first passes.
    if (!heading_tf_captured_)
    {
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
                ROS_INFO("[Init] TF at init time %s -> %s: t=(%.2f, %.2f, %.2f)",
                         map_frame_.c_str(), robot_frame_.c_str(),
                         tf_stamped.getOrigin().x(), tf_stamped.getOrigin().y(), tf_stamped.getOrigin().z());
            }
            catch (const tf::TransformException& ex)
            {
                ROS_WARN("[Init] TF lookup failed at init time (%s) -- using accumulated LIO position", ex.what());
            }
        }
        heading_tf_captured_ = true;
    }

    // Build initial pose.
    const Eigen::Matrix4d& init_tf_source =
        (init_from_tf_ && tf_at_heading_valid_) ? tf_at_heading_pose_
                                                 : current_frame_.pose_corrected_eig_;
    gtsam::Pose3 init_pose = poseEigToGtsamPose(init_tf_source);

    const GpsHandler::InitSnapshot snap = gps_handler_.getInitSnapshot();
    init_lat_ = snap.lat;
    init_lon_ = snap.lon;
    init_alt_ = snap.z;
    init_x_   = snap.x;
    init_y_   = snap.y;
    const bool have_gps_heading = !std::isnan(snap.yaw);
    if (have_gps_heading)
    {
        const gtsam::Rot3 lio_rot = init_pose.rotation();
        const gtsam::Rot3 gps_rot = gtsam::Rot3::RzRyRx(lio_rot.roll(), lio_rot.pitch(), snap.yaw);
        init_pose = gtsam::Pose3(gps_rot, init_pose.translation());
        ROS_INFO("\033[1;32m[Init] Using GPS heading yaw=%.1f deg for GTSAM prior.\033[0m",
                 snap.yaw * 180.0 / M_PI);
    }
    else
    {
        ROS_WARN("[Init] No GPS/heading yaw at init -- origin yaw taken from %s and left LOOSE "
                 "(var=%.3g rad^2) so GPS + motion can rotate the map.",
                 (init_from_tf_ && tf_at_heading_valid_) ? "TF" : "LIO",
                 init_prior_noise_yaw_unknown_);
    }
    init_pose = gtsam::Pose3(init_pose.rotation(), gtsam::Point3(snap.x, snap.y, snap.z));
    ROS_INFO("\033[1;32m[Init] Using GPS position for origin: (%.2f, %.2f, %.2f)\033[0m",
             snap.x, snap.y, snap.z);

    current_frame_.pose_corrected_eig_ = gtsamPoseToPoseEig(init_pose);
    {
        std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
        odom_delta_          = Eigen::Matrix4d::Identity();
        last_corrected_pose_ = current_frame_.pose_corrected_eig_;
    }

    keyframes_.push_back(current_frame_);
    {
        std::lock_guard<std::mutex> lk(vis_mutex_);
        updateOdomsAndPaths(current_frame_);
    }

    // Yaw prior is tight only when a GPS/heading source fixed the initial yaw. Without one,
    // the map's absolute rotation is unknown at init; a tight yaw prior would lock the whole
    // map to LIO's arbitrary start heading and fight later GPS position factors.
    const double yaw_var = have_gps_heading ? 1e-4 : init_prior_noise_yaw_unknown_;
    auto var = (gtsam::Vector(6) << 1e-4, 1e-4, yaw_var, 1e-2, 1e-2, init_prior_noise_z_).finished();
    auto prior_noise = gtsam::noiseModel::Diagonal::Variances(var);
    // initGraph inserts key 0 into staged_init_ — do NOT call stageInitValue(0) afterwards.
    isam_backend_.initGraph(init_pose, prior_noise, 0);

    // Commit the prior factor immediately (no odom factor on the very first node).
    auto result = isam_backend_.commit(/*structural=*/false, /*gps_added=*/false, /*gps_lm_passes=*/0);
    if (result.ok)
    {
        std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
        corrected_esti_      = std::move(result.estimate);
        pose_covariance_     = result.marginal_cov;
    }

    current_keyframe_idx_++;
    loop_closure_->updateScancontext(current_frame_.pcd_);
    is_initialized_ = true;
    ROS_INFO("\033[1;32m[Init] GTSAM graph initialised. Node ready.\033[0m");
}

// ── processKeyframe ───────────────────────────────────────────────────────────

void FastLioSamScQn::processKeyframe(const nav_msgs::OdometryConstPtr& odom_msg)
{
    // Push keyframe.
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        keyframes_.push_back(current_frame_);
    }

    // Accumulate step distance for GPS traveled-dist gate.
    if (keyframes_.size() >= 2)
        gps_total_path_length_ += (keyframes_.back().pose_corrected_eig_.block<3,1>(0,3) -
                                   keyframes_[keyframes_.size()-2].pose_corrected_eig_.block<3,1>(0,3)).norm();

    const int    prev_idx = current_keyframe_idx_ - 1;
    const int    curr_idx = current_keyframe_idx_;
    const gtsam::Pose3 pose_from = poseEigToGtsamPose(keyframes_[prev_idx].pose_corrected_eig_);
    const gtsam::Pose3 pose_to   = poseEigToGtsamPose(current_frame_.pose_corrected_eig_);

    // Stage odometry factor.
    // Loosen the odom factor for the first keyframe after a LIO reinit (bridge across the
    // degeneracy/outage gap), so GPS can re-pin it without a tight-but-wrong constraint fighting.
    const bool bridge = bridge_after_reinit_;
    bridge_after_reinit_ = false;
    if (bridge)
        ROS_WARN("[LIO] Bridging keyframe %d after reinit -- odom position loosened, rotation kept tight",
                 curr_idx);
    else if (current_frame_.is_degenerate_)
        ROS_INFO_THROTTLE(1.0, "[LIO] Degenerate keyframe %d accepted -- odom factor loosened", curr_idx);
    isam_backend_.stageOdomFactor(prev_idx, curr_idx, pose_from, pose_to,
                                  current_frame_.is_degenerate_, bridge);
    isam_backend_.stageInitValue(curr_idx, pose_to);

    // Build additional factors for this keyframe.
    gtsam::NonlinearFactorGraph kf_factors;
    bool structural = false;

    // GPS position (and optional GPS-derived heading) factor.
    auto gps_result = gps_handler_.tryAddFactor(odom_msg->header.stamp.toSec(), curr_idx,
                                                  gps_total_path_length_,
                                                  current_frame_.pose_corrected_eig_,
                                                  kf_factors);
    if (gps_result.factor_added)
    {
        structural = true;
        perf_gps_accepted_.fetch_add(1, std::memory_order_relaxed);
        if (gps_result.ground_z_updated)
        {
            ground_z_ref_   = gps_result.new_ground_z;
            ground_z_ready_ = true;
        }
        // Publish GPS markers immediately so RViz sees them without waiting for vis timer.
        gps_constraint_pub_.publish(gps_handler_.getGpsMarkers(map_frame_));
    }

    // Ground Z prior.
    if (use_ground_prior_ && ground_z_ready_)
        kf_factors.add(ZHeightFactor(curr_idx, ground_z_ref_, ground_prior_noise_));

    isam_backend_.stageFactors(kf_factors);
    current_keyframe_idx_++;
    loop_closure_->updateScancontext(current_frame_.pcd_);

    {
        std::lock_guard<std::mutex> lk(vis_mutex_);
        updateOdomsAndPaths(current_frame_);
    }

    // Adaptive LM passes for GPS re-entry.
    int gps_lm_passes = 1;
    if (gps_result.triggers_lm)
    {
        const BackendParams& bp = isam_backend_.params();  // accessor needed — see note
        if (bp.lm_max_passes > 1 && bp.lm_max_distance > 0.0)
        {
            const double step = bp.lm_max_distance / static_cast<double>(bp.lm_max_passes);
            gps_lm_passes = std::max(1, std::min(bp.lm_max_passes,
                               static_cast<int>(std::floor(gps_result.lm_dist_gap / step))));
        }
    }

    // Commit to ISAM2.
    auto result = isam_backend_.commit(structural, gps_result.triggers_lm, gps_lm_passes);

    if (result.ok)
    {
        {
            std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
            corrected_esti_      = result.estimate;
            last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size()-1));
            odom_delta_          = Eigen::Matrix4d::Identity();
        }
        pose_covariance_ = result.marginal_cov;

        if (result.needs_vis)
        {
            {
                std::lock_guard<std::mutex> lk(keyframes_mutex_);
                for (size_t i = 0; i < result.estimate.size(); ++i)
                    keyframes_[i].pose_corrected_eig_ =
                        gtsamPoseToPoseEig(result.estimate.at<gtsam::Pose3>(i));
            }
            loop_added_flag_vis_ = true;
        }

        // Yaw-drift diagnostic: quantify how much ISAM2 rotated this node relative to LIO.
        // Large Δ at GPS factor nodes indicates accumulated yaw drift between constraints.
        if (gps_result.factor_added)
        {
            const double lio_yaw  = pose_to.rotation().yaw() * 180.0 / M_PI;
            const double slam_yaw = result.estimate.at<gtsam::Pose3>(curr_idx).rotation().yaw() * 180.0 / M_PI;
            double delta = slam_yaw - lio_yaw;
            while (delta >  180.0) delta -= 360.0;
            while (delta < -180.0) delta += 360.0;
            ROS_INFO("[GPS] Yaw d @node %d: LIO %.1f deg -> SLAM %.1f deg (D=%.2f deg)",
                     curr_idx, lio_yaw, slam_yaw, delta);
        }
    }
}

// ── loopTimerFunc ─────────────────────────────────────────────────────────────

void FastLioSamScQn::loopTimerFunc(const ros::TimerEvent& /*event*/)
{
    if (!is_initialized_)
        return;

    // Self-heal: if the base->lidar TF only became available after startup, cache it now and
    // hand it to loop closure so its submaps compose the lever arm (idempotent once ready).
    if (input_pcd_lidar_frame_ && !lidar_extrinsic_ready_)
    {
        ensureLidarExtrinsic(0.0, false);
        if (lidar_extrinsic_ready_)
            loop_closure_->setRenderExtrinsic(T_base_lidar_);
    }

    // ── Snapshot the latest unprocessed keyframe under lock ──────────────────
    // keyframes_ is mutated by processKeyframe() (running on spinner threads via
    // AsyncSpinner(4)).  Taking a reference to back() and using it outside the
    // lock is a use-after-free risk when push_back triggers reallocation.
    // Copy the PosePcd under the lock and work on the copy; remaining index-based
    // accesses to keyframes_[] are safe because the vector only grows (existing
    // elements are never erased or moved once inserted).
    PosePcd latest_keyframe;
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        if (keyframes_.empty())
            return;
        auto& back = keyframes_.back();
        if (back.processed_)
            return;
        back.processed_ = true;
        latest_keyframe = back;   // deep copy — safe after lock release
    }

    const high_resolution_clock::time_point t1 = high_resolution_clock::now();

    const int closest_keyframe_idx = loop_closure_->fetchCandidateKeyframeIdx(latest_keyframe, keyframes_);
    if (closest_keyframe_idx < 0)
        return;

    const int min_sep = isam_backend_.minLoopKfSep();
    if (std::abs(latest_keyframe.idx_ - closest_keyframe_idx) < min_sep)
    {
        ROS_DEBUG_THROTTLE(5.0, "[Loop] Candidate kf %d too close (gap %d < min %d) -- skipping",
                  closest_keyframe_idx,
                  std::abs(latest_keyframe.idx_ - closest_keyframe_idx), min_sep);
        return;
    }

    const int src_bucket = latest_keyframe.idx_ / min_sep;
    const int dst_bucket = closest_keyframe_idx / min_sep;
    if (isam_backend_.isLoopDuplicate(src_bucket, dst_bucket))
    {
        ROS_DEBUG("[Loop] Duplicate suppressed: src bucket %d / dst bucket %d", src_bucket, dst_bucket);
        return;
    }

    const RegistrationOutput& reg_output =
        loop_closure_->performLoopClosure(latest_keyframe, keyframes_, closest_keyframe_idx);

    if (reg_output.is_valid_)
    {
        ROS_INFO("\033[1;32mLoop closure accepted. Score: %.3f  Overlap: %.0f%% (kf %d -> %d, gap %d)\033[0m",
                 reg_output.score_, 100.0 * reg_output.overlap_ratio_,
                 latest_keyframe.idx_, closest_keyframe_idx,
                 std::abs(latest_keyframe.idx_ - closest_keyframe_idx));

        const bool accepted = isam_backend_.tryAddLoop(
            latest_keyframe.idx_, closest_keyframe_idx,
            src_bucket, dst_bucket,
            reg_output.pose_between_eig_,
            latest_keyframe.pose_corrected_eig_,
            keyframes_[closest_keyframe_idx].pose_corrected_eig_,
            reg_output.score_);

        if (accepted)
        {
            loop_added_flag_vis_ = true;
            // loops_accepted already incremented inside isam_backend_.tryAddLoop()
        }
        else
        {
            // Yaw gate rejection — backend did not count this
            isam_backend_.loops_rejected.fetch_add(1, std::memory_order_relaxed);
        }
    }
    else
    {
        ROS_WARN("[Loop] Rejected: score=%.3f  overlap=%.0f%% (kf %d -> %d)",
                 reg_output.score_, 100.0 * reg_output.overlap_ratio_,
                 latest_keyframe.idx_, closest_keyframe_idx);
        isam_backend_.loops_rejected.fetch_add(1, std::memory_order_relaxed);
    }

    const high_resolution_clock::time_point t2 = high_resolution_clock::now();

    if (debug_src_pub_.getNumSubscribers() > 0)
        debug_src_pub_.publish(pclToPclRos(loop_closure_->getSourceCloud(), map_frame_));
    if (debug_dst_pub_.getNumSubscribers() > 0)
        debug_dst_pub_.publish(pclToPclRos(loop_closure_->getTargetCloud(), map_frame_));
    if (debug_fine_aligned_pub_.getNumSubscribers() > 0)
        debug_fine_aligned_pub_.publish(pclToPclRos(loop_closure_->getFinalAlignedCloud(), map_frame_));
    if (debug_coarse_aligned_pub_.getNumSubscribers() > 0)
        debug_coarse_aligned_pub_.publish(pclToPclRos(loop_closure_->getCoarseAlignedCloud(), map_frame_));

    ROS_DEBUG("[Loop] Timer: %.1f ms", duration_cast<microseconds>(t2 - t1).count() / 1e3);

    if (show_perf_stats_)
    {
        const double ms = duration_cast<microseconds>(t2 - t1).count() / 1e3;
        std::lock_guard<std::mutex> lk(perf_mutex_);
        perf_loop_time_sum_ms_ += ms;
        perf_loop_time_max_ms_  = std::max(perf_loop_time_max_ms_, ms);
        ++perf_loop_count_window_;
    }
}

// ── visTimerFunc ──────────────────────────────────────────────────────────────

void FastLioSamScQn::visTimerFunc(const ros::TimerEvent& /*event*/)
{
    if (!is_initialized_)
        return;

    ensureLidarExtrinsic(0.0, false);  // recover the base->lidar extrinsic if TF was late at startup

    const high_resolution_clock::time_point tv1 = high_resolution_clock::now();

    if (loop_added_flag_vis_)
    {
        gtsam::Values corrected_esti_copied;
        pcl::PointCloud<pcl::PointXYZ> corrected_odoms;
        nav_msgs::Path corrected_path;
        {
            std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
            corrected_esti_copied = corrected_esti_;
        }
        for (size_t i = 0; i < corrected_esti_copied.size(); ++i)
        {
            const gtsam::Pose3 p = corrected_esti_copied.at<gtsam::Pose3>(i);
            corrected_odoms.points.emplace_back(p.translation().x(), p.translation().y(), p.translation().z());
            corrected_path.poses.push_back(gtsamPoseToPoseStamped(p, map_frame_));
        }
        if (isam_backend_.hasLoops())
            loop_detection_pub_.publish(isam_backend_.getLoopMarkers(corrected_esti_copied, map_frame_));
        {
            std::lock_guard<std::mutex> lk(vis_mutex_);
            corrected_odoms_          = corrected_odoms;
            corrected_path_.poses     = corrected_path.poses;
        }
        loop_added_flag_vis_  = false;
        global_map_vis_switch_ = true;
    }

    if (gps_handler_.hasGpsConstraints())
        gps_constraint_pub_.publish(gps_handler_.getGpsMarkers(map_frame_));

    {
        std::lock_guard<std::mutex> lk(vis_mutex_);
        odom_pub_.publish(pclToPclRos(odoms_, map_frame_));
        path_pub_.publish(odom_path_);
        corrected_odom_pub_.publish(pclToPclRos(corrected_odoms_, map_frame_));
        corrected_path_pub_.publish(corrected_path_);
    }

    if (global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() > 0)
    {
        pcl::PointCloud<PointType>::Ptr corrected_map(new pcl::PointCloud<PointType>());
        corrected_map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());
        {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            for (size_t i = 0; i < keyframes_.size(); ++i)
                *corrected_map += transformPcd(keyframes_[i].pcd_, renderPose(keyframes_[i].pose_corrected_eig_));
        }
        const auto& voxelized = voxelizePcd(corrected_map, voxel_res_);
        corrected_pcd_map_pub_.publish(pclToPclRos(*voxelized, map_frame_));
        global_map_vis_switch_ = false;
    }
    if (!global_map_vis_switch_ && corrected_pcd_map_pub_.getNumSubscribers() == 0)
        global_map_vis_switch_ = true;

    ROS_DEBUG("[Vis] Timer: %.1f ms",
              duration_cast<microseconds>(high_resolution_clock::now() - tv1).count() / 1e3);
}

// ── lmRefineSrvCallback ───────────────────────────────────────────────────────

bool FastLioSamScQn::lmRefineSrvCallback(grover_msgs::SrvInt16::Request& req,
                                           grover_msgs::SrvInt16::Response& res)
{
    if (!is_initialized_)
    {
        res.success = false;
        res.message = "SLAM not yet initialized -- no graph to refine";
        return true;
    }

    const BackendParams& bp = isam_backend_.params();
    // req.data == 0  → use the config default (lm_max_passes).
    // req.data  > 0  → use that many passes, capped at 100.
    const int requested = static_cast<int>(req.data);
    const int passes = (requested <= 0) ? bp.lm_max_passes
                                        : std::min(requested, 100);
    const bool ok = isam_backend_.runLMRefinement(passes, "SRV");
    if (!ok)
    {
        res.success = false;
        res.message = "LM refinement skipped or failed (check logs for details)";
        return true;
    }

    const gtsam::Values new_esti = isam_backend_.calculateEstimate();
    if (new_esti.empty())
    {
        res.success = false;
        res.message = "LM succeeded but calculateEstimate() returned empty";
        return true;
    }

    {
        std::lock_guard<std::mutex> lk(realtime_pose_mutex_);
        corrected_esti_      = new_esti;
        last_corrected_pose_ = gtsamPoseToPoseEig(corrected_esti_.at<gtsam::Pose3>(corrected_esti_.size()-1));
        odom_delta_          = Eigen::Matrix4d::Identity();
    }
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        for (size_t i = 0; i < new_esti.size(); ++i)
            keyframes_[i].pose_corrected_eig_ = gtsamPoseToPoseEig(new_esti.at<gtsam::Pose3>(i));
    }
    loop_added_flag_vis_ = true;

    res.success = true;
    res.message = "LM refinement completed -- corrected_esti and keyframes updated";
    return true;
}

// ── saveFlagCallback ──────────────────────────────────────────────────────────

void FastLioSamScQn::saveFlagCallback(const std_msgs::String::ConstPtr& msg)
{
    if (!save_map_pcd_) return;
    const std::string base_dir = msg->data.empty() ? package_path_ : msg->data;
    saveMapPcd(base_dir);
}


// ── saveMapPcd ────────────────────────────────────────────────────────────────

std::string FastLioSamScQn::saveMapPcd(const std::string& base_dir)
{
    if (keyframes_.empty()) return {};

    ensureLidarExtrinsic(1.0, true);  // render the saved map with the base->lidar extrinsic

    // Build timestamped subdirectory: base_dir/YYYY-MM-DD-HH-MM/
    const std::time_t now = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y-%m-%d-%H-%M", std::localtime(&now));
    const fs::path out_dir = fs::path(base_dir) / ts;

    std::error_code ec;
    fs::create_directories(out_dir, ec);
    if (ec)
    {
        ROS_ERROR("[Save] Failed to create directory %s: %s", out_dir.c_str(), ec.message().c_str());
        return {};
    }

    // Assemble voxelized map.
    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>());
    map->reserve(keyframes_[0].pcd_.size() * keyframes_.size());
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        for (size_t i = 0; i < keyframes_.size(); ++i)
            *map += transformPcd(keyframes_[i].pcd_, renderPose(keyframes_[i].pose_corrected_eig_));
    }
    const auto& voxelized = voxelizePcd(map, voxel_res_);

    const std::string pcd_path  = (out_dir / "cloud.pcd").string();
    const std::string yaml_path = (out_dir / "metadata.yaml").string();

    pcl::io::savePCDFileASCII<PointType>(pcd_path, *voxelized);
    ROS_INFO("\033[1;32m[Save] Map saved -> %s  (%zu pts)\033[0m",
             pcd_path.c_str(), voxelized->size());

    // Write metadata.yaml
    {
        std::ofstream ofs(yaml_path);
        ofs << std::fixed << std::setprecision(9);
        ofs << "# SLAM map metadata — generated by fast_lio_sam_sc_qn\n";
        ofs << "timestamp: \"" << ts << "\"\n";
        ofs << "map_frame: \"" << map_frame_ << "\"\n";
        // Frame convention for downstream tools (grover_slam_tools georef): the map
        // is a true-north, true-metric local ENU frame with datum at origin.init_gps,
        // so map->UTM is a geodetic projection (ENU->ECEF->UTM), NOT a translation.
        ofs << "frame_convention: \"local-ENU-true-north; datum=origin.init_gps; "
               "map_to_utm=geodetic-projection\"\n";
        ofs << "voxel_resolution: " << voxel_res_ << "\n";
        ofs << "keyframes: " << keyframes_.size() << "\n";
        ofs << "origin:\n";

        // PHASE 2: prefer the GTSAM node-0 datum captured at init (its exact
        // WGS-84 fix + map position). The map frame is a true-north local ENU
        // frame pinned at that datum (see gps_handler onGpsOdom), so node-0
        // lat/lon paired with init_utm re-project to UTM exactly (grover_slam_tools
        // georef). The /toLL service reflects navsat's UTM-grid transform, whose
        // meridian convergence is uncorrected -- for the map origin ~5 m off node 0
        // that mislocates the datum by ~0.1-0.2 m and shifts the whole cloud. So
        // use /toLL only as a fallback when no init datum is available.
        bool got_gps_from_toll = false;
        double gps_lat = std::numeric_limits<double>::quiet_NaN();
        double gps_lon = std::numeric_limits<double>::quiet_NaN();
        double gps_alt = std::numeric_limits<double>::quiet_NaN();

        if (!std::isnan(init_lat_) && !std::isnan(init_lon_))
        {
            ROS_INFO("[Save] Origin GPS from GTSAM node-0 init datum (exact; true-north ENU frame).");
        }
        else if (to_ll_client_.exists())
        {
            robot_localization::ToLL toll_req;
            toll_req.request.map_point.x = 0.0;
            toll_req.request.map_point.y = 0.0;
            toll_req.request.map_point.z = 0.0;
            if (to_ll_client_.call(toll_req))
            {
                gps_lat = toll_req.response.ll_point.latitude;
                gps_lon = toll_req.response.ll_point.longitude;
                gps_alt = toll_req.response.ll_point.altitude;
                got_gps_from_toll = true;
                ROS_INFO("[Save] Origin GPS from /toLL(0,0,0): lat=%.9f lon=%.9f alt=%.3f",
                         gps_lat, gps_lon, gps_alt);
            }
            else
            {
                ROS_WARN("[Save] /toLL service call failed; falling back to GTSAM node 0 position.");
            }
        }
        else
        {
            ROS_WARN("[Save] /toLL service not advertised; falling back to GTSAM node 0 position.");
        }

        if (got_gps_from_toll)
        {
            ofs << "  description: \"Map origin (0,0,0) converted to WGS-84 via /toLL service\"\n";
            ofs << "  init_gps:\n";
            ofs << "    latitude:  " << gps_lat << "  # WGS-84 of map origin via /toLL\n";
            ofs << "    longitude: " << gps_lon << "\n";
            ofs << "    altitude:  " << gps_alt << "  # metres\n";
        }
        else if (!std::isnan(init_lat_) && !std::isnan(init_lon_))
        {
            ofs << "  description: \"GTSAM node 0 — position at SLAM initialisation\"\n";
            ofs << "  init_gps:\n";
            ofs << "    latitude:  " << init_lat_ << "  # WGS-84 at init time\n";
            ofs << "    longitude: " << init_lon_ << "\n";
            ofs << "    altitude:  " << init_alt_ << "  # metres\n";
        }
        else
        {
            ofs << "  description: \"GTSAM node 0 — position at SLAM initialisation\"\n";
            ofs << "  init_gps:\n";
            ofs << "    latitude:  null  # GPS fix not available at init\n";
            ofs << "    longitude: null\n";
            ofs << "    altitude:  null\n";
        }
        // init_utm MUST be the map-frame position of the point init_gps
        // describes, so georeferencing pins that map point to that WGS-84 fix.
        // On the /toLL path init_gps is the map origin (0,0,0), so init_utm is
        // (0,0,0); only on the node-0 fallback is it node 0's map position.
        // (Writing node 0's offset alongside the origin's GPS shifts the whole
        // georeferenced cloud by that offset — see grover_slam_tools georef.)
        if (got_gps_from_toll)
        {
            ofs << "  init_utm:\n";
            ofs << "    description: \"Map origin (0,0,0) map-frame position\"\n";
            ofs << "    x: 0.0  # easting  [m]\n";
            ofs << "    y: 0.0  # northing [m]\n";
            ofs << "    z: 0.0  # altitude [m]\n";
        }
        else if (!std::isnan(init_x_) && !std::isnan(init_y_))
        {
            ofs << "  init_utm:\n";
            ofs << "    description: \"GTSAM node 0 map-frame position at initialisation\"\n";
            ofs << "    x: " << init_x_ << "  # easting  [m]\n";
            ofs << "    y: " << init_y_ << "  # northing [m]\n";
            ofs << "    z: " << init_alt_ << "  # altitude [m]\n";
        }
        else
        {
            ofs << "  init_utm: null  # GPS position not available at init\n";
        }
    }
    ROS_INFO("\033[1;32m[Save] Metadata -> %s\033[0m", yaml_path.c_str());

    // ── Session dump for offline PGO ──────────────────────────────────────────
    // 1) keyframes.csv — pose snapshots (copied under lock, written outside)
    {
        struct KfSnap { int idx; double stamp; bool is_degenerate; Eigen::Matrix4d pose; };
        std::vector<KfSnap> snaps;
        {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            snaps.reserve(keyframes_.size());
            for (const auto& kf : keyframes_)
                snaps.push_back({kf.idx_, kf.timestamp_, kf.is_degenerate_,
                                 kf.pose_corrected_eig_});
        }
        std::ofstream kf_csv((out_dir / "keyframes.csv").string());
        kf_csv << std::fixed << std::setprecision(9);
        kf_csv << "idx,stamp,is_degenerate";
        for (int i = 0; i < 16; ++i) kf_csv << ",m" << i;  // row-major in file (written below)
        kf_csv << "\n";
        for (const auto& s : snaps)
        {
            kf_csv << s.idx << "," << s.stamp << "," << static_cast<int>(s.is_degenerate);
            for (int r = 0; r < 4; ++r)
                for (int c = 0; c < 4; ++c)
                    kf_csv << "," << s.pose(r, c);  // row-major in file
            kf_csv << "\n";
        }
        ROS_INFO("\033[1;32m[Save] Keyframes -> %s  (%zu rows)\033[0m",
                 (out_dir / "keyframes.csv").c_str(), snaps.size());
    }

    // 2) Per-keyframe point clouds in sensor frame (for offline ICP).
    //    Written under keyframes_mutex_ to guarantee a consistent snapshot.
    {
        const fs::path clouds_dir = out_dir / "clouds";
        std::error_code ec2;
        fs::create_directories(clouds_dir, ec2);
        if (ec2)
        {
            ROS_ERROR("[Save] Cannot create clouds dir: %s", ec2.message().c_str());
        }
        else
        {
            char name[16];
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            for (const auto& kf : keyframes_)
            {
                std::snprintf(name, sizeof(name), "%06d.pcd", kf.idx_);
                pcl::io::savePCDFileBinary((clouds_dir / name).string(), kf.pcd_);
            }
            ROS_INFO("\033[1;32m[Save] %zu keyframe clouds -> %s/\033[0m",
                     keyframes_.size(), clouds_dir.c_str());
        }
    }

    // 3) SC descriptors
    loop_closure_->saveDescriptors(out_dir.string());

    // Signal listeners (map_export) that a map was saved, so they can
    // auto-georeference it to LAZ using the Python georef tools.
    {
        std_msgs::String saved_msg;
        saved_msg.data = out_dir.string();
        map_saved_pub_.publish(saved_msg);
    }
    return out_dir.string();
}

// ── saveMapSrvCallback ────────────────────────────────────────────────────────

bool FastLioSamScQn::saveMapSrvCallback(std_srvs::Trigger::Request& /*req*/,
                                         std_srvs::Trigger::Response& res)
{
    if (!is_initialized_ || keyframes_.empty())
    {
        res.success = false;
        res.message = "SLAM not yet initialized — no map to save";
        return true;
    }

    const std::string out_dir = saveMapPcd(package_path_);
    if (out_dir.empty())
    {
        res.success = false;
        res.message = "saveMapPcd failed — check logs";
        return true;
    }

    // Contract: on success, `message` is EXACTLY the output directory (no prose),
    // so callers (e.g. grover_navigation map_export) can consume it directly.
    // Human-readable status is logged by saveMapPcd via ROS_INFO.
    res.success = true;
    res.message = out_dir;
    return true;
}

// ── lioDiagCallback ───────────────────────────────────────────────────────────

void FastLioSamScQn::lioDiagCallback(const voxel_slam::LIODiagConstPtr& msg)
{
    latest_diag_state_.store(msg->degrade_state);
}

// ── Utilities ─────────────────────────────────────────────────────────────────

void FastLioSamScQn::updateOdomsAndPaths(const PosePcd& pose_pcd_in)
{
    odoms_.points.emplace_back(pose_pcd_in.pose_eig_(0,3),
                               pose_pcd_in.pose_eig_(1,3),
                               pose_pcd_in.pose_eig_(2,3));
    corrected_odoms_.points.emplace_back(pose_pcd_in.pose_corrected_eig_(0,3),
                                          pose_pcd_in.pose_corrected_eig_(1,3),
                                          pose_pcd_in.pose_corrected_eig_(2,3));
    odom_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_eig_, map_frame_));
    corrected_path_.poses.emplace_back(poseEigToPoseStamped(pose_pcd_in.pose_corrected_eig_, map_frame_));
}

bool FastLioSamScQn::checkIfKeyframe(const PosePcd& a, const PosePcd& b) const
{
    return keyframe_thr_ <
        (b.pose_corrected_eig_.block<3,1>(0,3) - a.pose_corrected_eig_.block<3,1>(0,3)).norm();
}

// ── cloudSparsifyThread ───────────────────────────────────────────────────────

void FastLioSamScQn::cloudSparsifyThread()
{
    while (!cloud_sparsify_stop_.load(std::memory_order_relaxed))
    {
        std::this_thread::sleep_for(std::chrono::seconds(2));

        int boundary;
        {
            std::lock_guard<std::mutex> lk(keyframes_mutex_);
            boundary = static_cast<int>(keyframes_.size()) - cloud_sparsify_age_;
        }
        if (boundary <= 0) continue;

        for (int i = 0; i < boundary; ++i)
        {
            if (cloud_sparsify_stop_.load(std::memory_order_relaxed)) break;

            pcl::PointCloud<PointType> cloud_copy;
            {
                std::lock_guard<std::mutex> lk(keyframes_mutex_);
                if (i >= static_cast<int>(keyframes_.size())) break;
                if (keyframes_[i].cloud_sparsified_) continue;
                cloud_copy = keyframes_[i].pcd_;
            }
            if (cloud_copy.empty()) continue;

            pcl::VoxelGrid<PointType> vf;
            const float leaf = static_cast<float>(cloud_sparsify_res_);
            vf.setInputCloud(cloud_copy.makeShared());
            vf.setLeafSize(leaf, leaf, leaf);
            pcl::PointCloud<PointType> filtered;
            vf.filter(filtered);

            {
                std::lock_guard<std::mutex> lk(keyframes_mutex_);
                if (i < static_cast<int>(keyframes_.size()) && !keyframes_[i].cloud_sparsified_)
                {
                    keyframes_[i].pcd_              = std::move(filtered);
                    keyframes_[i].cloud_sparsified_ = true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// ── perfTimerFunc ─────────────────────────────────────────────────────────────

void FastLioSamScQn::perfTimerFunc(const ros::WallTimerEvent& /*event*/)
{
    const double uptime_s = (ros::WallTime::now() - perf_init_time_).toSec();

    double   kf_avg_ms = 0.0,   kf_max_ms = 0.0;  uint64_t kf_window  = 0;
    double   loop_avg_ms = 0.0, loop_max_ms = 0.0; uint64_t loop_window = 0;
    {
        std::lock_guard<std::mutex> lk(perf_mutex_);
        kf_window  = perf_kf_count_window_;
        kf_max_ms  = perf_kf_time_max_ms_;
        kf_avg_ms  = kf_window > 0 ? perf_kf_time_sum_ms_ / kf_window : 0.0;
        perf_kf_time_sum_ms_ = perf_kf_time_max_ms_ = 0.0; perf_kf_count_window_ = 0;

        loop_window  = perf_loop_count_window_;
        loop_max_ms  = perf_loop_time_max_ms_;
        loop_avg_ms  = loop_window > 0 ? perf_loop_time_sum_ms_ / loop_window : 0.0;
        perf_loop_time_sum_ms_ = perf_loop_time_max_ms_ = 0.0; perf_loop_count_window_ = 0;
    }

    const uint64_t frames    = perf_frames_total_.load(std::memory_order_relaxed);
    const uint64_t dropped   = perf_frames_dropped_.load(std::memory_order_relaxed);
    const uint64_t loops_acc = isam_backend_.loops_accepted.load(std::memory_order_relaxed);
    const uint64_t loops_rej = isam_backend_.loops_rejected.load(std::memory_order_relaxed);
    const uint64_t gps_acc   = perf_gps_accepted_.load(std::memory_order_relaxed);
    const size_t kf_total    = static_cast<size_t>(current_keyframe_idx_);

    size_t kf_pts_total = 0;
    {
        std::lock_guard<std::mutex> lk(keyframes_mutex_);
        for (const auto& kf : keyframes_) kf_pts_total += kf.pcd_.size();
    }
    const double kf_cloud_mb = kf_pts_total * sizeof(PointType) / 1.0e6;

    size_t rss_mb = 0;
    {
        std::ifstream proc("/proc/self/status");
        std::string line;
        while (std::getline(proc, line))
            if (line.compare(0, 6, "VmRSS:") == 0)
            {
                size_t kb = 0;
                std::sscanf(line.c_str(), "VmRSS: %zu kB", &kb);
                rss_mb = kb / 1024; break;
            }
    }

    ROS_INFO(
        "\033[1;95m[Perf +%.0fs]\033[0m"
        "  frames %lu (drop %lu / %.0f%%)"
        "  KF %zu (%.2f/s)  kf_cb avg %.1f max %.1f ms"
        "  loop %lu/%lu acc  loop_cb avg %.1f max %.1f ms"
        "  GPS %lu factors"
        "  cloud %.0f MB  RSS %zu MB",
        uptime_s,
        frames, dropped, frames > 0 ? 100.0 * dropped / frames : 0.0,
        kf_total, uptime_s > 0.0 ? static_cast<double>(kf_total) / uptime_s : 0.0,
        kf_avg_ms, kf_max_ms,
        loops_acc, loops_acc + loops_rej,
        loop_avg_ms, loop_max_ms,
        gps_acc, kf_cloud_mb, rss_mb);
}
