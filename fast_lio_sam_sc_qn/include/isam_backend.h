#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/linear/linearExceptions.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "utilities.hpp"

/// Configuration for the ISAM2 graph backend and loop closure quality gates.
struct BackendParams
{
    // LM batch refinement
    int    max_lm_factors    = 500;    ///< skip batch LM when committed graph exceeds this
    int    lm_max_iterations = 200;    ///< max LM iterations per solve
    double lm_rel_tol        = 1e-5;   ///< LM relative error change threshold to stop
    double lm_abs_tol        = 1e-5;   ///< LM absolute error change threshold to stop
    int    lm_max_passes     = 3;      ///< max adaptive LM passes on GPS re-entry
    double lm_max_distance   = 50.0;   ///< [m] dist gap that maps to lm_max_passes_

    // LM after loop closure
    int loop_lm_passes       = 1;      ///< LM passes after each accepted loop closure

    // Per-axis odom BetweenFactor variances — [roll², pitch², yaw²] rad² and [x², y², z²] m².
    // Inflate z and yaw to let GPS/heading override LIO drift on those axes.
    std::array<double,3> odom_noise_rot       = {1e-4, 1e-4, 1e-4};  ///< normal mode rotation
    std::array<double,3> odom_noise_pos       = {1e-2, 1e-2, 1e-2};  ///< normal mode position
    std::array<double,3> odom_noise_rot_degen = {1e-2, 1e-2, 1e-2};  ///< degenerate mode rotation
    std::array<double,3> odom_noise_pos_degen = {1e-1, 1e-1, 1e-1};  ///< degenerate mode position

    // Loop closure quality gates
    int    min_loop_kf_sep       = 50;    ///< minimum keyframe-index gap between loop endpoints
    double loop_noise_floor_rot  = 0.01;  ///< [rad²] minimum rotation variance for loop factor
    double loop_noise_rot_scale  = 1.0;   ///< scale: rot_var = floor * (1 + scale * yaw_diff_deg)
    double loop_noise_floor_pos  = 1.0;   ///< [m²] minimum position variance for loop factor
    double loop_max_yaw_diff_deg = 30.0;  ///< [deg] reject loop if ICP yaw disagrees with LIO by more than this
    double loop_max_pos_diff_m   = 5.0;   ///< [m]  reject loop if ICP translation disagrees with LIO by more than this
};

///
/// IsamBackend — owns all GTSAM graph state and ISAM2 incremental optimization:
///
///   - Factor staging (odom, GPS, heading, ground) from the odom-callback thread
///   - Pending loop accumulation from the loop-timer thread (mutex-protected)
///   - Atomic commit: merge staged + pending loops, run ISAM2 + optional LM
///   - Loop closure quality gating and deduplication
///   - LM batch refinement (GPS re-entry, loop closure, and on-demand service)
///
class IsamBackend
{
public:
    /// Result from commit(), propagated by the main class to corrected_esti_ / keyframes.
    struct UpdateResult
    {
        bool         ok        = false;
        bool         needs_vis = false;   ///< structural change → corrected poses were updated
        gtsam::Values estimate;           ///< from calculateEstimate(); empty on failure
        Eigen::MatrixXd marginal_cov;     ///< marginalCovariance(last_key)
    };

    explicit IsamBackend(const BackendParams& params);

    /// Read-only access to loaded parameters.
    const BackendParams& params() const { return p_; }
    /// Shorthand: minimum keyframe separation before a loop can be added.
    int minLoopKfSep() const { return p_.min_loop_kf_sep; }

    // ── Graph initialisation (first keyframe, no lock) ────────────────────────
    void initGraph(const gtsam::Pose3& init_pose,
                   gtsam::noiseModel::Diagonal::shared_ptr prior_noise,
                   int first_key);

    // ── Factor staging: odom-callback thread only, no lock needed ─────────────
    void stageOdomFactor(int prev_key, int curr_key,
                         const gtsam::Pose3& prev_pose, const gtsam::Pose3& curr_pose,
                         bool is_degenerate);
    void stageFactors(const gtsam::NonlinearFactorGraph& factors);
    void stageInitValue(int key, const gtsam::Pose3& pose);

    // ── ISAM2 commit (odom-callback thread, acquires lock internally) ─────────
    /// structural_change: true when GPS and/or heading factors were added this cycle
    ///   → triggers 5 extra ISAM2 updates and a corrected-pose refresh.
    /// gps_added: true when GPS triggers LM refinement this cycle.
    /// gps_lm_passes: adaptive pass count from GpsHandler::FactorResult::lm_dist_gap.
    UpdateResult commit(bool structural_change, bool gps_added, int gps_lm_passes);

    // ── Loop closure (loop-timer thread, acquires lock internally) ────────────
    bool isLoopDuplicate(int src_bucket, int dst_bucket) const;

    /// Validates yaw gate, computes noise, adds to pending_loops_ if accepted.
    /// Returns true if the loop was accepted.
    bool tryAddLoop(int src_idx, int dst_idx, int src_bucket, int dst_bucket,
                    const Eigen::Matrix4d& icp_pose_between_eig,
                    const Eigen::Matrix4d& lio_src_eig,
                    const Eigen::Matrix4d& lio_dst_eig,
                    double icp_score);

    // ── LM refinement (service callback, acquires lock) ───────────────────────
    bool runLMRefinement(int passes, const std::string& tag);
    gtsam::Values calculateEstimate();
    Eigen::MatrixXd getMarginalCovariance(size_t last_key);

    // ── Visualisation ─────────────────────────────────────────────────────────
    visualization_msgs::Marker getLoopMarkers(const gtsam::Values& esti,
                                              const std::string& map_frame) const;
    bool hasLoops() const;

    // ── Perf stats (atomic, readable from any thread) ─────────────────────────
    std::atomic<uint64_t> loops_accepted{0};
    std::atomic<uint64_t> loops_rejected{0};

private:
    BackendParams p_;

    std::shared_ptr<gtsam::ISAM2> isam_;

    // Staging (written only from odom-callback thread — no lock required)
    gtsam::NonlinearFactorGraph staged_graph_;
    gtsam::Values               staged_init_;

    // Pending loop factors (written from loop-timer thread — protected by graph_mutex_)
    gtsam::NonlinearFactorGraph pending_loops_;

    // Full committed factor history (for LM re-seed)
    gtsam::NonlinearFactorGraph committed_;

    mutable std::mutex graph_mutex_;

    // Loop deduplication and visualisation
    std::set<std::pair<int, int>>       existing_loops_;
    std::vector<std::pair<size_t, size_t>> loop_idx_pairs_;

    // ── Private helpers ───────────────────────────────────────────────────────
    static std::shared_ptr<gtsam::ISAM2> makeIsam2();

    /// Run force-relinearize + LM passes + ISAM2 rebuild.
    /// Caller MUST hold graph_mutex_.
    bool runLMRefinementLocked(int passes, const std::string& tag);
};
