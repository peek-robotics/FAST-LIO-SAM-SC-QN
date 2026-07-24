# Georef rework — Phase 3 follow-ups (revised after audit)

Follow-up to Phases 1–2 (branch `_LD_geodetic_georef`). **The original Phase 3 —
"drop navsat_transform" — was investigated and rejected (see Audit). What remains is
GPS/yaw noise retuning, which is empirical and must be done in the bag-validation loop.**

## Audit result: navsat_transform stays (it is shared robot infrastructure)

A workspace-wide audit of `navsat_transform` / `/toLL` / `/fromLL` / `/odometry/gps`
consumers:

| Consumer | Uses |
|---|---|
| `grover_spraybar/nodes/spraybar_controller.py` | `/toLL` |
| `grover_attachments/scripts/camrig_ig.py` | `/fromLL`, `/toLL`, `/odometry/gps` |
| `grover_navigation/src/GlobalMap.cpp` | `/fromLL` |
| `grover_navigation/.../action_processor.py`, `waypoint_store.py` | `/fromLL` (polygon→map) |
| `grover_navigation/nodes/gps_filter.py`, `odom_cal_client.py` | `/odometry/gps` |
| SLAM dev bringup (`grover_lidar_slam_dev.launch`) | `/odometry/gps` |

The live node is in **`grover_navigation/launch/robot_localization.launch`**
(`magnetic_declination_radians`, `yaw_offset`, `/imu/data`→`/gps/heading_imu`). The
voxel_slam `module_navsat.launch` is **dead** (included nowhere). Conclusion:
`navsat_transform` is core infra and **must keep running**. Phase 2 already decoupled the
SLAM *frame* from it (SLAM builds factors from the raw `NavSatFix`, not navsat's UTM), which
was the real goal. Do **not** remove navsat.

## Done in this pass (low-risk, no graph impact)
- `metadata.yaml` now stamps `frame_convention: local-ENU-true-north; datum=origin.init_gps;
  map_to_utm=geodetic-projection` (`saveMapPcd`), so downstream tools are unambiguous.
- `AGENTS.md` §4 updated with the Phase 1/2 status and the "navsat stays" finding.

## Remaining real work: GPS/yaw noise retuning (bag loop only)

The inflation knobs in `grover_lidar_bringup/config/fast_lio_sam_sc_qn/grover.yaml` were set
to survive the ~1.5° true-north-vs-grid fight (heading forced to win yaw; GPS softened while
it disagreed with the map). Phase 2 removed that fight, so these can likely relax — but this
is **calibration, not code**: apply ONE change, re-replay the same GCP bag, compare the
re-exported `cloud.laz` to GCPs and watch map sharpness (the doubled-trunk regression) and
`[Perf]`. Do **not** batch.

| Param | Current | Proposed start | Rationale |
|---|---|---|---|
| `backend/odom_noise_rot` (yaw term) | `1e-3` (×10) | `3e-4` (×3) | Yaw was inflated so heading overrode LIO; with GPS+heading now consistent, less is needed. |
| `gps/cov_scale` | `100.0` | `10.0` → `1.0` | Softened GPS ~10× (σ) to tolerate the convergence disagreement; step down and let RTK pull harder. |
| `gps/noise_floor` (per tier) | `1.0` | `0.1` (≈0.3 m σ) | Toward true RTK σ once positions are trustworthy. |
| `gps/heading_noise_floor` / `heading_factor_noise` | `0.01` / `0.05` | revisit last | Rebalance heading vs GPS once the above settle. |

Order: `cov_scale` → `noise_floor` → `odom_noise_rot` yaw → heading floors. Stop when maps
are sharp and GCP-accurate; over-tightening GPS re-introduces smearing from fix noise.

## Explicitly NOT doing (and why)
- **Removing navsat / `/toLL`** — shared infra (audit above). `saveMapPcd` keeps `/toLL` as a
  fallback; it already prefers the exact init datum (Phase 2).
- **Refactoring `gps_handler` to drive off `/gps/fix` instead of `/odometry/gps`** — since
  navsat stays, this is a marginal cleanup (SLAM would no longer need navsat's odom *stream*)
  for real init-path/interpolation refactor risk. Not worth it now; Phase 2 already sources
  the *position* from the raw fix. Reconsider only if SLAM must run without navsat.

## Gate still open
**Phase 2 is not yet bag-validated.** Validate it first — build `fast_lio_sam_sc_qn`, replay
a GCP bag, and confirm: the re-exported `cloud.laz` lands on GCPs *at distance* (not just near
origin); the saved cloud agrees with its own `/fast_lio_sam/gps_constraints` markers;
`[Init] GPS datum set (...)` logs; and `[GPS]` factor positions are small ENU values near the
origin (not raw 6-digit UTM eastings). Only then start the retuning above.

## Open questions
- Confirm `/gps/heading_imu` is a true-north dual-antenna azimuth and that
  `magnetic_declination_radians`/`yaw_offset` in `robot_localization.launch` aren't
  double-correcting it (a heading bias would masquerade as residual convergence).
- Decide the Z/altitude convention (init altitude is ellipsoidal; GIS may want orthometric —
  a ~constant geoid offset per block).
