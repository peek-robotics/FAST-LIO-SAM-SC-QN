# Phase 3 — drop navsat_transform + retune the anti-convergence hacks

Groundwork for the final step of the georeferencing rework. **Do not start until
Phase 2 is bag-validated** (below). This is a plan, not applied changes.

## Context

- **Phase 1 (done, merged on `grover_lidar_3d@_LD_geodetic_georef`):** `grover_slam_tools/georef.py` projects map→UTM geodetically (ENU→ECEF→UTM) instead of a pure translation. Fixes every *saved* map / export. Verified sub-mm vs geodesic truth; on a real 520 m block it moved points 5.9 m median / 14 m max.
- **Phase 2 (done, this branch, NEEDS BAG TEST):** `gps_handler` builds GPS position factors in a true-north local ENU frame (`geodeticToEnu` at the init datum) from the raw `NavSatFix`, instead of navsat's UTM `/odometry/gps`. Removes the true-north-heading vs grid-north-position fight *inside* the graph, so `cloud.pcd` itself is coherent. `saveMapPcd` now writes the exact init datum instead of `/toLL`.

After Phase 2, navsat_transform is still running but the SLAM node only uses it as
an odom *stream* source — its UTM framing is no longer trusted. Phase 3 removes
that dependency and relaxes the parameters that only existed to survive the fight.

## Precondition: validate Phase 2 first

Build (human) and replay a bag over a site with RTK GCPs. Confirm all three:
1. Re-exported `cloud.laz` lands on the GCPs at distance (not just near origin).
2. The saved cloud agrees with its own `/fast_lio_sam/gps_constraints` markers
   (no rotation of the map relative to the GPS factors it was built from).
3. `[Init] GPS datum set (...)` logs, and `[GPS] Factor @node` positions are
   small ENU values near the origin (not raw UTM 6-digit eastings).

If (1) fails but (2) holds, the export datum is off — check `metadata.yaml`
`init_gps`/`init_utm` vs the known datum. If (2) fails, the graph is still
fighting — re-check `nearestFixLL` pairing and that the heading is true-north.

## Phase 3 changes

### 1. Remove navsat_transform from the SLAM path
- **`gps_handler`**: make `/gps/fix` (NavSatFix) the primary GPS input; drive the
  factor cadence off it directly instead of `/odometry/gps`. `onGpsOdom` /
  `gps_queue_` can then be retired or fed from the fix. Keep the interpolation (#4)
  and time_offset (#6) logic — they now operate on the ENU position.
- **Launch**: drop `navsat_transform_node` (and `ekf_gps` if unused elsewhere)
  from `voxel_slam/launch/include/module_navsat.launch`. **Audit other consumers
  first** — `/odometry/gps`/`/odometry/navsat` and `/toLL` may feed row_follow_3d,
  BEV, or the localization EKF; keep navsat only if a non-SLAM consumer needs it.
- **`/gps/topic`** (`grover.yaml:119`) becomes unused for position.

### 2. Remove the `/toLL` fallback in `saveMapPcd`
Phase 2 already prefers the init datum. Once navsat is gone, `/toLL` never
resolves — delete the `to_ll_client_` branch and the `robot_localization` ToLL
include/dep, leaving the init-datum path as the only one.

### 3. Retune the anti-convergence noise inflation
These knobs (in `grover_lidar_bringup/config/fast_lio_sam_sc_qn/grover.yaml`) were
set to let the heading win the yaw argument and to soften GPS while it disagreed
with the map by ~1.5°. With the frame now consistent, re-evaluate — expect to be
able to relax them, improving conditioning:

| Param | Current | Why it was set / Phase-3 hypothesis |
|---|---|---|
| `backend/odom_noise_rot` | `[1e-4,1e-4,1e-3]` (yaw ×10) | Yaw inflated so heading overrides LIO. With GPS+heading consistent, less inflation needed. |
| `gps/cov_scale` | `100.0` | Softens GPS ~10× (σ). Was compensating for GPS positions being "wrong" by the convergence. Lower it and let RTK pull harder. |
| `gps/heading_noise_floor` / `heading_factor_noise` | `0.01` / `0.05` | Heading authority vs GPS. Rebalance now that they agree. |
| `gps/noise_floor` (per tier) | `1.0` | Can likely tighten toward true RTK σ (few cm). |

Change one at a time, re-replay, watch `[Perf]`, the constraint markers, and
map sharpness (doubled-trunk regression). Do **not** batch these.

### 4. Metadata / docs
- Stamp the map-frame convention in `metadata.yaml` (`frame: local-ENU, true-north,
  datum=init_gps`) so downstream tools are unambiguous.
- Update `AGENTS.md` §4 (georeferencing pipeline) and `config.yaml` comments to
  drop the navsat/`/odometry/filtered_map` framing.

## Validation & rollback
- Each step: build (human) + replay the same GCP bag; compare re-exported
  `cloud.laz` to GCPs and to the previous step. Keep the bag + GCP set fixed.
- Rollback is per-step (each is a separate commit). Phase 2 is independent of
  Phase 3, so Phase 3 can be reverted without touching the ENU factor change.

## Open questions to resolve before starting
- Where is grover's `navsat_transform_node` actually launched/configured? (Not in
  `grover_lidar_bringup`; find it before removing — it may be in a separate GPS
  bringup package.)
- Confirm `/gps/heading_imu` is a true-north dual-antenna azimuth with no double
  magnetic-declination applied (a heading bias would masquerade as residual
  convergence).
- Decide the Z/altitude convention (init altitude is ellipsoidal; GIS may want
  orthometric — a ~constant geoid offset per block).
