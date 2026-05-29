# PCD Grid Planner: Progress + Remaining Plan

Companion to [`pcd_fix.md`](pcd_fix.md). That doc holds the design rationale
and the priority-ordered list of fixes (P1–P8); this one tracks the PR rollout
and what still needs to ship.

## Status snapshot (branch `pcd-grid-planner`)

All four PRs from the plan are implemented and committed on this branch.
The behavior is unverified on a real saved map — the build is clean and the
EDT math has unit tests, but the end-to-end smoke verification listed under
each PR is still to do.

| PR  | Status        | Commit    |
|-----|---------------|-----------|
| PR 1 | ✅ implemented | `521f17b` |
| PR 2 | ✅ implemented | `ce1bd20` |
| PR 3 | ✅ implemented | `0d13111` |
| PR 4 | ✅ implemented | `cfefe3a` |
| PR 5 | ✅ implemented (uncommitted) | — |

## PR 5 — Map source: SLAM slice instead of terrain analysis

**Why:** the `/terrain_map_ext` output the grid was built from is noisy — its
per-point `intensity` is *height above a per-point estimated ground*, and that
estimator (a neighborhood plane fit expanded by `disRatioZ * dis`) smears and
invents obstacles/holes on uneven floors, reflective surfaces, and sparse
returns. The published 2D map inherited that noise.

**What changed:** a new `map_source` parameter selects the evidence source.

- `map_source: slice` (new default) — subscribe to `/registered_scan` (raw
  registered SLAM points, already in the map frame; geometry only, `intensity`
  ignored). Each cell keeps a **robust local floor** (`floor_z`, the running min
  of return z in that cell, lowered by at most `slice_floor_max_drop`, which is
  clamped `<= slice_obstacle_height` so a single sub-floor outlier can never
  manufacture a phantom obstacle). A return is **obstacle** evidence when it
  rises between `floor + slice_obstacle_height` and `floor + slice_obstacle_ceiling`
  (the robot fits under anything taller), **free** within `slice_obstacle_height`
  of the floor, ignored above the ceiling. Near-floor returns (`<=
  slice_ground_band`) define the published per-cell ground z. This is a per-cell
  *column* model: slope-robust (no moving-Z reference, unlike a vehicleZ-anchored
  slice) and free of the neighborhood plane-fit smearing.
- `map_source: terrain` — the legacy `/terrain_map_ext` path, unchanged.
- **Spatial cleanup** (the "smooth/filter to clean it" ask): after log-odds
  classification, a filtered *view* (`filtered_state_`) is produced and consumed
  by the SDF, the published `/pcd_2d_map`, the planner snapshot, and debug
  markers; the raw log-odds `grid_[*].state` is never overwritten, so the
  temporal and spatial filters don't fight. The filter is isolated-component
  removal (`spatial_min_component_size`, default 3: clears BLOCKED 8-connected
  clusters of ≤N cells **only when they touch no UNKNOWN neighbor**, i.e. sit
  inside known-free space — connected walls and frontier obstacles are never
  touched) plus optional morphological opening (`spatial_opening`, default off:
  anti-extensive, can thin 1-cell walls). Kill switch: `spatial_filter_enabled`.

The planning pipeline (multi-source A*, SDF clearance, snap, line-of-sight
smoothing, replan) is byte-for-byte unchanged; only the evidence source and the
spatial cleanup are new. Build clean; a 4-perspective design review and a
4-dimension adversarial diff review (slice/floor math, spatial filter,
concurrency, integration) returned no confirmed bugs.

**New parameters** (`config/default.yaml`): `map_source`, `slice_obstacle_height`
(0.15), `slice_obstacle_ceiling` (0.80), `slice_ground_band` (0.10),
`slice_floor_max_drop` (0.12), `spatial_filter_enabled` (true),
`spatial_min_component_size` (3), `spatial_opening` (false). `live_topic`
default is now `/registered_scan`; terrain mode auto-redirects it to
`/terrain_map_ext` with a warning if left at the slice default.

**Known v1 limitations (by design):** no negative-obstacle / drop-off detection
(a sharp floor drop beyond `slice_floor_max_drop` keeps the upper floor and
reads free); recurrent sub-floor noise can slowly lower a cell's floor (fails
toward phantom obstacles — the safe direction — and isolated ones are swept by
component removal); multi-level overlap (mezzanine over floor in one XY cell)
collapses to one floor. Acceptable for the single-level indoor target; revisit
if deployed on stairs/multi-level.

## PR 6 — Live-mapping fixes (frame, blind cone, dynamic clearing, grow-to-fit)

Field-debugging follow-ups on the slice mapper, in order found:

1. **Frame guard removed.** The early frame-id reject dropped every
   `/registered_scan` when the SLAM world frame wasn't literally `"map"`. The
   cloud is pre-registered world coordinates; we accept any frame and log it once.
2. **Blind-cone fill (`robot_free_carve`).** A low Mid-360 (downward FOV ~−7°)
   sees no ground within ~3 m, leaving the robot ringed by UNKNOWN. A BFS
   flood-fill from the robot through unobserved cells (bounded by
   `robot_free_radius`, stopping at observed walls/floor) marks the near-field
   FREE with a borrowed ground z. Also raised `live_l_free_thr` to −0.4 so
   observed floor turns FREE in one tick.
3. **Ray-trace clearing (`raytrace_clearing`).** Dynamic obstacles used to stick
   forever (a cell only cleared if a floor return landed in it). Now, per scan,
   beams are traced (Amanatides-Woo) from the robot to each **floor** return and
   the swept cells get a FREE/miss vote — so a person who walks away is cleared
   (~2.5 s) once the floor behind them reappears. Clearing along *floor* beams
   only means low static obstacles (which occlude the floor behind them) are not
   wrongly cleared; obstacle endpoints are never cleared, so a standing obstacle
   stays blocked.
4. **Grow-to-fit grid (`live_grid_growth`).** Replaced the fixed 80×80 m extent
   with a 24×24 m initial grid that expands to cover the explored area (returns
   within `live_grid_grow_max_range_m` + margin), bounded by `grid_max_cells`.
   Made thread-safe without a snapshot-dims refactor by gating the realloc on
   `!executing_`: the single-threaded executor + worker-only-while-executing
   means growth never races the planner reading the grid dims.

Two design reviews + a 4-/3-dimension adversarial diff review backed each step;
the last review caught and fixed 3 real bugs (floor-target registration vs
coincident obstacle returns, interval-sampling DDA skipping cells, missing
`carve_visit_gen_` init). Build clean; unverified end-to-end on hardware.

## PR 7 — Slice model: robot-relative lidar-plane band

Replaced the per-cell column model with a thin **horizontal band at the lidar
plane**, by operator choice (simpler, and it drops the per-cell floor estimation
that caused most PR-6 bugs). Per return: `rel = z − (robot_z + slice_band_offset)`;
`|rel| ≤ slice_band_half` (default 0.10) → obstacle, `rel < −half` → floor/free
(sets cell ground z), `rel > half` → overhead (ignored). The band is centered on
the robot's current z, so it tracks the lidar plane and is immune to SLAM
z-drift and floor-level changes — no absolute-z reference, no running-min floor.
Free space still comes from below-band floor returns + ray-trace clearing + the
blind-cone carve; grow-to-fit unchanged. Removed `slice_obstacle_height/
ceiling/ground_band/floor_max_drop` and the `LiveAccum` floor fields; added
`slice_band_half`, `slice_band_offset`. Also set `grid_resolution` default to
0.05 m. **Limitation:** a thin slice at the lidar plane misses obstacles shorter
than the lidar; if the lidar sits <`slice_band_half` above the floor, raise
`slice_band_offset` so the band clears the floor.

**Verification before merge:** smoke-test on a real `/registered_scan` (bag or
robot): confirm `/pcd_2d_map` shows clean walls + free near-field floor, send a
goal and watch `/pcd_grid_path` reach it, compare map cleanliness against
`map_source: terrain` on the same path, and check per-tick timing on the Jetson
Orin at the configured extent (full-grid EDT + filter run each tick when cells
change).

Outstanding work, in order:
1. **Smoke verification** on `Map-<name>.pcd` for each verification case
   listed below (cold start, slope, support sparsity, skim, fused
   clearance, action race, RViz load).
2. **Calibration of `grid_min_support_points`** against the actual saved
   maps. Today it auto-derives from `(grid_resolution/map_voxel_size)^2 *
   grid_min_support_fill` (default fill=0.20). Worth tuning per profile
   if outdoor maps come out too sparse.

## Completed: PR 1 — Mechanical refactor (no behavior change)

Covers plan items **P1** (delete dead PRM code, rename package) and the **P7**
items that fall out naturally (revert unrelated diff, drop the
`roadmap_update_mode` plumbing, gate the planner on a non-empty map path).

### What changed

- **Reverted unrelated diff.** `src/autonomy/route_planner/far_planner/config/outdoor.yaml`
  `is_debug_output` back to `true`. Not part of this work.
- **Package renamed.** `pcd_roadmap_planner → pcd_grid_planner` everywhere:
  - directory, `package.xml` name + description, `CMakeLists.txt` project +
    executable + install paths
  - `src/pcd_roadmap_planner.cpp → src/pcd_grid_planner.cpp`
  - `launch/pcd_roadmap_planner.launch.py → launch/pcd_grid_planner.launch.py`
  - C++ class `PcdRoadmapPlanner → PcdGridPlanner`
- **Dead PRM code deleted** (~1300 lines). Removed: `nodes_`, `graph_`,
  `components_`, `component_sizes_`, `node_cloud_`, `node_tree_`,
  `map_tree_`, `live_tree_`, `live_cloud_`, `live_mutex_`, structs
  `RoadmapEdge / RoadmapNode / SnapCandidate / SnapAttempt`, methods
  `estimateTraversableSurface`, `updateNodeCloud`, `buildEdges`,
  `bridgeNearbyComponents`, `computeConnectedComponents`, `snapCandidates`,
  `isSnapConnectorTraversable`, `hasSharedComponent`, `componentId`,
  `componentSize`, `snapCandidateSummary`, `nearestNode`, `searchGraph`,
  `isGeometricEdgeAllowed`, `isStaticEdgeTraversable`, `hasSupportNear`,
  `hasStaticObstacleNear`, `isLiveEdgeBlocked`, `terrainCallback`, the
  unreachable second half of `planRoute`, the PRM branch of
  `publishRoadmapMarkers`, the inert `isRouteBlocked` short-circuit, plus
  the live-overlay subscription.
- **Eigen dependency dropped** from `package.xml` and `CMakeLists.txt`
  (PCA only lived in `estimateTraversableSurface`).
- **Topics renamed**: `/pcd_roadmap_path → /pcd_grid_path`, `/pcd_roadmap →
  /pcd_grid_markers`, `pcd_roadmap_route → /pcd_grid_route`. `/pcd_2d_map`
  unchanged (already the right name). RViz config updated to match.
- **Default config trimmed.** `default.yaml` no longer declares
  `grid_min_support_points` (wired but unused at this stage),
  `node_spacing`, `z_bin_size`, `connect_radius`, `component_bridge_radius`,
  `max_ground_roughness`, `min_support_points`, `support_radius`,
  `edge_check_resolution`, `edge_support_check`,
  `fallback_start_snap_radius`, `fallback_goal_snap_radius`,
  `live_block_radius`, `live_block_height`, `replan_period`,
  `max_replan_attempts`. The smaller `start_snap_radius` parameter now
  carries what `fallback_start_snap_radius` used to (renamed; default 6.0 m).
- **Live-overlay knob gone.** `roadmap_update_mode` was inert in the grid
  pipeline; dropped from `default.yaml`, planner code, both bringup launch
  files, `RoadmapUpdateMode` literal in `schema.py`, `robot.yaml`, and the
  console / RViz scripts. P4 will introduce a new knob (`live_layer_enabled`
  or similar) with the live layer it actually controls.
- **Backend literal renamed.** `RoutePlannerBackend = Literal['far',
  'pcd_roadmap']` → `Literal['far', 'pcd_grid']`. Updated in `schema.py`,
  `robot.yaml`, both bringup launch files, `scripts/rviz_launch.py`,
  `scripts/sdk_console.py`, `src/autonomy/README.md`.
- **Empty-map guard.** `system_real_robot_with_route_planner.launch.py` now
  gates the `pcd_grid_planner` include on `map_dir != ''`. Previously an
  empty `slam_map_name` flowed through and the node aborted at startup.

### Build status

`colcon build --packages-select pcd_grid_planner typego_config
vehicle_simulator typego_sdk` succeeds. The one stderr line is a CMake
`CMP0074` policy warning from `find_package(PCL)`, unrelated. Pydantic
loader round-trips the updated `robot.yaml` (`route_planner_backend=far`,
no `roadmap_update_mode` field).

### Not verified yet

- End-to-end smoke test: load a saved PCD, send a goal in RViz, observe
  `/pcd_grid_path` reach the goal and the robot follow `/way_point`.
  Mechanical-only PR — behavior should be identical to pre-rename, but
  worth a 5-minute run before treating PR 1 as done.

## PRs

### PR 2 — Correctness (plan items P2 + P3)

**Theme:** make the static grid actually mean what the planner pretends it
means. Today a 0.5 m wall between cells is a 12 cm cost nudge instead of
"no", unobserved cells masquerade as free, and clearance/inflation/snapping
are three separate brittle thresholds.

**Work items**

1. **Three-state cell.** Replace `raw_free / traversable / inflated`
   booleans with an explicit `enum {UNKNOWN, FREE, BLOCKED}` for the
   *static* layer. Keep "this robot can stand here" as a derived
   per-planner view, not part of the published map. Encode the published
   `/pcd_2d_map` with `0 / 100 / -1` per the OccupancyGrid spec.
2. **Slope + step enforcement.** In `gridMoveAllowed`, reject the
   transition if `|z_a - z_b| > max_step_height` OR `|z_a - z_b| > step_xy
   * tan(max_slope_rad)`. Both `z`s are already cached in `GridCell::z`.
   The existing `0.25 * |dz|` cost nudge in `searchGridToCandidates` stays
   as a tiebreaker but is no longer load-bearing.
3. **Support gate.** A cell is `FREE` only if `low_count >=
   grid_min_support_points` (which becomes a read parameter again).
   Unobserved / no-low-support cells → `UNKNOWN`. Mark cells whose
   4-neighborhood touches the grid bounding box as `UNKNOWN` to give
   negative-obstacle protection at the map edge.
4. **Support neighborhood, calibrated.** Do not pick a single global
   default for `grid_min_support_points`. Either derive from
   `(grid_resolution / map_voxel_size)^2 * fill_fraction`, or expose
   per-profile knobs. Before merging this PR, run a small calibration
   experiment: report free/unknown/blocked counts across the saved maps
   under `src/typego_sdk/resource/Map-*` for the candidate
   (support_points × support_neighborhood) settings. PR 2 ships with the
   chosen default; the experiment artifact lives next to this doc or in
   the PR description.
5. **Signed distance field.** After classifying cells, compute `sdf_[N]`
   = Euclidean distance (in meters) to the nearest `BLOCKED`-or-`UNKNOWN`
   cell using a 2-pass Felzenszwalb–Huttenlocher EDT. Replace:
   - Inflation pass → `sdf >= clearance_radius` predicate.
   - `gridSnapCandidates` heuristic → expanding-ring search that accepts
     the first cell with `sdf >= clearance_radius` (start cell may use a
     smaller margin so the robot can recover if it's already wedged).
   - `gridLineTraversableWorld` → sample along the line, require `min_sdf
     >= margin`.
   - Optional A* cost shaping: `step_cost = step_xy * (1 + k *
     max(0, w - sdf) / w)` with `w ≈ 1 m`. Robots prefer corridor
     centerlines.
6. **Static SDF only (for now).** P4 will introduce a second "fused" SDF
   built from `static_state OR live_state`. Keep the static SDF
   conceptually separate from the fused one so the published
   reusable-map contract stays robot-agnostic.

**Calibration step (gating):** numeric report of free/unknown/blocked
counts on real saved maps before settling on the default.

**Verification before merge**
- Static replay still finds a path on `Map-<name>.pcd`.
- Slope synthetic test: PCD with a step ≥ `max_step_height` between two
  flat regions — path must route around it.
- Support synthetic test: sparse PCD with an unobserved hole — hole cells
  publish `-1` and the planner refuses them.
- **Skim test:** goal placed so the straight-line path runs along an
  inflation boundary. Line-of-sight check must refuse to smooth through
  cells whose SDF margin is below threshold. This test gates PR 4's
  smoothing pass.

### PR 3 — Capability (plan item P4)

**Theme:** make the 2D map a live, reusable artifact. Today
`/pcd_2d_map` is published once at startup. Replanning around new
obstacles is disabled.

**Work items**

1. **Subscribe to `/terrain_map`** unconditionally on startup. (The
   `live_overlay` mode knob is gone; a kill-switch param can be added if
   needed.) Rasterize incoming points with `intensity > obstacle_height`
   into a `live_grid_` aligned to `grid_`.
2. **Per-cell timestamps + state**: `LIVE_FRESH_BLOCKED` (just stamped)
   transitions to `LIVE_STALE` after `live_timeout_sec / 2`, then clears
   after `live_timeout_sec`. The planner treats `FRESH_BLOCKED` as a
   hard block and `STALE` as a soft cost penalty. This dampens the
   route-flap that pure binary decay causes.
3. **Liveness watchdog.** If no `/terrain_map` message arrives for
   `live_input_timeout_sec`, drop the live layer to empty. Perception
   may have died; better to fall back to the static prior than strand the
   robot on phantom obstacles.
4. **Publish two topics:**
   - `/pcd_2d_map` — static (prior) map, unchanged contract.
   - `/pcd_2d_map_live` — fused (static OR live) at ~5 Hz.
5. **Fused SDF.** Build a separate SDF from the fused blocked/unknown
   layer for the route search. Static SDF stays available for the
   published map. Incremental rebuild over the bounding box of changed
   cells (PR 4 / P8 optimization) — for PR 3, full 2-pass EDT is fine.
6. **Re-enable `isRouteBlocked`.** Walk the route from `route_index`
   forward over a `replan_horizon_m` window, sample every
   `grid_resolution / 2`, return `true` if any sample falls in a
   `FRESH_BLOCKED` or now-`UNKNOWN` cell. Drop the `if (!grid_.empty())
   return false;` short-circuit.
7. **Replan loop.** The execution loop's `live_overlay` branch (already
   present in `executeGoal` and `executeTopicGoal`) now actually runs
   `planGridRoute` again on the fused grid when `isRouteBlocked` fires.
   Restore `replan_period` and `max_replan_attempts` parameters.

**Verification before merge**
- **Cold start, no live input:** bring up the planner and send a goal
  *before any `/terrain_map` arrives*. First plan must succeed on the
  static layer alone.
- **Live input dropout:** kill the `/terrain_map` publisher mid-run; after
  `live_input_timeout_sec`, the live layer drops to empty and the robot
  is not stranded on stale phantom obstacles.
- **Replan trigger:** bag-replay `/terrain_map` with a virtual obstacle
  dropped on the planned route — replan happens within `replan_period`.
- **Fused clearance:** inject a live obstacle near (not on) the path —
  replanning respects clearance from the fused SDF, not just the
  obstacle's center cell.

### PR 4 — Quality (plan items P5 + P6 + P8)

**Theme:** sand the corners. Smoother paths, fewer threads, less memory.

**Work items (in dependency order)**

1. **Theta*-style smoothing (P5).** Greedy line-of-sight compaction
   post-A*: walk the route, advance `j` while
   `gridLineTraversableWorld(route[i], route[j+1])` holds, otherwise emit
   `route[j]`. Uses the fused SDF margin from PR 2 + PR 3. If diagonal
   artifacts remain, fall back to Lazy Theta* in search. **Prerequisite:**
   PR 2's skim test must pass before this lands.
2. **Single navigation worker (P6).** Extract `runNavigation(goal_point,
   on_cancel, on_feedback)` helper. Both `executeGoal` and
   `executeTopicGoal` collapse to a `NavigationRequest` struct + a call.
3. **Goal-admission race fix (P6).** `handleGoal()` currently checks
   `executing_.load()`, but `executing_` is set later in
   `handleAccepted()`. Switch to `executing_.exchange(true)` in the
   action-acceptance path (or a mutex over a request state) so two
   `NavigateToPose` goals cannot both pass admission.
4. **Joinable worker thread (P6).** Replace the two detached
   `std::thread`s with one joinable `std::thread` member joined in the
   destructor. Skip `std::jthread` — it forces a C++14 → C++20 bump in
   `CMakeLists.txt` for no real win.
5. **RAII `executing_` guard (P6).** Wrap `executing_.store(false)` in a
   small guard so every return path clears it. Today the cleanup is
   sprinkled across ~10 sites.
6. **Build-time / runtime grid split (P8).** Use temporary accumulators
   for `count`, `low_count`, `obstacle_count`, `min_z`, `max_z`,
   `low_sum_z` during build; after finalization, keep only compact
   runtime state (`state`, `z`, static SDF, live timestamp/state flags).
   Matters at the 2 M-cell cap.
7. **Incremental fused SDF (P8).** Recompute SDF only over the bounding
   box of cells whose live state changed since the last tick. Optional;
   land it if profiling shows the EDT is a hot path on outdoor maps.
8. **Debug-marker gate (P8).** `publishGridMarkers` currently emits one
   CUBE per traversable cell. Default `publish_debug_markers: false`,
   let RViz render `/pcd_2d_map` natively.

**Verification before merge**
- **Skim regression:** PR 2's skim test still passes — smoothing doesn't
  cut through inflation boundaries.
- **Action race:** send two nearly-simultaneous `NavigateToPose` goals;
  exactly one is accepted/executed. Cancel clears `executing_` exactly
  once.
- **Cancel + shutdown:** cancel a goal mid-run; ROS shutdown during a
  goal — no thread outlives the node.
- **RViz load time:** large saved map → RViz comes up under a couple of
  seconds (debug markers off by default).

## Cross-cutting reminders

- The `/pcd_2d_map` contract is reusable-map describing the world, not a
  robot-specific plan layer. `clearance_radius` and other vehicle knobs
  must not appear in its data. Keep robot-specific derived views on
  separate topics.
- After PR 4, several formerly-PRM parameters (`replan_period`,
  `max_replan_attempts`) come back online. Re-add them to `default.yaml`
  and the launch surface when PR 3 ships, not earlier.
- The plan in `pcd_fix.md` is the source of truth for *why*. This doc is
  the source of truth for *where we are*. Update both when scope shifts.
