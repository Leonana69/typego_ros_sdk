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
