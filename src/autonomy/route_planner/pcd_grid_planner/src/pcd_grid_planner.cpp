#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace
{
double pointDistXY(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

geometry_msgs::msg::Point lerpPoint(
    const geometry_msgs::msg::Point& a,
    const geometry_msgs::msg::Point& b,
    double t)
{
  geometry_msgs::msg::Point out;
  out.x = a.x + (b.x - a.x) * t;
  out.y = a.y + (b.y - a.y) * t;
  out.z = a.z + (b.z - a.z) * t;
  return out;
}
}  // namespace

// Live-mapping 2D grid planner. The grid is built at runtime into a per-cell
// log-odds occupancy filter. Two evidence sources are supported (map_source):
//
//   slice   (default): raw SLAM points from /registered_scan (already in the
//           map frame). A thin horizontal band at the lidar plane (robot z +
//           slice_band_offset, half-thickness slice_band_half): returns within
//           the band are obstacles, returns below it are floor/free (and set
//           the cell ground z), returns above it are overhead and ignored. The
//           band follows the robot's z, so it is immune to SLAM z-drift and
//           floor-level changes, and needs no per-cell floor estimation.
//   terrain (legacy): terrain_analysis output (/terrain_map_ext), intensity =
//           height above estimated ground.
//
// After classification a spatial cleanup filter (isolated-component removal,
// optional morphological opening) produces a filtered view used for the SDF and
// the published map; the raw log-odds state is never overwritten by it. The
// planning pipeline (SDF, A*, snap, line-of-sight smoothing, replan) is
// unchanged; only the source and the spatial cleanup differ.
class PcdGridPlanner : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  PcdGridPlanner()
      : Node("pcd_grid_planner")
  {
    readParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 10,
        std::bind(&PcdGridPlanner::odomCallback, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&PcdGridPlanner::goalPoseCallback, this, std::placeholders::_1));
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        live_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdGridPlanner::cloudCallback, this, std::placeholders::_1));

    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/pcd_grid_path", rclcpp::QoS(1).transient_local());
    grid_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/pcd_2d_map", rclcpp::QoS(1).transient_local());
    grid_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pcd_grid_markers", rclcpp::QoS(1).transient_local());
    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pcd_grid_route", rclcpp::QoS(1).transient_local());

    // Allocate immediately unless we're waiting for the first odom to center
    // the grid on the robot.
    if (!live_map_recenter_on_first_odom_)
    {
      grid_allocated_.store(true);
      allocateLiveGrid(live_map_center_x_, live_map_center_y_);
    }

    const auto mapping_period = std::chrono::milliseconds(
        static_cast<int>(1000.0 / mapping_rate_hz_));
    mapping_timer_ = create_wall_timer(
        mapping_period, std::bind(&PcdGridPlanner::onMappingTick, this));

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&PcdGridPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PcdGridPlanner::handleCancel, this, std::placeholders::_1),
        std::bind(&PcdGridPlanner::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(
        get_logger(),
        "PCD grid planner ready (map_source=%s, input=%s); action server on "
        "/navigate_to_pose, RViz goals on /goal_pose.",
        map_source_ == MapSource::SLICE ? "slice" : "terrain",
        live_topic_.c_str());
  }

  ~PcdGridPlanner() override
  {
    // Signal the worker to stop, then join. Relying on rclcpp::ok() alone
    // would block forever if the node is destroyed while the ROS context
    // is still valid; stop_requested_ breaks the navigation loop directly.
    stop_requested_.store(true);
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

private:
  // Clears `executing_` on destruction. Replaces a dozen scattered
  // executing_.store(false) calls; every return path from runNavigation
  // funnels through it.
  struct ExecutingGuard
  {
    std::atomic_bool& flag;
    explicit ExecutingGuard(std::atomic_bool& f) : flag(f) {}
    ~ExecutingGuard() { flag.store(false); }
    ExecutingGuard(const ExecutingGuard&) = delete;
    ExecutingGuard& operator=(const ExecutingGuard&) = delete;
  };

  // Spawns the navigation worker. Joins any previous worker first so we
  // never have two workers, and the std::thread member stays joinable
  // for the destructor.
  void startWorker(std::function<void()> task)
  {
    std::lock_guard<std::mutex> lock(worker_mutex_);
    if (worker_.joinable())
    {
      worker_.join();
    }
    worker_ = std::thread(std::move(task));
  }

  enum class CellState : std::uint8_t
  {
    UNKNOWN = 0,
    FREE = 1,
    BLOCKED = 2,
  };

  // Evidence source for the live grid. SLICE consumes raw /registered_scan with
  // a per-cell column model; TERRAIN consumes terrain_analysis /terrain_map_ext.
  enum class MapSource : std::uint8_t
  {
    SLICE = 0,
    TERRAIN = 1,
  };

  // What a single slice return contributed to its cell.
  enum class SliceEvidence : std::uint8_t
  {
    IGNORE = 0,    // overhead / outlier — no vote
    FREE = 1,      // floor return — free vote (and a ray-trace clearing target)
    OBSTACLE = 2,  // above floor — obstacle vote
  };

  // Runtime cell — the current best map, owned by the executor thread
  // (mapping tick + publishers). The navigation worker never reads this
  // directly; it reads an immutable FusedSnapshot copy instead.
  struct GridCell
  {
    double z = 0.0;
    double sdf = 0.0;      // meters to nearest non-FREE cell
    CellState state = CellState::UNKNOWN;
  };

  // Per-cell log-odds accumulator. obs_count/free_count are raw evidence
  // tallied by cloudCallback and collapsed to one log-odds vote per mapping
  // tick (decoupling the filter rate from the sensor rate). logodds +
  // ground_sum/ground_count persist across ticks; the counts reset.
  struct LiveAccum
  {
    float logodds = 0.0f;
    std::uint16_t obs_count = 0;
    std::uint16_t free_count = 0;
    double ground_sum = 0.0;     // sum of floor-return z (below-band); defines cell ground z
    std::uint32_t ground_count = 0;
  };

  struct GridCandidate
  {
    int index = -1;
    double distance = 0.0;
  };

  // Immutable snapshot of the classified grid (state + ground z + SDF) that
  // the planner reads for the duration of a single planning operation. The
  // mapping tick swaps a fresh shared_ptr under fused_mutex_; readers retain
  // a shared_ptr and read lock-free, so the whole planning input is atomic.
  struct FusedSnapshot
  {
    std::vector<CellState> state;
    std::vector<double> z;
    std::vector<double> sdf;
  };

  void readParameters()
  {
    declare_parameter<std::string>("map_frame", "map");

    declare_parameter<double>("grid_resolution", 0.05);
    declare_parameter<int>("grid_max_cells", 2000000);

    // Live grid extent: initial size (recentered on first odom), grown to fit
    // the explored area as the robot moves (grow-to-fit, slam_toolbox-style).
    declare_parameter<double>("live_map_width_m", 24.0);
    declare_parameter<double>("live_map_height_m", 24.0);
    declare_parameter<double>("live_map_center_x", 0.0);
    declare_parameter<double>("live_map_center_y", 0.0);
    declare_parameter<bool>("live_map_recenter_on_first_odom", true);
    declare_parameter<bool>("live_grid_growth", true);
    declare_parameter<double>("live_grid_growth_margin_m", 3.0);   // expand this far past new returns
    declare_parameter<double>("live_grid_grow_max_range_m", 20.0); // ignore returns farther than this for growth

    declare_parameter<double>("max_slope_deg", 25.0);
    declare_parameter<double>("max_step_height", 0.18);

    declare_parameter<double>("min_clearance", 0.25);
    declare_parameter<double>("obstacle_height", 0.15);
    declare_parameter<double>("vehicle_height", 1.0);   // reserved; unused in v1
    declare_parameter<double>("vehicle_length", 0.7);
    declare_parameter<double>("vehicle_width", 0.3);
    // Min SDF along a connector line (start/goal snap + edge check). A bit
    // less than clearance_radius so the planner can recover when the robot
    // is already wedged near an obstacle.
    declare_parameter<double>("line_clearance", -1.0);

    declare_parameter<double>("start_snap_radius", 6.0);
    declare_parameter<double>("goal_snap_radius", 2.0);
    declare_parameter<double>("lookahead_distance", 2.5);
    declare_parameter<double>("waypoint_reach_radius", 0.6);
    declare_parameter<double>("goal_tolerance", 0.35);
    declare_parameter<double>("waypoint_rate", 5.0);
    declare_parameter<bool>("smoothing_enabled", true);
    declare_parameter<bool>("publish_debug_markers", false);

    // Live occupancy mapping.
    declare_parameter<std::string>("map_source", "slice");   // "slice" | "terrain"
    declare_parameter<std::string>("live_topic", "/registered_scan");
    declare_parameter<double>("mapping_rate_hz", 2.0);
    declare_parameter<double>("blockage_ceiling", 1.6);

    // SLICE mode: thin horizontal band at the lidar plane (robot-relative).
    // A return is an obstacle within slice_band_half of the band center
    // (robot z + slice_band_offset); below the band it's floor/free; above it
    // is overhead and ignored. The band follows the robot, so it's immune to
    // SLAM z-drift and works across floor levels.
    declare_parameter<double>("slice_band_half", 0.10);         // half-thickness of the obstacle band
    declare_parameter<double>("slice_band_offset", 0.0);        // band center = robot_z + this (lidar plane)

    // Spatial cleanup filter (applied to a filtered view; never overwrites the
    // raw log-odds state). Removes isolated obstacle specks; opening optional.
    declare_parameter<bool>("spatial_filter_enabled", true);
    declare_parameter<int>("spatial_min_component_size", 3);    // BLOCKED component <= this → cleared
    declare_parameter<bool>("spatial_opening", false);          // erode→dilate (can thin walls; off)

    // Robot free-space carving: the robot stands on traversable ground, so a
    // small disc around its pose is FREE by definition. Fills the lidar's
    // near-field blind cone and seeds free space at startup before the log-odds
    // warmup. Carved cells borrow ground z from observed neighbors.
    declare_parameter<bool>("robot_free_carve", true);
    declare_parameter<double>("robot_free_radius", 5.0);        // blind-cone flood cap (stops at observations)

    // Ray-trace clearing (SLICE mode): clears cells a beam sweeps to a floor
    // return, so dynamic obstacles decay once the floor behind them reappears.
    declare_parameter<bool>("raytrace_clearing", true);
    declare_parameter<double>("raytrace_min_range_m", 0.3);
    declare_parameter<double>("raytrace_max_range_m", 15.0);

    // Clearance/SDF source: false (default) = keep clearance from BLOCKED only,
    // so the unobserved frontier of a live map doesn't block paths through
    // observed-free space; true = also inflate from UNKNOWN (conservative, for
    // a complete saved map).
    declare_parameter<bool>("clearance_inflate_unknown", false);

    // Slope/step gate between cells. Off by default: needs a reliable per-cell
    // elevation, which the slice band z is not, and its per-step threshold is
    // sub-noise at fine resolution. Enable only for terrain-mode elevation.
    declare_parameter<bool>("enforce_slope_check", false);
    declare_parameter<double>("live_l_hit", 0.85);
    declare_parameter<double>("live_l_miss", 0.40);
    declare_parameter<double>("live_l_min", -2.0);
    declare_parameter<double>("live_l_max", 4.0);
    declare_parameter<double>("live_l_occ_thr", 1.5);
    declare_parameter<double>("live_l_free_thr", -0.4);

    // Replanning.
    declare_parameter<double>("replan_period_sec", 0.5);
    declare_parameter<double>("replan_horizon_m", 4.0);
    declare_parameter<int>("max_replan_attempts", 20);

    get_parameter("map_frame", map_frame_);

    get_parameter("grid_resolution", grid_resolution_);
    get_parameter("grid_max_cells", grid_max_cells_);

    get_parameter("live_map_width_m", live_map_width_m_);
    get_parameter("live_map_height_m", live_map_height_m_);
    get_parameter("live_map_center_x", live_map_center_x_);
    get_parameter("live_map_center_y", live_map_center_y_);
    get_parameter("live_map_recenter_on_first_odom", live_map_recenter_on_first_odom_);
    get_parameter("live_grid_growth", live_grid_growth_);
    get_parameter("live_grid_growth_margin_m", live_grid_growth_margin_);
    get_parameter("live_grid_grow_max_range_m", live_grid_grow_max_range_);

    get_parameter("max_slope_deg", max_slope_deg_);
    get_parameter("max_step_height", max_step_height_);

    get_parameter("min_clearance", min_clearance_);
    get_parameter("obstacle_height", obstacle_height_);
    get_parameter("vehicle_height", vehicle_height_);
    get_parameter("vehicle_length", vehicle_length_);
    get_parameter("vehicle_width", vehicle_width_);
    get_parameter("line_clearance", line_clearance_);

    get_parameter("start_snap_radius", start_snap_radius_);
    get_parameter("goal_snap_radius", goal_snap_radius_);
    get_parameter("lookahead_distance", lookahead_distance_);
    get_parameter("waypoint_reach_radius", waypoint_reach_radius_);
    get_parameter("goal_tolerance", goal_tolerance_);
    get_parameter("waypoint_rate", waypoint_rate_);
    get_parameter("smoothing_enabled", smoothing_enabled_);
    get_parameter("publish_debug_markers", publish_debug_markers_);

    std::string map_source_str;
    get_parameter("map_source", map_source_str);
    map_source_ = (map_source_str == "terrain") ? MapSource::TERRAIN : MapSource::SLICE;

    get_parameter("live_topic", live_topic_);
    get_parameter("mapping_rate_hz", mapping_rate_hz_);
    get_parameter("blockage_ceiling", blockage_ceiling_);

    get_parameter("slice_band_half", slice_band_half_);
    get_parameter("slice_band_offset", slice_band_offset_);

    get_parameter("spatial_filter_enabled", spatial_filter_enabled_);
    get_parameter("spatial_min_component_size", spatial_min_component_size_);
    get_parameter("spatial_opening", spatial_opening_);

    get_parameter("robot_free_carve", robot_free_carve_);
    get_parameter("robot_free_radius", robot_free_radius_);
    get_parameter("raytrace_clearing", raytrace_clearing_);
    get_parameter("raytrace_min_range_m", raytrace_min_range_);
    get_parameter("raytrace_max_range_m", raytrace_max_range_);
    get_parameter("clearance_inflate_unknown", clearance_inflate_unknown_);
    get_parameter("enforce_slope_check", enforce_slope_check_);
    get_parameter("live_l_hit", live_l_hit_);
    get_parameter("live_l_miss", live_l_miss_);
    get_parameter("live_l_min", live_l_min_);
    get_parameter("live_l_max", live_l_max_);
    get_parameter("live_l_occ_thr", live_l_occ_thr_);
    get_parameter("live_l_free_thr", live_l_free_thr_);

    get_parameter("replan_period_sec", replan_period_sec_);
    get_parameter("replan_horizon_m", replan_horizon_m_);
    get_parameter("max_replan_attempts", max_replan_attempts_);

    grid_resolution_ = std::max(0.05, grid_resolution_);
    grid_max_cells_ = std::max(1000, grid_max_cells_);
    mapping_rate_hz_ = std::max(0.1, mapping_rate_hz_);
    waypoint_rate_ = std::max(0.5, waypoint_rate_);
    replan_period_sec_ = std::max(0.05, replan_period_sec_);
    replan_horizon_m_ = std::max(grid_resolution_, replan_horizon_m_);
    max_replan_attempts_ = std::max(0, max_replan_attempts_);

    slice_band_half_ = std::max(0.0, slice_band_half_);
    robot_free_radius_ = std::max(0.0, robot_free_radius_);
    live_grid_growth_margin_ = std::max(0.0, live_grid_growth_margin_);
    live_grid_grow_max_range_ = std::max(grid_resolution_, live_grid_grow_max_range_);
    raytrace_min_range_ = std::max(0.0, raytrace_min_range_);
    raytrace_max_range_ = std::max(raytrace_min_range_ + grid_resolution_, raytrace_max_range_);
    spatial_min_component_size_ = std::max(0, spatial_min_component_size_);

    // Footgun guard: terrain mode wants the elevation cloud, not raw scans.
    if (map_source_ == MapSource::TERRAIN && live_topic_ == "/registered_scan")
    {
      live_topic_ = "/terrain_map_ext";
      RCLCPP_WARN(
          get_logger(),
          "map_source=terrain but live_topic was the slice default; using "
          "'/terrain_map_ext'. Set live_topic explicitly to override.");
    }

    max_slope_rad_ = max_slope_deg_ * M_PI / 180.0;
    // Inscribed footprint radius (half the SHORT side) — the clearance the robot
    // physically needs to fit, since it can rotate to align with a gap. The old
    // circumscribed radius (0.5*hypot) demanded the worst-case 45°-in-a-corner
    // margin everywhere, which a route planner over a tight live map can't meet
    // (the robot couldn't even anchor its own start). Exact footprint/orientation
    // safety is handled downstream by the footprint-aware local planner.
    const double inscribed = 0.5 * std::min(vehicle_length_, vehicle_width_);
    clearance_radius_ = std::max(min_clearance_, inscribed);
    if (line_clearance_ < 0.0)
    {
      line_clearance_ = std::max(0.5 * grid_resolution_, clearance_radius_ - grid_resolution_);
    }
  }

  // Allocates the fixed-extent grid centered on (cx, cy). Called once: either
  // at construction (recenter disabled) or on the first odom. Sets map_ready_
  // last so callbacks/goals bail until the grid exists.
  void allocateLiveGrid(double cx, double cy)
  {
    grid_width_ = std::max(1, static_cast<int>(std::ceil(live_map_width_m_ / grid_resolution_)));
    grid_height_ = std::max(1, static_cast<int>(std::ceil(live_map_height_m_ / grid_resolution_)));
    const long long cell_count =
        static_cast<long long>(grid_width_) * static_cast<long long>(grid_height_);
    if (cell_count <= 0 || cell_count > grid_max_cells_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "Live grid size invalid or too large: %dx%d cells=%lld max=%d. "
          "Reduce live_map_width_m/height_m or raise grid_max_cells.",
          grid_width_, grid_height_, cell_count, grid_max_cells_);
      return;
    }

    grid_origin_x_ = cx - 0.5 * live_map_width_m_;
    grid_origin_y_ = cy - 0.5 * live_map_height_m_;

    const std::size_t cells = static_cast<std::size_t>(cell_count);
    grid_.assign(cells, GridCell{});
    filtered_state_.assign(cells, CellState::UNKNOWN);
    {
      std::lock_guard<std::mutex> lock(accum_mutex_);
      accum_.assign(cells, LiveAccum{});
      in_touched_.assign(cells, 0);
      touched_.clear();
    }
    // Seed an all-UNKNOWN snapshot so a goal before any terrain arrives fails
    // cleanly ("no FREE cell") instead of dereferencing a null snapshot, and
    // publish the (empty) extent so RViz shows the grid frame from the start.
    map_ready_ = true;
    publishSnapshot();
    publishGridMap();

    ray_stamp_.assign(cells, 0u);
    ray_gen_ = 0u;
    carve_visit_gen_.assign(cells, 0u);
    carve_gen_ = 0u;

    RCLCPP_INFO(
        get_logger(),
        "Live grid allocated: %dx%d @ %.2fm, extent %.0fx%.0f m, origin (%.2f, %.2f).",
        grid_width_, grid_height_, grid_resolution_,
        live_map_width_m_, live_map_height_m_, grid_origin_x_, grid_origin_y_);
  }

  // Grow-to-fit: if near-field returns (within grow_max_range of the robot)
  // fall outside the current extent, expand the grid to cover them plus a
  // margin. Bounds growth to the observed area (slam_toolbox-style) instead of
  // a fixed 80 m. Caller holds accum_mutex_; gated on !executing_ by the caller
  // so the realloc never races the navigation worker. Executor thread only.
  void maybeGrowGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud, double rx, double ry)
  {
    const double range_sq = live_grid_grow_max_range_ * live_grid_grow_max_range_;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    bool any = false;
    for (const auto& p : cloud.points)
    {
      if (!std::isfinite(p.x) || !std::isfinite(p.y)) continue;
      const double dx = p.x - rx;
      const double dy = p.y - ry;
      if (dx * dx + dy * dy > range_sq) continue;  // far outlier — don't grow to it
      min_x = std::min(min_x, static_cast<double>(p.x));
      max_x = std::max(max_x, static_cast<double>(p.x));
      min_y = std::min(min_y, static_cast<double>(p.y));
      max_y = std::max(max_y, static_cast<double>(p.y));
      any = true;
    }
    if (!any) return;

    const double m = live_grid_growth_margin_;
    const double cur_min_x = grid_origin_x_;
    const double cur_max_x = grid_origin_x_ + grid_width_ * grid_resolution_;
    const double cur_min_y = grid_origin_y_;
    const double cur_max_y = grid_origin_y_ + grid_height_ * grid_resolution_;

    const int dx_lo = (min_x - m) < cur_min_x
                          ? static_cast<int>(std::ceil((cur_min_x - (min_x - m)) / grid_resolution_)) : 0;
    const int dx_hi = (max_x + m) > cur_max_x
                          ? static_cast<int>(std::ceil(((max_x + m) - cur_max_x) / grid_resolution_)) : 0;
    const int dy_lo = (min_y - m) < cur_min_y
                          ? static_cast<int>(std::ceil((cur_min_y - (min_y - m)) / grid_resolution_)) : 0;
    const int dy_hi = (max_y + m) > cur_max_y
                          ? static_cast<int>(std::ceil(((max_y + m) - cur_max_y) / grid_resolution_)) : 0;
    if (dx_lo + dx_hi + dy_lo + dy_hi == 0) return;
    growGrid(dx_lo, dx_hi, dy_lo, dy_hi);
  }

  // Reallocates the grid larger by the given per-side cell counts, copying every
  // existing cell to its shifted position (so indices remap by a constant
  // offset). Publishes a fresh snapshot at the new dimensions immediately so a
  // goal accepted before the next mapping tick reads dims matching the snapshot.
  void growGrid(int dx_lo, int dx_hi, int dy_lo, int dy_hi)
  {
    const int new_w = grid_width_ + dx_lo + dx_hi;
    const int new_h = grid_height_ + dy_lo + dy_hi;
    const long long new_cells = static_cast<long long>(new_w) * static_cast<long long>(new_h);
    if (new_cells <= 0 || new_cells > grid_max_cells_)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Grid growth to %dx%d (%lld cells) would exceed grid_max_cells=%d; "
          "capping. Far area will not be mapped — raise grid_max_cells or lower "
          "live_grid_grow_max_range.",
          new_w, new_h, new_cells, grid_max_cells_);
      return;
    }
    const std::size_t nc = static_cast<std::size_t>(new_cells);

    std::vector<GridCell> new_grid(nc);
    std::vector<CellState> new_filtered(nc, CellState::UNKNOWN);
    std::vector<LiveAccum> new_accum(nc);
    std::vector<std::uint8_t> new_in_touched(nc, 0);
    for (int gy = 0; gy < grid_height_; ++gy)
    {
      const int row_old = gy * grid_width_;
      const int row_new = (gy + dy_lo) * new_w + dx_lo;
      for (int gx = 0; gx < grid_width_; ++gx)
      {
        new_grid[row_new + gx] = grid_[row_old + gx];
        new_filtered[row_new + gx] = filtered_state_[row_old + gx];
        new_accum[row_new + gx] = accum_[row_old + gx];
        new_in_touched[row_new + gx] = in_touched_[row_old + gx];
      }
    }
    for (int& t : touched_)
    {
      const int ogx = t % grid_width_;
      const int ogy = t / grid_width_;
      t = (ogy + dy_lo) * new_w + (ogx + dx_lo);
    }

    grid_.swap(new_grid);
    filtered_state_.swap(new_filtered);
    accum_.swap(new_accum);
    in_touched_.swap(new_in_touched);
    grid_origin_x_ -= dx_lo * grid_resolution_;
    grid_origin_y_ -= dy_lo * grid_resolution_;
    grid_width_ = new_w;
    grid_height_ = new_h;
    // Generation-stamped scratch: resize + zero. The counters reset to 0 and
    // are pre-incremented before use, so 0 always reads as "unvisited".
    carve_visit_gen_.assign(nc, 0u);
    carve_gen_ = 0u;
    ray_stamp_.assign(nc, 0u);
    ray_gen_ = 0u;

    // New perimeter cells are UNKNOWN with sdf 0 (conservative) until the next
    // mapping tick rebuilds the SDF; publish now so any goal sees matching dims.
    publishSnapshot();
    publishGridMap();

    RCLCPP_INFO(
        get_logger(),
        "Live grid grew to %dx%d (%.0fx%.0f m), origin (%.2f, %.2f).",
        grid_width_, grid_height_,
        grid_width_ * grid_resolution_, grid_height_ * grid_resolution_,
        grid_origin_x_, grid_origin_y_);
  }

  // Felzenszwalb-Huttenlocher 1D squared-distance transform along a strip.
  // f_in/f_out are squared distances; samples f_in[i] = 0 are sources.
  // Pre-allocated scratch v[] (index of parabola in lower envelope) and
  // zb[] (boundaries) are sized to >= n+1 by the caller.
  static void edt1d(
      const double* f_in, double* f_out, int n, int* v, double* zb)
  {
    constexpr double kInf = std::numeric_limits<double>::infinity();
    int k = 0;
    v[0] = 0;
    zb[0] = -kInf;
    zb[1] = kInf;
    for (int q = 1; q < n; ++q)
    {
      double s = 0.0;
      while (true)
      {
        const double vk = static_cast<double>(v[k]);
        const double qd = static_cast<double>(q);
        // Intersection of parabolas rooted at v[k] and q.
        s = ((f_in[q] + qd * qd) - (f_in[v[k]] + vk * vk)) / (2.0 * (qd - vk));
        if (s > zb[k])
        {
          break;
        }
        --k;
      }
      ++k;
      v[k] = q;
      zb[k] = s;
      zb[k + 1] = kInf;
    }
    k = 0;
    for (int q = 0; q < n; ++q)
    {
      while (zb[k + 1] < static_cast<double>(q))
      {
        ++k;
      }
      const double dq = static_cast<double>(q - v[k]);
      f_out[q] = dq * dq + f_in[v[k]];
    }
  }

  // Rebuilds grid_[*].sdf = meters to the nearest non-FREE cell in the filtered
  // view (filtered_state_), via a 2-pass EDT. O(N); ~ms for a 160k-cell grid.
  // Runs on the executor thread (mapping tick) so grid_ is touched single-threaded.
  void buildSdf()
  {
    const std::size_t cells = grid_.size();
    if (cells == 0)
    {
      return;
    }

    // Large finite sentinel — avoids inf - inf = NaN inside the parabola
    // intersection arithmetic. Dominates any real squared distance (bounded
    // by width^2 + height^2 in cell units).
    const double sentinel =
        static_cast<double>(grid_width_) * grid_width_ +
        static_cast<double>(grid_height_) * grid_height_ + 1.0;

    // SDF sources (distance 0) are the cells the robot must keep clearance
    // from. By default that's only BLOCKED (real obstacles): treating UNKNOWN
    // as a source would inflate clearance off the entire unobserved frontier of
    // a live map and make most observed-FREE space non-traversable. Set
    // clearance_inflate_unknown to restore the conservative behavior (treat
    // unobserved cells as obstacles too — appropriate for a complete saved map).
    std::vector<double> squared(cells);
    for (std::size_t i = 0; i < cells; ++i)
    {
      const bool is_source = clearance_inflate_unknown_
                                 ? (filtered_state_[i] != CellState::FREE)
                                 : (filtered_state_[i] == CellState::BLOCKED);
      squared[i] = is_source ? 0.0 : sentinel;
    }

    const int n = std::max(grid_width_, grid_height_);
    std::vector<double> row_in(n);
    std::vector<double> row_out(n);
    std::vector<int> v(n + 1);
    std::vector<double> zb(n + 2);

    for (int gy = 0; gy < grid_height_; ++gy)
    {
      for (int gx = 0; gx < grid_width_; ++gx)
      {
        row_in[gx] = squared[gridIndex(gx, gy)];
      }
      edt1d(row_in.data(), row_out.data(), grid_width_, v.data(), zb.data());
      for (int gx = 0; gx < grid_width_; ++gx)
      {
        squared[gridIndex(gx, gy)] = row_out[gx];
      }
    }
    for (int gx = 0; gx < grid_width_; ++gx)
    {
      for (int gy = 0; gy < grid_height_; ++gy)
      {
        row_in[gy] = squared[gridIndex(gx, gy)];
      }
      edt1d(row_in.data(), row_out.data(), grid_height_, v.data(), zb.data());
      for (int gy = 0; gy < grid_height_; ++gy)
      {
        squared[gridIndex(gx, gy)] = row_out[gy];
      }
    }

    for (std::size_t i = 0; i < cells; ++i)
    {
      grid_[i].sdf = std::sqrt(squared[i]) * grid_resolution_;
    }
  }

  // Builds filtered_state_ from the raw log-odds grid_ states. The filter is a
  // VIEW consumed by the SDF and the published map; grid_[*].state (the
  // log-odds classification) is never modified, so the temporal filter and the
  // spatial one can't fight each other. Optional morphological opening, then
  // isolated-component removal. Executor thread only.
  void computeFilteredStates()
  {
    const std::size_t cells = grid_.size();
    if (filtered_state_.size() != cells)
    {
      filtered_state_.assign(cells, CellState::UNKNOWN);
    }
    for (std::size_t i = 0; i < cells; ++i)
    {
      filtered_state_[i] = grid_[i].state;
    }
    if (!spatial_filter_enabled_)
    {
      return;
    }
    if (spatial_opening_)
    {
      morphologicalOpenBlocked();
    }
    if (spatial_min_component_size_ > 0)
    {
      removeSmallBlockedComponents();
    }
  }

  // 8-connected opening (erode then dilate) of the BLOCKED set. Erosion clears
  // a BLOCKED cell unless all 8 neighbors are BLOCKED; dilation restores
  // survivors, restricted to originally-BLOCKED cells (opening is
  // anti-extensive, so the result never paints over FREE/UNKNOWN). Removes thin
  // specks but can also thin one-cell walls — off by default.
  void morphologicalOpenBlocked()
  {
    const int w = grid_width_;
    const int h = grid_height_;
    auto is_blocked = [&](int gx, int gy, const std::vector<CellState>& s) {
      return gx >= 0 && gy >= 0 && gx < w && gy < h &&
             s[gy * w + gx] == CellState::BLOCKED;
    };

    std::vector<CellState> eroded = filtered_state_;
    for (int gy = 0; gy < h; ++gy)
    {
      for (int gx = 0; gx < w; ++gx)
      {
        if (filtered_state_[gy * w + gx] != CellState::BLOCKED)
        {
          continue;
        }
        bool all_blocked = true;
        for (int dy = -1; dy <= 1 && all_blocked; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (!is_blocked(gx + dx, gy + dy, filtered_state_))
            {
              all_blocked = false;
              break;
            }
          }
        }
        if (!all_blocked)
        {
          eroded[gy * w + gx] = CellState::FREE;
        }
      }
    }

    for (int gy = 0; gy < h; ++gy)
    {
      for (int gx = 0; gx < w; ++gx)
      {
        const int idx = gy * w + gx;
        if (filtered_state_[idx] != CellState::BLOCKED)
        {
          continue;
        }
        bool near_survivor = false;
        for (int dy = -1; dy <= 1 && !near_survivor; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (is_blocked(gx + dx, gy + dy, eroded))
            {
              near_survivor = true;
              break;
            }
          }
        }
        filtered_state_[idx] = near_survivor ? CellState::BLOCKED : CellState::FREE;
      }
    }
  }

  // Clears BLOCKED connected components (8-connected) of at most
  // spatial_min_component_size_ cells to FREE — but only when the component
  // touches no UNKNOWN neighbor, i.e. it sits inside known-free space and is
  // almost certainly speckle. Small blocked clusters at the unobserved
  // frontier are kept (likely real, partially-seen obstacles). O(N).
  void removeSmallBlockedComponents()
  {
    const int w = grid_width_;
    const int h = grid_height_;
    const std::size_t cells = filtered_state_.size();
    const int max_speck = spatial_min_component_size_;

    std::vector<char> visited(cells, 0);
    std::vector<int> stack;
    std::vector<int> component;
    for (int start = 0; start < static_cast<int>(cells); ++start)
    {
      if (visited[start] || filtered_state_[start] != CellState::BLOCKED)
      {
        continue;
      }
      stack.clear();
      component.clear();
      stack.push_back(start);
      visited[start] = 1;
      bool touches_unknown = false;
      int count = 0;
      while (!stack.empty())
      {
        const int idx = stack.back();
        stack.pop_back();
        ++count;
        if (count <= max_speck)
        {
          component.push_back(idx);   // only a speck's worth is worth remembering
        }
        const int gx = idx % w;
        const int gy = idx / w;
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0)
            {
              continue;
            }
            const int nx = gx + dx;
            const int ny = gy + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h)
            {
              continue;
            }
            const int nidx = ny * w + nx;
            const CellState ns = filtered_state_[nidx];
            if (ns == CellState::UNKNOWN)
            {
              touches_unknown = true;
            }
            else if (ns == CellState::BLOCKED && !visited[nidx])
            {
              visited[nidx] = 1;
              stack.push_back(nidx);
            }
          }
        }
      }
      if (count <= max_speck && !touches_unknown)
      {
        for (int idx : component)
        {
          filtered_state_[idx] = CellState::FREE;
        }
      }
    }
  }

  std::shared_ptr<const FusedSnapshot> getFusedSnapshot() const
  {
    std::lock_guard<std::mutex> lock(fused_mutex_);
    return fused_snapshot_;
  }

  // Copies the current grid_ (state/z/sdf) into a fresh immutable snapshot and
  // swaps it in for the planner. Executor thread only.
  void publishSnapshot()
  {
    const std::size_t cells = grid_.size();
    auto snap = std::make_shared<FusedSnapshot>();
    snap->state.resize(cells);
    snap->z.resize(cells);
    snap->sdf.resize(cells);
    for (std::size_t i = 0; i < cells; ++i)
    {
      snap->state[i] = filtered_state_[i];
      snap->z[i] = grid_[i].z;
      snap->sdf[i] = grid_[i].sdf;
    }
    std::lock_guard<std::mutex> lock(fused_mutex_);
    fused_snapshot_ = std::move(snap);
  }

  // Tally raw per-cell evidence from one incoming cloud. No log-odds here —
  // that's collapsed to one vote per cell at the mapping tick so the filter
  // rate is decoupled from the sensor rate. Per-point classification branches
  // on map_source_; touched cells are deduped into touched_ for the tick.
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!map_ready_)
    {
      return;
    }
    // /registered_scan is already registered into the SLAM world frame. We do
    // NOT transform it — the planner holds no TF buffer, and odometry is read
    // the same way. The SLAM backend may label the world frame "map",
    // "sensor_init", etc.; accept any frame and treat its coordinates as the
    // world frame. Note the actual frame once so it can be sanity-checked.
    if (!cloud_frame_logged_)
    {
      cloud_frame_logged_ = true;
      RCLCPP_INFO(
          get_logger(),
          "First cloud on '%s': frame='%s', map_frame='%s' — treating cloud "
          "coordinates as the world frame (no transform applied).",
          live_topic_.c_str(),
          msg->header.frame_id.empty() ? "<none>" : msg->header.frame_id.c_str(),
          map_frame_.c_str());
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty())
    {
      return;
    }

    // Sensor pose (robot xyz). x,y are the ray-trace origin + grow-to-fit
    // reference; z is the lidar plane the slice band is centered on.
    bool have_sensor = false;
    double rx = 0.0;
    double ry = 0.0;
    double rz = 0.0;
    if (has_odom_.load())
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      rx = latest_pose_.position.x;
      ry = latest_pose_.position.y;
      rz = latest_pose_.position.z;
      have_sensor = true;
    }
    // SLICE mode needs the robot z to place the band; without odom, skip.
    if (map_source_ == MapSource::SLICE && !have_sensor)
    {
      return;
    }
    const double slice_band_center = rz + slice_band_offset_;

    std::lock_guard<std::mutex> lock(accum_mutex_);

    // Grow-to-fit: expand the grid to cover near-field returns. Gated on
    // !executing_ so the realloc never races the navigation worker (it reads
    // grid_ dims; growth only changes them between goals). Far outliers beyond
    // grow_max_range are not grown to.
    if (live_grid_growth_ && have_sensor && !executing_.load())
    {
      maybeGrowGrid(cloud, rx, ry);
    }

    int sgx = 0;
    int sgy = 0;
    const bool ray_clear = map_source_ == MapSource::SLICE && raytrace_clearing_ &&
                           have_sensor && worldToGrid(rx, ry, sgx, sgy);
    ++ray_gen_;
    if (ray_clear)
    {
      scan_floor_cells_.clear();
    }

    std::size_t in_grid = 0;
    for (const auto& point : cloud.points)
    {
      if (!std::isfinite(point.x) || !std::isfinite(point.y))
      {
        continue;
      }
      int gx = 0;
      int gy = 0;
      if (!worldToGrid(point.x, point.y, gx, gy))
      {
        continue;          // outside the extent — dropped
      }
      ++in_grid;
      const int idx = gridIndex(gx, gy);
      LiveAccum& a = accum_[idx];

      if (map_source_ == MapSource::SLICE)
      {
        const SliceEvidence ev = tallySlicePoint(a, point.z, slice_band_center);
        if (ev == SliceEvidence::IGNORE)
        {
          continue;
        }
        if (!in_touched_[idx])
        {
          in_touched_[idx] = 1;
          touched_.push_back(idx);
        }
        if (ray_clear)
        {
          // Obstacle returns mark the cell so clearing rays never free-vote it
          // (a standing obstacle must not decay). FREE returns are collected as
          // clearing targets — only beams to a visible floor point are traced,
          // and an obstacle occludes the floor behind it, so a clearing beam
          // never sweeps through a real obstacle. The target list is deduped
          // after the loop, so floor/obstacle return order in a cell is moot.
          if (ev == SliceEvidence::OBSTACLE)
          {
            ray_stamp_[idx] = ray_gen_;
          }
          else  // SliceEvidence::FREE
          {
            scan_floor_cells_.push_back(idx);
          }
        }
      }
      else if (tallyTerrainPoint(a, point) && !in_touched_[idx])
      {
        in_touched_[idx] = 1;
        touched_.push_back(idx);
      }
    }

    if (ray_clear)
    {
      std::sort(scan_floor_cells_.begin(), scan_floor_cells_.end());
      scan_floor_cells_.erase(
          std::unique(scan_floor_cells_.begin(), scan_floor_cells_.end()),
          scan_floor_cells_.end());
      for (const int endpoint : scan_floor_cells_)
      {
        raytraceClearLine(sgx, sgy, endpoint);
      }
    }

    last_cloud_size_ = cloud.size();
    last_cloud_in_grid_ = in_grid;
  }

  // Walks the 2D line from the sensor cell to a floor-return cell and casts a
  // FREE (miss) vote into each cell the beam passes through (excluding the
  // endpoint and any cell that is itself a scan endpoint this tick). This is
  // the nav2/costmap raytraceFreespace idea: cells a beam sweeps are free, so a
  // dynamic obstacle that moves away gets cleared once the floor behind it is
  // seen again. Cleared cells with no observed ground of their own borrow the
  // endpoint's ground z so the slope/step check stays consistent. accum_mutex_
  // is held by the caller; executor thread only.
  void raytraceClearLine(int sgx, int sgy, int endpoint_idx)
  {
    const int ex = endpoint_idx % grid_width_;
    const int ey = endpoint_idx / grid_width_;
    if (sgx == ex && sgy == ey)
    {
      return;
    }
    const LiveAccum& ea = accum_[endpoint_idx];
    const double zE = ea.ground_count > 0
                          ? ea.ground_sum / static_cast<double>(ea.ground_count)
                          : grid_[endpoint_idx].z;

    // Amanatides-Woo voxel traversal from the sensor cell center to the
    // endpoint cell center — visits every cell the beam crosses (no gaps),
    // unlike interval sampling. Endpoint and sensor cells are excluded.
    const double fx = static_cast<double>(sgx) + 0.5;
    const double fy = static_cast<double>(sgy) + 0.5;
    const double tx = static_cast<double>(ex) + 0.5;
    const double ty = static_cast<double>(ey) + 0.5;
    const double dx = tx - fx;
    const double dy = ty - fy;
    const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);
    constexpr double kInf = std::numeric_limits<double>::infinity();
    const double inv_dx = step_x != 0 ? 1.0 / std::abs(dx) : kInf;
    const double inv_dy = step_y != 0 ? 1.0 / std::abs(dy) : kInf;
    double t_max_x = step_x > 0 ? (static_cast<double>(sgx + 1) - fx) * inv_dx
                   : step_x < 0 ? (fx - static_cast<double>(sgx)) * inv_dx : kInf;
    double t_max_y = step_y > 0 ? (static_cast<double>(sgy + 1) - fy) * inv_dy
                   : step_y < 0 ? (fy - static_cast<double>(sgy)) * inv_dy : kInf;
    const double t_delta_x = inv_dx;
    const double t_delta_y = inv_dy;

    int cx = sgx;
    int cy = sgy;
    const int max_iter = grid_width_ + grid_height_ + 4;
    for (int iter = 0; (cx != ex || cy != ey) && iter < max_iter; ++iter)
    {
      if (t_max_x < t_max_y)
      {
        cx += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cy += step_y;
        t_max_y += t_delta_y;
      }
      if (cx == ex && cy == ey)
      {
        break;  // reached the endpoint — never clear it
      }
      if (!inGrid(cx, cy))
      {
        break;
      }
      const double r = std::hypot(static_cast<double>(cx - sgx) * grid_resolution_,
                                  static_cast<double>(cy - sgy) * grid_resolution_);
      if (r < raytrace_min_range_)
      {
        continue;
      }
      if (r > raytrace_max_range_)
      {
        break;
      }
      const int c = gridIndex(cx, cy);
      if (ray_stamp_[c] == ray_gen_)
      {
        continue;  // an obstacle endpoint or already cleared this scan
      }
      ray_stamp_[c] = ray_gen_;
      LiveAccum& a = accum_[c];
      if (a.free_count < 65535)
      {
        a.free_count++;  // clearing miss vote
      }
      if (a.ground_count == 0)
      {
        // A cleared cell has no floor return of its own; give it a provisional
        // ground z (the beam's floor endpoint) for the slope check only. No
        // ground evidence is added, so a real floor return later overrides it.
        grid_[c].z = zE;
      }
      if (!in_touched_[c])
      {
        in_touched_[c] = 1;
        touched_.push_back(c);
      }
    }
  }

  // TERRAIN mode: intensity = height above terrain-analysis ground. Returns
  // true if the point added evidence (so the cell should be reclassified).
  bool tallyTerrainPoint(LiveAccum& a, const pcl::PointXYZI& point)
  {
    const float intensity = point.intensity;  // height above ground, meters
    if (!std::isfinite(intensity))
    {
      return false;
    }
    if (intensity < obstacle_height_)
    {
      if (a.free_count < 65535)
      {
        a.free_count++;    // ground / free evidence
      }
      a.ground_sum += static_cast<double>(point.z) - static_cast<double>(intensity);
      a.ground_count++;
      return true;
    }
    if (intensity <= blockage_ceiling_)
    {
      if (a.obs_count < 65535)
      {
        a.obs_count++;     // obstacle or no-data blockage evidence
      }
      return true;
    }
    return false;          // overhead / outlier — ignore
  }

  // SLICE mode: thin horizontal band at the lidar plane. `band_center` is the
  // robot's current z (+ slice_band_offset), so the band follows the robot and
  // is immune to SLAM z-drift / floor-level changes. A return within
  // slice_band_half of the center is an OBSTACLE; below the band it's
  // floor/FREE (and sets the cell ground z); above the band it's overhead and
  // ignored. No per-cell floor estimation — obstacle height is judged against
  // the lidar plane directly. Returns what the point contributed.
  SliceEvidence tallySlicePoint(LiveAccum& a, float pz, double band_center)
  {
    if (!std::isfinite(pz))
    {
      return SliceEvidence::IGNORE;
    }
    const double rel = static_cast<double>(pz) - band_center;
    if (rel > slice_band_half_)
    {
      return SliceEvidence::IGNORE;   // above the band — overhead
    }
    if (rel >= -slice_band_half_)
    {
      if (a.obs_count < 65535)
      {
        a.obs_count++;                // within the band — obstacle
      }
      return SliceEvidence::OBSTACLE;
    }
    // below the band — floor / free evidence; its z defines the cell ground.
    if (a.free_count < 65535)
    {
      a.free_count++;
    }
    a.ground_sum += static_cast<double>(pz);
    a.ground_count++;
    return SliceEvidence::FREE;
  }

  // Classify one cell from its accumulator into grid_. Sticky-FREE: a FREE cell
  // stays FREE through the UNKNOWN band; only crossing live_l_occ_thr moves it
  // to BLOCKED. Grid-boundary cells are forced UNKNOWN (no negative-obstacle
  // protection at the extent edge).
  void classifyCell(int idx, const LiveAccum& a)
  {
    CellState next;
    if (a.logodds >= live_l_occ_thr_)
    {
      next = CellState::BLOCKED;
    }
    else if (a.logodds <= live_l_free_thr_)
    {
      next = CellState::FREE;
    }
    else
    {
      next = CellState::UNKNOWN;
    }
    if (grid_[idx].state == CellState::FREE && next == CellState::UNKNOWN)
    {
      next = CellState::FREE;  // sticky-FREE override
    }

    // Negative-obstacle protection at the grid extent: force the boundary ring
    // UNKNOWN so the planner won't path off the edge. Only in conservative mode
    // — otherwise (live grow-to-fit map) this seals an observed edge cell as a
    // non-traversable wall, and the extent is the unobserved frontier anyway.
    const int gx = idx % grid_width_;
    const int gy = idx / grid_width_;
    if (clearance_inflate_unknown_ &&
        (gx == 0 || gy == 0 || gx == grid_width_ - 1 || gy == grid_height_ - 1))
    {
      next = CellState::UNKNOWN;
    }
    grid_[idx].state = next;

    if (a.ground_count > 0)
    {
      grid_[idx].z = a.ground_sum / static_cast<double>(a.ground_count);
    }
  }

  // Fills the lidar's near-field blind cone with FREE. A low-mounted Mid-360
  // (downward FOV limit ~-7°) sees no ground closer than ~height/tan(7°) — a
  // several-metre hole around the robot that never gets returns. The robot is
  // physically standing on traversable ground there, so we flood-fill it: BFS
  // from the robot cell through genuinely-unobserved cells, bounded by
  // robot_free_radius_, stopping at the first observed cells (walls or the
  // observed floor ring) so it never leaks into occluded space behind a wall.
  // Filled cells borrow the nearest observed ground z; if no ground has been
  // observed near the robot yet, carving is skipped rather than guessing a z.
  // Observed evidence (free or wall) is never overridden. Returns cells turned FREE.
  std::size_t carveRobotFreeSpace()
  {
    if (!robot_free_carve_ || !has_odom_.load())
    {
      return 0;
    }
    double rx = 0.0;
    double ry = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      rx = latest_pose_.position.x;
      ry = latest_pose_.position.y;
    }
    int cgx = 0;
    int cgy = 0;
    if (!worldToGrid(rx, ry, cgx, cgy))
    {
      return 0;
    }
    const int rad =
        std::max(1, static_cast<int>(std::ceil(robot_free_radius_ / grid_resolution_)));
    const long rad_sq = static_cast<long>(rad) * rad;

    std::lock_guard<std::mutex> lock(accum_mutex_);
    // Nearest observed-ground z reference to the robot, within the radius. The
    // Mid-360's downward FOV limit puts the closest ground return several
    // meters out, so search the whole radius. Without any observed ground,
    // don't carve — a guessed z would break the slope check at the boundary.
    double ground_z = 0.0;
    long best_d2 = std::numeric_limits<long>::max();
    for (int dy = -rad; dy <= rad; ++dy)
    {
      for (int dx = -rad; dx <= rad; ++dx)
      {
        const long d2 = static_cast<long>(dx) * dx + static_cast<long>(dy) * dy;
        if (d2 > rad_sq || d2 >= best_d2) continue;
        const int gx = cgx + dx;
        const int gy = cgy + dy;
        if (!inGrid(gx, gy)) continue;
        const int idx = gridIndex(gx, gy);
        if (accum_[idx].ground_count > 0)
        {
          best_d2 = d2;
          ground_z = grid_[idx].z;
        }
      }
    }
    if (best_d2 == std::numeric_limits<long>::max())
    {
      return 0;  // no observed ground near the robot yet
    }

    // Flood-fill the connected blind cone: BFS from the robot cell through
    // genuinely-unobserved cells, bounded by the radius, stopping at any
    // observed cell. Fills the near-field the lidar can't reach (free by the
    // robot's presence) yet never crosses a wall or the observed floor ring
    // into occluded space behind it. A generation stamp avoids clearing the
    // visited array each tick.
    if (carve_visit_gen_.size() != grid_.size())
    {
      carve_visit_gen_.assign(grid_.size(), 0u);
      carve_gen_ = 0u;
    }
    ++carve_gen_;
    std::vector<int> stack;
    auto consider = [&](int gx, int gy) {
      if (!inGrid(gx, gy)) return;
      const long d2 = static_cast<long>(gx - cgx) * (gx - cgx) +
                      static_cast<long>(gy - cgy) * (gy - cgy);
      if (d2 > rad_sq) return;
      const int idx = gridIndex(gx, gy);
      if (carve_visit_gen_[idx] == carve_gen_) return;
      carve_visit_gen_[idx] = carve_gen_;   // visited; observed cells act as boundaries
      const LiveAccum& a = accum_[idx];
      if (grid_[idx].state != CellState::UNKNOWN || a.logodds != 0.0f || a.ground_count > 0)
      {
        return;  // observed — don't flood through it
      }
      stack.push_back(idx);
    };

    // Seed from the robot's OWN cell first — it's the blind spot directly under
    // the sensor and is usually pristine UNKNOWN; consider() carves it FREE so
    // the robot can anchor a path start there (the start connector check needs
    // the start cell FREE). Then seed the neighbors so the rest of the cone
    // fills even if the robot's own cell happened to be observed.
    consider(cgx, cgy);
    for (int dy = -1; dy <= 1; ++dy)
    {
      for (int dx = -1; dx <= 1; ++dx)
      {
        if (dx != 0 || dy != 0) consider(cgx + dx, cgy + dy);
      }
    }

    std::size_t carved = 0;
    while (!stack.empty())
    {
      const int idx = stack.back();
      stack.pop_back();
      LiveAccum& a = accum_[idx];
      a.logodds = static_cast<float>(std::clamp(live_l_free_thr_, live_l_min_, live_l_max_));
      grid_[idx].z = ground_z;  // provisional slope z; floor left to the first real return
      classifyCell(idx, a);
      if (grid_[idx].state == CellState::FREE) ++carved;
      const int gx = idx % grid_width_;
      const int gy = idx / grid_width_;
      for (int dy = -1; dy <= 1; ++dy)
      {
        for (int dx = -1; dx <= 1; ++dx)
        {
          if (dx != 0 || dy != 0) consider(gx + dx, gy + dy);
        }
      }
    }
    return carved;
  }

  // Mapping tick: collapse evidence → one log-odds step per touched cell,
  // reclassify, carve free space under the robot, rebuild the SDF, swap the
  // snapshot, and republish the map.
  void onMappingTick()
  {
    if (!map_ready_ || grid_.empty())
    {
      return;
    }

    std::size_t updated = 0;
    {
      std::lock_guard<std::mutex> lock(accum_mutex_);
      for (int idx : touched_)
      {
        in_touched_[idx] = 0;
        LiveAccum& a = accum_[idx];
        if (a.obs_count == 0 && a.free_count == 0)
        {
          continue;  // duplicate index already processed this tick
        }
        if (a.obs_count > 0)
        {
          a.logodds += static_cast<float>(live_l_hit_);  // obstacle wins
        }
        else
        {
          a.logodds -= static_cast<float>(live_l_miss_);
        }
        a.logodds = std::clamp(
            a.logodds, static_cast<float>(live_l_min_), static_cast<float>(live_l_max_));
        a.obs_count = 0;
        a.free_count = 0;
        classifyCell(idx, a);
        ++updated;
      }
      touched_.clear();
    }

    const std::size_t carved = carveRobotFreeSpace();

    if (updated == 0 && carved == 0)
    {
      return;  // nothing changed; skip the filter + EDT + publish
    }

    computeFilteredStates();
    buildSdf();
    publishSnapshot();
    publishGridMap();
    if (publish_debug_markers_)
    {
      publishGridMarkers();
    }

    // Throttled diagnostics. Compares the robot pose to where classified cells
    // actually are, and reports the nearest free cell — so we can separate
    // "no data" (pts=0) from "data in the wrong place" (centroid far from
    // robot) from "robot vicinity unobserved" (nearest_free large, blind cone).
    std::size_t n_free = 0;
    std::size_t n_blocked = 0;
    std::size_t n_unknown = 0;
    std::size_t n_class = 0;
    double cx_sum = 0.0;
    double cy_sum = 0.0;
    constexpr double kInf = std::numeric_limits<double>::infinity();
    double min_x = kInf, max_x = -kInf, min_y = kInf, max_y = -kInf;
    double nearest_free = kInf;
    double rx = 0.0;
    double ry = 0.0;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      rx = latest_pose_.position.x;
      ry = latest_pose_.position.y;
    }
    for (std::size_t i = 0; i < filtered_state_.size(); ++i)
    {
      const CellState s = filtered_state_[i];
      if (s == CellState::FREE) ++n_free;
      else if (s == CellState::BLOCKED) ++n_blocked;
      else { ++n_unknown; continue; }
      ++n_class;
      const int gx = static_cast<int>(i) % grid_width_;
      const int gy = static_cast<int>(i) / grid_width_;
      const double wx = grid_origin_x_ + (static_cast<double>(gx) + 0.5) * grid_resolution_;
      const double wy = grid_origin_y_ + (static_cast<double>(gy) + 0.5) * grid_resolution_;
      cx_sum += wx;
      cy_sum += wy;
      min_x = std::min(min_x, wx);
      max_x = std::max(max_x, wx);
      min_y = std::min(min_y, wy);
      max_y = std::max(max_y, wy);
      if (s == CellState::FREE)
      {
        nearest_free = std::min(nearest_free, std::hypot(wx - rx, wy - ry));
      }
    }
    const double cls_cx = n_class ? cx_sum / static_cast<double>(n_class) : 0.0;
    const double cls_cy = n_class ? cy_sum / static_cast<double>(n_class) : 0.0;
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "slice map: pts=%zu in_grid=%zu | free=%zu blocked=%zu unknown=%zu | "
        "robot=(%.1f,%.1f) classified_centroid=(%.1f,%.1f) "
        "bbox=[%.1f,%.1f]x[%.1f,%.1f] nearest_free=%.2fm | carved=%zu updated=%zu",
        last_cloud_size_, last_cloud_in_grid_, n_free, n_blocked, n_unknown,
        rx, ry, cls_cx, cls_cy,
        n_class ? min_x : 0.0, n_class ? max_x : 0.0,
        n_class ? min_y : 0.0, n_class ? max_y : 0.0,
        nearest_free, carved, updated);
  }

  // True if any sample along the next replan_horizon_m_ of route is no longer
  // traversable in the current snapshot (non-FREE or sub-clearance).
  bool isRouteBlocked(
      const std::vector<geometry_msgs::msg::Point>& route,
      std::size_t route_index) const
  {
    if (route.size() < 2)
    {
      return false;
    }
    auto snap = getFusedSnapshot();
    if (!snap)
    {
      return false;
    }

    const double step = std::max(0.05, 0.5 * grid_resolution_);
    double walked = 0.0;
    for (std::size_t i = route_index; i + 1 < route.size(); ++i)
    {
      const geometry_msgs::msg::Point& a = route[i];
      const geometry_msgs::msg::Point& b = route[i + 1];
      const double seg = pointDistXY(a, b);
      if (seg < 1e-6)
      {
        continue;
      }
      const int n = std::max(1, static_cast<int>(std::ceil(seg / step)));
      for (int k = 1; k <= n; ++k)
      {
        const double t = static_cast<double>(k) / static_cast<double>(n);
        const geometry_msgs::msg::Point sample = lerpPoint(a, b, t);
        int gx = 0;
        int gy = 0;
        if (!worldToGrid(sample.x, sample.y, gx, gy))
        {
          return true;
        }
        const int idx = gridIndex(gx, gy);
        if (snap->state[idx] != CellState::FREE || snap->sdf[idx] < clearance_radius_)
        {
          return true;
        }
        walked += seg / n;
        if (walked >= replan_horizon_m_)
        {
          return false;
        }
      }
      if (walked >= replan_horizon_m_)
      {
        return false;
      }
    }
    return false;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      latest_pose_ = msg->pose.pose;
      latest_odom_stamp_ = msg->header.stamp;
      has_odom_.store(true);
    }
    // One-shot: center the grid on the robot's first pose.
    if (live_map_recenter_on_first_odom_ && !grid_allocated_.exchange(true))
    {
      allocateLiveGrid(msg->pose.pose.position.x, msg->pose.pose.position.y);
    }
  }

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!map_ready_)
    {
      RCLCPP_WARN(get_logger(), "Ignoring /goal_pose: grid is not ready.");
      return;
    }
    if (!has_odom_.load())
    {
      RCLCPP_WARN(get_logger(), "Ignoring /goal_pose: no /state_estimation received yet.");
      return;
    }
    if (!msg->header.frame_id.empty() && msg->header.frame_id != map_frame_)
    {
      RCLCPP_WARN(
          get_logger(),
          "Ignoring /goal_pose in frame '%s'; expected '%s'.",
          msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
    if (executing_.exchange(true))
    {
      RCLCPP_WARN(get_logger(), "Ignoring /goal_pose: another goal is active.");
      return;
    }

    RCLCPP_INFO(
        get_logger(), "Accepted RViz /goal_pose x=%.2f y=%.2f z=%.2f",
        msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    const auto goal_pose = *msg;
    startWorker([this, goal_pose] { executeTopicGoal(goal_pose); });
  }

  bool currentPosition(geometry_msgs::msg::Point& point, geometry_msgs::msg::PoseStamped& pose)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (!has_odom_.load())
    {
      return false;
    }
    point = latest_pose_.position;
    pose.header.frame_id = map_frame_;
    pose.header.stamp = latest_odom_stamp_;
    pose.pose = latest_pose_;
    return true;
  }

  rclcpp_action::GoalResponse handleGoal(
      const rclcpp_action::GoalUUID&,
      std::shared_ptr<const NavigateToPose::Goal> goal)
  {
    if (!map_ready_)
    {
      RCLCPP_WARN(get_logger(), "Rejecting navigation goal: grid is not ready.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!has_odom_.load())
    {
      RCLCPP_WARN(get_logger(), "Rejecting navigation goal: no /state_estimation received yet.");
      return rclcpp_action::GoalResponse::REJECT;
    }
    // Atomic claim: if another goal is already in flight, reject without
    // touching executing_. Two simultaneous goals can no longer both pass
    // admission and start workers.
    if (executing_.exchange(true))
    {
      RCLCPP_WARN(get_logger(), "Rejecting navigation goal: another goal is active.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(
        get_logger(), "Accepted PCD-grid goal x=%.2f y=%.2f z=%.2f",
        goal->pose.pose.position.x,
        goal->pose.pose.position.y,
        goal->pose.pose.position.z);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleNavigateToPose>)
  {
    RCLCPP_INFO(get_logger(), "Received request to cancel PCD-grid navigation.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    // executing_ already set true by handleGoal (atomic claim). Spawn worker.
    startWorker([this, goal_handle] { executeGoal(goal_handle); });
  }

  void executeGoal(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    runNavigation(
        goal_handle->get_goal()->pose.pose.position,
        "PCD-grid",
        goal_handle);
  }

  void executeTopicGoal(const geometry_msgs::msg::PoseStamped goal_pose)
  {
    runNavigation(goal_pose.pose.position, "RViz /goal_pose", nullptr);
  }

  // Unified navigation loop. goal_handle may be null (topic flow) — then
  // cancel checks and action-server callbacks are skipped, but the rest
  // of the logic (plan, lookahead, replan) is identical.
  void runNavigation(
      const geometry_msgs::msg::Point& final_goal,
      const std::string& label,
      std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    ExecutingGuard executing_guard(executing_);
    geometry_msgs::msg::Point current;
    geometry_msgs::msg::PoseStamped current_pose;
    if (!currentPosition(current, current_pose))
    {
      reportAbort(goal_handle, label, "No current pose available.");
      return;
    }

    std::vector<geometry_msgs::msg::Point> route;
    std::string error;
    if (!planGridRoute(current, final_goal, route, error))
    {
      reportAbort(goal_handle, label, error);
      return;
    }

    publishRoute(route);
    std::size_t route_index = route.size() > 1 ? 1 : 0;
    const rclcpp::Time start_time = now();
    rclcpp::Time last_replan_check = start_time;
    int replan_failures = 0;
    rclcpp::Rate rate(waypoint_rate_);

    while (rclcpp::ok() && !stop_requested_.load())
    {
      if (goal_handle && goal_handle->is_canceling())
      {
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->canceled(result);
        RCLCPP_INFO(get_logger(), "%s navigation canceled.", label.c_str());
        return;
      }

      if (!currentPosition(current, current_pose))
      {
        reportAbort(goal_handle, label, "Lost current pose.");
        return;
      }

      if (pointDistXY(current, route.back()) <= goal_tolerance_)
      {
        publishWaypoint(route.back());
        if (goal_handle)
        {
          auto result = std::make_shared<NavigateToPose::Result>();
          goal_handle->succeed(result);
        }
        RCLCPP_INFO(get_logger(), "%s navigation succeeded.", label.c_str());
        return;
      }

      // Throttled live replanning. Only triggers when the upcoming route
      // segment is no longer traversable in the latest snapshot.
      if ((now() - last_replan_check).seconds() >= replan_period_sec_)
      {
        last_replan_check = now();
        if (isRouteBlocked(route, route_index))
        {
          std::vector<geometry_msgs::msg::Point> new_route;
          std::string replan_error;
          if (planGridRoute(current, final_goal, new_route, replan_error))
          {
            route = std::move(new_route);
            route_index = route.size() > 1 ? 1 : 0;
            replan_failures = 0;
            publishRoute(route);
            RCLCPP_INFO(get_logger(),
                        "%s replanned around live obstacle.", label.c_str());
          }
          else
          {
            ++replan_failures;
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "%s live replan failed (%d/%d): %s",
                label.c_str(), replan_failures, max_replan_attempts_,
                replan_error.c_str());
            if (max_replan_attempts_ > 0 && replan_failures >= max_replan_attempts_)
            {
              reportAbort(goal_handle, label, "Live replanning failed repeatedly.");
              return;
            }
          }
        }
      }

      const geometry_msgs::msg::Point waypoint = selectWaypoint(current, route, route_index);
      publishWaypoint(waypoint);
      if (goal_handle)
      {
        publishFeedback(goal_handle, current_pose, route, route_index, start_time);
      }
      rate.sleep();
    }

    reportAbort(goal_handle, label, "ROS shutdown during navigation.");
  }

  void reportAbort(
      const std::shared_ptr<GoalHandleNavigateToPose>& goal_handle,
      const std::string& label,
      const std::string& message)
  {
    if (goal_handle && goal_handle->is_active())
    {
      auto result = std::make_shared<NavigateToPose::Result>();
      goal_handle->abort(result);
    }
    RCLCPP_ERROR(get_logger(), "%s navigation aborted: %s",
                 label.c_str(), message.c_str());
  }

  bool inGrid(int gx, int gy) const
  {
    return gx >= 0 && gy >= 0 && gx < grid_width_ && gy < grid_height_;
  }

  int gridIndex(int gx, int gy) const
  {
    return gy * grid_width_ + gx;
  }

  bool worldToGrid(double x, double y, int& gx, int& gy) const
  {
    gx = static_cast<int>(std::floor((x - grid_origin_x_) / grid_resolution_));
    gy = static_cast<int>(std::floor((y - grid_origin_y_) / grid_resolution_));
    return inGrid(gx, gy);
  }

  // Cell-center XY (z left at 0 — the planner overwrites/ignores z, and never
  // reads grid_ from the worker thread). The marker publisher reads grid_[i].z
  // directly on the executor thread.
  geometry_msgs::msg::Point gridCenter(int index) const
  {
    const int gx = index % grid_width_;
    const int gy = index / grid_width_;
    geometry_msgs::msg::Point point;
    point.x = grid_origin_x_ + (static_cast<double>(gx) + 0.5) * grid_resolution_;
    point.y = grid_origin_y_ + (static_cast<double>(gy) + 0.5) * grid_resolution_;
    point.z = 0.0;
    return point;
  }

  bool gridFree(int index, const FusedSnapshot& snap) const
  {
    return index >= 0 && index < static_cast<int>(snap.state.size()) &&
           snap.state[index] == CellState::FREE;
  }

  bool gridTraversable(int index, const FusedSnapshot& snap) const
  {
    return gridFree(index, snap) && snap.sdf[index] >= clearance_radius_;
  }

  // Slope + step constraint between two FREE cells, read from the snapshot.
  // OFF by default: the SLICE band model's per-cell z is a rough mean of
  // below-band returns (not a clean terrain elevation), and the per-step slope
  // threshold step_xy*tan(max_slope) shrinks with resolution — at 0.05 m it is
  // only ~2.3 cm, below lidar z-noise, so it would reject flat-floor steps and
  // make A* find no path. Enable only with a reliable per-cell elevation
  // (e.g. terrain mode at a coarse resolution).
  bool stepWithinSlope(int from_idx, int to_idx, const FusedSnapshot& snap) const
  {
    if (!enforce_slope_check_)
    {
      return true;
    }
    const double dz = std::abs(snap.z[from_idx] - snap.z[to_idx]);
    if (dz > max_step_height_)
    {
      return false;
    }
    const int dx = (to_idx % grid_width_) - (from_idx % grid_width_);
    const int dy = (to_idx / grid_width_) - (from_idx / grid_width_);
    const double step_xy = std::hypot(
        static_cast<double>(dx) * grid_resolution_,
        static_cast<double>(dy) * grid_resolution_);
    if (step_xy > 1e-6 && dz > step_xy * std::tan(max_slope_rad_))
    {
      return false;
    }
    return true;
  }

  bool gridMoveAllowed(int from_idx, int to_idx, const FusedSnapshot& snap) const
  {
    ++dbg_edges_;
    if (!gridTraversable(from_idx, snap) || !gridTraversable(to_idx, snap))
    {
      ++dbg_rej_traversable_;
      return false;
    }
    if (!stepWithinSlope(from_idx, to_idx, snap))
    {
      ++dbg_rej_slope_;
      return false;
    }

    const int fx = from_idx % grid_width_;
    const int fy = from_idx / grid_width_;
    const int dx = (to_idx % grid_width_) - fx;
    const int dy = (to_idx / grid_width_) - fy;
    if (std::abs(dx) == 1 && std::abs(dy) == 1)
    {
      const int side_a = gridIndex(fx + dx, fy);
      const int side_b = gridIndex(fx, fy + dy);
      if (!gridTraversable(side_a, snap) || !gridTraversable(side_b, snap))
      {
        ++dbg_rej_diagonal_;
        return false;
      }
    }
    return true;
  }

  // Walks the grid cells the straight segment passes through (Amanatides-Woo
  // DDA, 4-connected) and validates the full motion model along the line:
  // every cell FREE + sdf >= margin, and every consecutive transition within
  // step height + slope. The slope/step check stops smoothing from cutting a
  // straight shortcut across a terrain discontinuity the grid search routed
  // around. `min_sdf < 0` defaults to line_clearance_.
  bool gridLineTraversableWorld(
      const geometry_msgs::msg::Point& from,
      const geometry_msgs::msg::Point& to,
      const FusedSnapshot& snap,
      double min_sdf = -1.0) const
  {
    const double margin = min_sdf < 0.0 ? line_clearance_ : min_sdf;

    auto cell_clear = [&](int idx) {
      return snap.state[idx] == CellState::FREE && snap.sdf[idx] >= margin;
    };

    const double fx = (from.x - grid_origin_x_) / grid_resolution_;
    const double fy = (from.y - grid_origin_y_) / grid_resolution_;
    const double tx = (to.x - grid_origin_x_) / grid_resolution_;
    const double ty = (to.y - grid_origin_y_) / grid_resolution_;

    int cx = static_cast<int>(std::floor(fx));
    int cy = static_cast<int>(std::floor(fy));
    const int ex = static_cast<int>(std::floor(tx));
    const int ey = static_cast<int>(std::floor(ty));

    if (!inGrid(cx, cy) || !cell_clear(gridIndex(cx, cy)))
    {
      return false;
    }
    if (!inGrid(ex, ey))
    {
      return false;
    }

    const double dx = tx - fx;
    const double dy = ty - fy;
    const int step_x = dx > 0.0 ? 1 : (dx < 0.0 ? -1 : 0);
    const int step_y = dy > 0.0 ? 1 : (dy < 0.0 ? -1 : 0);

    constexpr double kInf = std::numeric_limits<double>::infinity();
    const double inv_dx = step_x != 0 ? 1.0 / std::abs(dx) : kInf;
    const double inv_dy = step_y != 0 ? 1.0 / std::abs(dy) : kInf;
    double t_max_x = step_x > 0 ? (static_cast<double>(cx + 1) - fx) * inv_dx
                   : step_x < 0 ? (fx - static_cast<double>(cx)) * inv_dx
                                : kInf;
    double t_max_y = step_y > 0 ? (static_cast<double>(cy + 1) - fy) * inv_dy
                   : step_y < 0 ? (fy - static_cast<double>(cy)) * inv_dy
                                : kInf;
    const double t_delta_x = inv_dx;
    const double t_delta_y = inv_dy;

    int prev_idx = gridIndex(cx, cy);
    const int max_iter = grid_width_ + grid_height_ + 4;
    for (int iter = 0; (cx != ex || cy != ey) && iter < max_iter; ++iter)
    {
      if (t_max_x < t_max_y)
      {
        cx += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cy += step_y;
        t_max_y += t_delta_y;
      }
      if (!inGrid(cx, cy))
      {
        return false;
      }
      const int idx = gridIndex(cx, cy);
      if (!cell_clear(idx) || !stepWithinSlope(prev_idx, idx, snap))
      {
        return false;
      }
      prev_idx = idx;
    }
    return true;
  }

  // Expanding-ring snap. Accept the first `max_candidates` FREE cells with
  // sdf >= sdf_floor that have a clear line-of-sight to the query point.
  std::vector<GridCandidate> gridSnapCandidates(
      const geometry_msgs::msg::Point& point,
      double radius,
      double sdf_floor,
      std::size_t max_candidates,
      const FusedSnapshot& snap) const
  {
    int center_x = static_cast<int>(std::floor((point.x - grid_origin_x_) / grid_resolution_));
    int center_y = static_cast<int>(std::floor((point.y - grid_origin_y_) / grid_resolution_));
    center_x = std::max(0, std::min(grid_width_ - 1, center_x));
    center_y = std::max(0, std::min(grid_height_ - 1, center_y));

    const int max_ring = static_cast<int>(std::ceil(radius / grid_resolution_));
    std::vector<GridCandidate> candidates;
    candidates.reserve(max_candidates ? max_candidates : 16);

    auto try_cell = [&](int gx, int gy) {
      if (!inGrid(gx, gy))
      {
        return;
      }
      const int idx = gridIndex(gx, gy);
      if (snap.state[idx] != CellState::FREE || snap.sdf[idx] < sdf_floor)
      {
        return;
      }
      const geometry_msgs::msg::Point cell_pt = gridCenter(idx);
      const double dist = pointDistXY(point, cell_pt);
      if (dist > radius)
      {
        return;
      }
      if (!gridLineTraversableWorld(point, cell_pt, snap))
      {
        return;
      }
      candidates.push_back(GridCandidate{idx, dist});
    };

    try_cell(center_x, center_y);
    for (int ring = 1; ring <= max_ring; ++ring)
    {
      const int x_min = center_x - ring;
      const int x_max = center_x + ring;
      const int y_min = center_y - ring;
      const int y_max = center_y + ring;
      for (int gx = x_min; gx <= x_max; ++gx)
      {
        try_cell(gx, y_min);
        try_cell(gx, y_max);
      }
      for (int gy = y_min + 1; gy <= y_max - 1; ++gy)
      {
        try_cell(x_min, gy);
        try_cell(x_max, gy);
      }
      if (max_candidates > 0 && candidates.size() >= max_candidates)
      {
        break;
      }
    }

    std::sort(
        candidates.begin(), candidates.end(),
        [](const GridCandidate& a, const GridCandidate& b) {
          return a.distance < b.distance;
        });
    if (max_candidates > 0 && candidates.size() > max_candidates)
    {
      candidates.resize(max_candidates);
    }
    return candidates;
  }

  // Multi-source, multi-goal A*. Sources are the start snap candidates (seeded
  // with their snap distance as g); goals are the goal snap candidates (snap
  // distance added when finalizing). The heuristic toward `goal_point` is
  // admissible (step costs >= straight-line XY distance, and every goal
  // candidate is within goal_snap_radius_ of goal_point). Early-terminates once
  // the cheapest open node can't beat the best goal found.
  bool searchGridToCandidates(
      const std::vector<GridCandidate>& start_candidates,
      const std::vector<GridCandidate>& goal_candidates,
      const geometry_msgs::msg::Point& goal_point,
      const FusedSnapshot& snap,
      std::vector<int>& cell_path,
      GridCandidate& selected_start,
      GridCandidate& selected_goal)
  {
    dbg_seeded_ = 0;
    dbg_expanded_ = 0;
    dbg_goals_traversable_ = 0;
    dbg_edges_ = 0;
    dbg_rej_traversable_ = 0;
    dbg_rej_slope_ = 0;
    dbg_rej_diagonal_ = 0;
    if (start_candidates.empty() || goal_candidates.empty())
    {
      return false;
    }

    const int cell_count = static_cast<int>(grid_width_) * grid_height_;
    std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::vector<int> source(cell_count, -1);
    std::vector<bool> closed(cell_count, false);

    // Goal candidates are capped (<=32), so a small sorted lookup beats two
    // grid-sized arrays allocated every plan.
    struct GoalEntry { int cell; double snap; int cand; };
    std::vector<GoalEntry> goals;
    goals.reserve(goal_candidates.size());
    for (std::size_t i = 0; i < goal_candidates.size(); ++i)
    {
      const GridCandidate& c = goal_candidates[i];
      if (gridTraversable(c.index, snap))
      {
        goals.push_back(GoalEntry{c.index, c.distance, static_cast<int>(i)});
      }
    }
    dbg_goals_traversable_ = goals.size();
    if (goals.empty())
    {
      return false;
    }
    std::sort(goals.begin(), goals.end(),
              [](const GoalEntry& a, const GoalEntry& b) { return a.cell < b.cell; });
    auto goal_at = [&](int cell) -> const GoalEntry* {
      auto it = std::lower_bound(
          goals.begin(), goals.end(), cell,
          [](const GoalEntry& e, int c) { return e.cell < c; });
      return (it != goals.end() && it->cell == cell) ? &(*it) : nullptr;
    };

    auto heuristic = [&](int idx) {
      const double d = pointDistXY(gridCenter(idx), goal_point);
      return std::max(0.0, d - goal_snap_radius_);
    };

    typedef std::pair<double, int> QueueItem;  // (f = g + h, cell)
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    for (std::size_t i = 0; i < start_candidates.size(); ++i)
    {
      const GridCandidate& candidate = start_candidates[i];
      if (!gridTraversable(candidate.index, snap) || candidate.distance >= g_score[candidate.index])
      {
        continue;
      }
      g_score[candidate.index] = candidate.distance;
      source[candidate.index] = static_cast<int>(i);
      open.push(QueueItem(candidate.distance + heuristic(candidate.index), candidate.index));
      ++dbg_seeded_;
    }

    const int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    double best_total = std::numeric_limits<double>::infinity();
    int best_goal_cell = -1;
    int best_goal_cand = -1;
    int since_stop_check = 0;

    while (!open.empty())
    {
      // Stay responsive to shutdown: bail out of a long/flooding search
      // when the node is being destroyed.
      if (++since_stop_check >= 4096)
      {
        since_stop_check = 0;
        if (stop_requested_.load())
        {
          return false;
        }
      }

      const double f_curr = open.top().first;
      const int current = open.top().second;
      open.pop();
      if (f_curr >= best_total)
      {
        break;
      }
      if (closed[current])
      {
        continue;
      }
      closed[current] = true;
      ++dbg_expanded_;

      if (const GoalEntry* ge = goal_at(current))
      {
        const double total = g_score[current] + ge->snap;
        if (total < best_total)
        {
          best_total = total;
          best_goal_cell = current;
          best_goal_cand = ge->cand;
        }
      }

      const int cx = current % grid_width_;
      const int cy = current / grid_width_;
      for (const auto& dir : dirs)
      {
        const int nx = cx + dir[0];
        const int ny = cy + dir[1];
        if (!inGrid(nx, ny))
        {
          continue;
        }
        const int next = gridIndex(nx, ny);
        if (closed[next] || !gridMoveAllowed(current, next, snap))
        {
          continue;
        }

        const double step_xy = std::hypot(
            static_cast<double>(dir[0]) * grid_resolution_,
            static_cast<double>(dir[1]) * grid_resolution_);
        const double tentative = g_score[current] + step_xy;
        if (tentative < g_score[next])
        {
          parent[next] = current;
          source[next] = source[current];
          g_score[next] = tentative;
          open.push(QueueItem(tentative + heuristic(next), next));
        }
      }
    }

    if (best_goal_cell < 0 || best_goal_cand < 0)
    {
      return false;
    }

    selected_goal = goal_candidates[best_goal_cand];
    const int selected_source = source[selected_goal.index];
    if (selected_source < 0 || selected_source >= static_cast<int>(start_candidates.size()))
    {
      return false;
    }
    selected_start = start_candidates[selected_source];

    cell_path.clear();
    int current = selected_goal.index;
    while (current >= 0)
    {
      cell_path.push_back(current);
      if (current == selected_start.index)
      {
        break;
      }
      current = parent[current];
    }
    if (cell_path.empty() || cell_path.back() != selected_start.index)
    {
      return false;
    }
    std::reverse(cell_path.begin(), cell_path.end());
    return true;
  }

  bool planGridRoute(
      const geometry_msgs::msg::Point& start,
      const geometry_msgs::msg::Point& requested_goal,
      std::vector<geometry_msgs::msg::Point>& route,
      std::string& error)
  {
    auto snap_ptr = getFusedSnapshot();
    if (!snap_ptr)
    {
      error = "Fused snapshot unavailable (grid not yet allocated).";
      return false;
    }
    const FusedSnapshot& snap = *snap_ptr;

    // Start may be wedged near an obstacle (robot footprint > clearance).
    // Permit a lower SDF floor for snapping; the line-of-sight check still
    // requires line_clearance_ along the connector.
    const double start_sdf_floor = std::max(0.5 * grid_resolution_,
                                            std::min(line_clearance_, clearance_radius_));
    std::vector<GridCandidate> start_candidates =
        gridSnapCandidates(start, start_snap_radius_, start_sdf_floor, 16, snap);
    std::vector<GridCandidate> goal_candidates =
        gridSnapCandidates(requested_goal, goal_snap_radius_, clearance_radius_, 32, snap);

    if (start_candidates.empty())
    {
      // Why couldn't the robot anchor its start? Report the robot cell's own
      // state + sdf (= distance to the nearest BLOCKED cell), the snap floor it
      // needed, and how much FREE space near the robot meets that floor. If the
      // robot cell isn't FREE → it's marked occupied (self-return / phantom);
      // if its sdf < floor → a BLOCKED cell sits within sdf metres of the robot.
      int rgx = 0;
      int rgy = 0;
      const char* robot_state = "off-grid";
      double robot_sdf = -1.0;
      std::size_t free_meeting_floor = 0;
      if (worldToGrid(start.x, start.y, rgx, rgy))
      {
        const int ridx = gridIndex(rgx, rgy);
        robot_sdf = snap.sdf[ridx];
        robot_state = snap.state[ridx] == CellState::FREE ? "FREE"
                    : snap.state[ridx] == CellState::BLOCKED ? "BLOCKED" : "UNKNOWN";
        const int r = static_cast<int>(std::ceil(start_snap_radius_ / grid_resolution_));
        for (int dy = -r; dy <= r; ++dy)
        {
          for (int dx = -r; dx <= r; ++dx)
          {
            const int gx = rgx + dx;
            const int gy = rgy + dy;
            if (!inGrid(gx, gy)) continue;
            const int idx = gridIndex(gx, gy);
            if (snap.state[idx] == CellState::FREE && snap.sdf[idx] >= start_sdf_floor)
            {
              ++free_meeting_floor;
            }
          }
        }
      }
      RCLCPP_WARN(
          get_logger(),
          "No start anchor | robot=(%.2f,%.2f) cell=%s sdf=%.2f(=dist to nearest BLOCKED) | "
          "start_floor=%.2f clearance=%.2f res=%.3f | FREE cells within %.1fm meeting floor=%zu",
          start.x, start.y, robot_state, robot_sdf, start_sdf_floor, clearance_radius_,
          grid_resolution_, start_snap_radius_, free_meeting_floor);
      error = "No FREE 2D grid cell with a clear connector near current pose.";
      return false;
    }
    if (goal_candidates.empty())
    {
      error = "No FREE 2D grid cell within clearance near requested goal.";
      return false;
    }

    std::vector<int> cell_path;
    GridCandidate selected_start;
    GridCandidate selected_goal;
    if (!searchGridToCandidates(start_candidates, goal_candidates, requested_goal, snap,
                                cell_path, selected_start, selected_goal))
    {
      // Diagnose why: how much of the FREE space is actually traversable
      // (FREE && sdf >= clearance). If traversable << free (or max_free_sdf <
      // clearance), the clearance requirement is the limiter — the corridor is
      // pinched by nearby obstacles/unknown. Otherwise it's connectivity
      // (FREE fragmented by UNKNOWN gaps) or the slope/step check.
      std::size_t n_free = 0;
      std::size_t n_trav = 0;
      std::size_t n_blocked = 0;
      std::size_t n_unknown = 0;
      double max_free_sdf = 0.0;
      for (std::size_t i = 0; i < snap.state.size(); ++i)
      {
        const CellState s = snap.state[i];
        if (s == CellState::BLOCKED) { ++n_blocked; continue; }
        if (s == CellState::UNKNOWN) { ++n_unknown; continue; }
        ++n_free;
        max_free_sdf = std::max(max_free_sdf, snap.sdf[i]);
        if (snap.sdf[i] >= clearance_radius_) ++n_trav;
      }
      const double start_sdf = start_candidates.empty() ? -1.0 : snap.sdf[start_candidates.front().index];
      const double goal_sdf = goal_candidates.empty() ? -1.0 : snap.sdf[goal_candidates.front().index];

      // Decisive check: is the FREE space (ignoring clearance) connected from the
      // start to any goal cell? YES → the barrier is purely the clearance margin
      // (a sub-clearance pinch). NO → the FREE region itself is split by
      // UNKNOWN/BLOCKED cells between start and goal.
      bool free_connected = false;
      std::size_t free_comp = 0;
      {
        std::vector<char> vis(snap.state.size(), 0);
        std::vector<char> is_goal(snap.state.size(), 0);
        for (const auto& gc : goal_candidates)
        {
          if (gc.index >= 0 && gc.index < static_cast<int>(is_goal.size())) is_goal[gc.index] = 1;
        }
        std::vector<int> st;
        const int s0 = start_candidates.front().index;
        if (s0 >= 0 && s0 < static_cast<int>(vis.size()) && snap.state[s0] == CellState::FREE)
        {
          vis[s0] = 1;
          st.push_back(s0);
        }
        while (!st.empty())
        {
          const int c = st.back();
          st.pop_back();
          ++free_comp;
          if (is_goal[c]) free_connected = true;
          const int cx = c % grid_width_;
          const int cy = c / grid_width_;
          for (int dy = -1; dy <= 1; ++dy)
          {
            for (int dx = -1; dx <= 1; ++dx)
            {
              if (dx == 0 && dy == 0) continue;
              const int nx = cx + dx;
              const int ny = cy + dy;
              if (!inGrid(nx, ny)) continue;
              const int nidx = gridIndex(nx, ny);
              if (vis[nidx] || snap.state[nidx] != CellState::FREE) continue;
              vis[nidx] = 1;
              st.push_back(nidx);
            }
          }
        }
      }

      RCLCPP_WARN(
          get_logger(),
          "No path | start=(%.2f,%.2f) goal=(%.2f,%.2f) res=%.3f clearance=%.2f | "
          "FREE=%zu trav(sdf>=clr)=%zu maxFREEsdf=%.2f BLOCKED=%zu UNKNOWN=%zu | "
          "start_cand=%zu(sdf=%.2f) goal_cand=%zu(sdf=%.2f) | "
          "A*: seeded=%zu goals_trav=%zu expanded=%zu | rejects: trav=%zu slope=%zu diag=%zu | "
          "FREE-connected(start->goal)=%s freeComp=%zu",
          start.x, start.y, requested_goal.x, requested_goal.y, grid_resolution_, clearance_radius_,
          n_free, n_trav, max_free_sdf, n_blocked, n_unknown,
          start_candidates.size(), start_sdf, goal_candidates.size(), goal_sdf,
          dbg_seeded_, dbg_goals_traversable_, dbg_expanded_,
          dbg_rej_traversable_, dbg_rej_slope_, dbg_rej_diagonal_,
          free_connected ? "YES" : "NO", free_comp);
      error = "No reachable 2D grid path from current pose to requested goal.";
      return false;
    }
    const int goal_idx = selected_goal.index;
    const double start_snap = selected_start.distance;
    const double goal_snap = selected_goal.distance;

    route.clear();
    route.push_back(start);
    compactTurns(cell_path, start.z, route);

    geometry_msgs::msg::Point final_point = gridCenter(goal_idx);
    final_point.z = start.z;
    if (gridLineTraversableWorld(final_point, requested_goal, snap))
    {
      final_point = requested_goal;
      final_point.z = start.z;
    }
    else
    {
      RCLCPP_INFO(
          get_logger(),
          "Requested goal snapped to traversable 2D cell x=%.2f y=%.2f (snap=%.2fm).",
          final_point.x, final_point.y, goal_snap);
    }
    if (route.empty() || pointDistXY(route.back(), final_point) > 1e-3)
    {
      route.push_back(final_point);
    }

    const std::size_t pre_smooth = route.size();
    if (smoothing_enabled_)
    {
      route = smoothRoute(route, snap);
    }

    RCLCPP_INFO(
        get_logger(),
        "2D grid route planned: %zu waypoints (pre-smooth=%zu), cells=%zu, "
        "start_snap=%.2fm, goal_snap=%.2fm.",
        route.size(), pre_smooth, cell_path.size(), start_snap, goal_snap);
    return route.size() >= 2;
  }

  // Appends one waypoint per direction change in the cell path to `route`
  // (drops collinear interior cells). All waypoints take `z`.
  void compactTurns(
      const std::vector<int>& cell_path,
      double z,
      std::vector<geometry_msgs::msg::Point>& route) const
  {
    int last_dx = 0;
    int last_dy = 0;
    for (std::size_t i = 0; i < cell_path.size(); ++i)
    {
      bool keep = i == 0 || i + 1 == cell_path.size();
      if (i > 0 && i + 1 < cell_path.size())
      {
        const int curr = cell_path[i];
        const int next = cell_path[i + 1];
        const int curr_dx = (next % grid_width_) - (curr % grid_width_);
        const int curr_dy = (next / grid_width_) - (curr / grid_width_);
        if (i == 1)
        {
          const int prev = cell_path[i - 1];
          last_dx = (curr % grid_width_) - (prev % grid_width_);
          last_dy = (curr / grid_width_) - (prev / grid_width_);
        }
        if (curr_dx != last_dx || curr_dy != last_dy)
        {
          keep = true;
        }
        last_dx = curr_dx;
        last_dy = curr_dy;
      }
      if (keep)
      {
        geometry_msgs::msg::Point point = gridCenter(cell_path[i]);
        point.z = z;
        if (route.empty() || pointDistXY(route.back(), point) > 1e-3)
        {
          route.push_back(point);
        }
      }
    }
  }

  // Greedy line-of-sight compaction at clearance_radius_ (the same margin the
  // grid search enforces — not the relaxed line_clearance_), so a smoothed
  // shortcut can't bow through a cell A* itself refused. Start/goal connectors
  // that use the lower margin are preserved automatically because a merge
  // across a sub-clearance connector fails this stricter check.
  std::vector<geometry_msgs::msg::Point> smoothRoute(
      const std::vector<geometry_msgs::msg::Point>& in,
      const FusedSnapshot& snap) const
  {
    if (in.size() <= 2)
    {
      return in;
    }
    std::vector<geometry_msgs::msg::Point> out;
    out.reserve(in.size());
    out.push_back(in.front());
    std::size_t anchor = 0;
    while (anchor + 1 < in.size())
    {
      std::size_t lookahead = anchor + 1;
      while (lookahead + 1 < in.size() &&
             gridLineTraversableWorld(in[anchor], in[lookahead + 1], snap, clearance_radius_))
      {
        ++lookahead;
      }
      out.push_back(in[lookahead]);
      anchor = lookahead;
    }
    return out;
  }

  geometry_msgs::msg::Point selectWaypoint(
      const geometry_msgs::msg::Point& current,
      const std::vector<geometry_msgs::msg::Point>& route,
      std::size_t& route_index)
  {
    while (route_index + 1 < route.size() &&
           pointDistXY(current, route[route_index]) <= waypoint_reach_radius_)
    {
      route_index++;
    }

    double remaining = lookahead_distance_;
    geometry_msgs::msg::Point segment_start = current;
    for (std::size_t i = route_index; i < route.size(); ++i)
    {
      const double segment_length = pointDistXY(segment_start, route[i]);
      if (segment_length >= remaining && segment_length > 1e-3)
      {
        return lerpPoint(segment_start, route[i], remaining / segment_length);
      }
      remaining -= segment_length;
      segment_start = route[i];
    }
    return route.back();
  }

  void publishWaypoint(const geometry_msgs::msg::Point& waypoint)
  {
    geometry_msgs::msg::PointStamped msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    msg.point = waypoint;
    waypoint_pub_->publish(msg);
  }

  void publishFeedback(
      const std::shared_ptr<GoalHandleNavigateToPose>& goal_handle,
      const geometry_msgs::msg::PoseStamped& current_pose,
      const std::vector<geometry_msgs::msg::Point>& route,
      std::size_t route_index,
      const rclcpp::Time& start_time)
  {
    auto feedback = std::make_shared<NavigateToPose::Feedback>();
    feedback->current_pose = current_pose;
    feedback->distance_remaining = remainingRouteDistance(current_pose.pose.position, route, route_index);
    const int64_t elapsed_ns = (now() - start_time).nanoseconds();
    feedback->navigation_time.sec = static_cast<int32_t>(elapsed_ns / 1000000000LL);
    feedback->navigation_time.nanosec = static_cast<uint32_t>(elapsed_ns % 1000000000LL);
    goal_handle->publish_feedback(feedback);
  }

  double remainingRouteDistance(
      const geometry_msgs::msg::Point& current,
      const std::vector<geometry_msgs::msg::Point>& route,
      std::size_t route_index) const
  {
    if (route.empty())
    {
      return 0.0;
    }
    route_index = std::min(route_index, route.size() - 1);
    double distance = pointDistXY(current, route[route_index]);
    for (std::size_t i = route_index; i + 1 < route.size(); ++i)
    {
      distance += pointDistXY(route[i], route[i + 1]);
    }
    return distance;
  }

  void publishRoute(const std::vector<geometry_msgs::msg::Point>& route)
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = now();
    path.poses.reserve(route.size());
    for (const auto& point : route)
    {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = point;
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(pose);
    }
    path_pub_->publish(path);

    visualization_msgs::msg::Marker marker;
    marker.header = path.header;
    marker.ns = "pcd_grid_route";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.08;
    marker.color.r = 0.0f;
    marker.color.g = 0.65f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    marker.points = route;

    visualization_msgs::msg::MarkerArray array;
    array.markers.push_back(marker);
    path_marker_pub_->publish(array);
  }

  // Publishes the live 2D world map: 0 FREE, 100 BLOCKED, -1 UNKNOWN per the
  // OccupancyGrid spec. Robot-specific clearance does NOT appear here — other
  // consumers can subscribe and apply their own footprint. Executor thread.
  void publishGridMap()
  {
    if (grid_.empty())
    {
      return;
    }

    nav_msgs::msg::OccupancyGrid map;
    map.header.frame_id = map_frame_;
    map.header.stamp = now();
    map.info.resolution = static_cast<float>(grid_resolution_);
    map.info.width = static_cast<std::uint32_t>(grid_width_);
    map.info.height = static_cast<std::uint32_t>(grid_height_);
    map.info.origin.position.x = grid_origin_x_;
    map.info.origin.position.y = grid_origin_y_;
    map.info.origin.position.z = 0.0;
    map.info.origin.orientation.w = 1.0;
    map.data.resize(grid_.size());

    for (std::size_t i = 0; i < grid_.size(); ++i)
    {
      switch (filtered_state_[i])
      {
        case CellState::FREE:    map.data[i] = 0;   break;
        case CellState::BLOCKED: map.data[i] = 100; break;
        case CellState::UNKNOWN: map.data[i] = -1;  break;
      }
    }

    grid_map_pub_->publish(map);
  }

  void publishGridMarkers()
  {
    if (grid_.empty())
    {
      return;
    }

    visualization_msgs::msg::MarkerArray array;

    auto make_marker = [&](const char* ns, int id, float r, float g, float b,
                           float a, double scale_z) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = now();
      m.ns = ns;
      m.id = id;
      m.type = visualization_msgs::msg::Marker::CUBE_LIST;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      m.scale.x = grid_resolution_;
      m.scale.y = grid_resolution_;
      m.scale.z = scale_z;
      m.color.r = r;
      m.color.g = g;
      m.color.b = b;
      m.color.a = a;
      return m;
    };

    auto traversable = make_marker("pcd_grid_free",     0, 0.10f, 0.80f, 0.55f, 0.28f, 0.03);
    auto near_obs    = make_marker("pcd_grid_near_obs", 1, 1.00f, 0.80f, 0.10f, 0.32f, 0.04);
    auto blocked     = make_marker("pcd_grid_blocked",  2, 1.00f, 0.25f, 0.05f, 0.35f, 0.05);

    for (std::size_t i = 0; i < grid_.size(); ++i)
    {
      const GridCell& cell = grid_[i];
      geometry_msgs::msg::Point point = gridCenter(static_cast<int>(i));
      point.z = cell.z;
      switch (filtered_state_[i])
      {
        case CellState::BLOCKED:
          point.z += 0.04;
          blocked.points.push_back(point);
          break;
        case CellState::FREE:
          if (cell.sdf >= clearance_radius_)
          {
            point.z += 0.02;
            traversable.points.push_back(point);
          }
          else
          {
            point.z += 0.03;
            near_obs.points.push_back(point);
          }
          break;
        case CellState::UNKNOWN:
          break;  // RViz renders these from /pcd_2d_map
      }
    }

    array.markers.push_back(traversable);
    array.markers.push_back(near_obs);
    array.markers.push_back(blocked);
    grid_marker_pub_->publish(array);
  }

  std::string map_frame_;

  double grid_resolution_ = 0.05;
  int grid_max_cells_ = 2000000;

  double live_map_width_m_ = 24.0;
  double live_map_height_m_ = 24.0;
  double live_map_center_x_ = 0.0;
  double live_map_center_y_ = 0.0;
  bool live_map_recenter_on_first_odom_ = true;
  bool live_grid_growth_ = true;
  double live_grid_growth_margin_ = 3.0;
  double live_grid_grow_max_range_ = 20.0;

  double max_slope_deg_ = 25.0;
  double max_slope_rad_ = 25.0 * M_PI / 180.0;
  double max_step_height_ = 0.18;
  double min_clearance_ = 0.25;
  double obstacle_height_ = 0.15;
  double vehicle_height_ = 1.0;          // reserved; unused in v1
  double vehicle_length_ = 0.7;
  double vehicle_width_ = 0.3;
  double clearance_radius_ = 0.35;
  double line_clearance_ = -1.0;         // -1 → derived from clearance_radius_

  double start_snap_radius_ = 6.0;
  double goal_snap_radius_ = 2.0;
  double lookahead_distance_ = 2.5;
  double waypoint_reach_radius_ = 0.6;
  double goal_tolerance_ = 0.35;
  double waypoint_rate_ = 5.0;
  bool smoothing_enabled_ = true;
  bool publish_debug_markers_ = false;

  // Live occupancy mapping.
  MapSource map_source_ = MapSource::SLICE;
  std::string live_topic_ = "/registered_scan";
  double mapping_rate_hz_ = 2.0;
  double blockage_ceiling_ = 1.6;       // TERRAIN mode only

  // SLICE mode lidar-plane band (robot-relative).
  double slice_band_half_ = 0.10;
  double slice_band_offset_ = 0.0;

  // Spatial cleanup filter.
  bool spatial_filter_enabled_ = true;
  int spatial_min_component_size_ = 3;
  bool spatial_opening_ = false;

  // Robot free-space carving (blind-cone flood-fill).
  bool robot_free_carve_ = true;
  double robot_free_radius_ = 5.0;
  std::vector<std::uint32_t> carve_visit_gen_;   // per-cell BFS visited stamp
  std::uint32_t carve_gen_ = 0;

  // Ray-trace clearing (SLICE mode dynamic-obstacle decay).
  bool raytrace_clearing_ = true;
  double raytrace_min_range_ = 0.3;
  double raytrace_max_range_ = 15.0;

  // Clearance/SDF: inflate from UNKNOWN too (conservative) vs BLOCKED only.
  bool clearance_inflate_unknown_ = false;
  // Slope/step gate (needs reliable per-cell elevation; off for slice band).
  bool enforce_slope_check_ = false;

  // A* diagnostics — reset and filled per search (one navigation at a time),
  // read by planGridRoute's failure log. mutable: written in const search/edge
  // methods. Not thread-shared (single active search).
  mutable std::size_t dbg_seeded_ = 0;            // start sources pushed into open
  mutable std::size_t dbg_expanded_ = 0;          // cells closed by A*
  mutable std::size_t dbg_goals_traversable_ = 0; // goal candidates that are traversable
  mutable std::size_t dbg_edges_ = 0;             // edge checks in gridMoveAllowed
  mutable std::size_t dbg_rej_traversable_ = 0;   // edges rejected: cell not FREE or sdf<clearance
  mutable std::size_t dbg_rej_slope_ = 0;         // edges rejected: slope/step gate
  mutable std::size_t dbg_rej_diagonal_ = 0;      // edges rejected: diagonal corner blocked
  std::vector<std::uint32_t> ray_stamp_;         // per-scan endpoint/cleared stamp
  std::uint32_t ray_gen_ = 0;
  std::vector<int> scan_floor_cells_;            // this scan's floor-return cells (ray-trace targets)

  // Diagnostics (executor thread only).
  bool cloud_frame_logged_ = false;
  std::size_t last_cloud_size_ = 0;
  std::size_t last_cloud_in_grid_ = 0;
  double live_l_hit_ = 0.85;
  double live_l_miss_ = 0.40;
  double live_l_min_ = -2.0;
  double live_l_max_ = 4.0;
  double live_l_occ_thr_ = 1.5;
  double live_l_free_thr_ = -0.4;

  // Replanning.
  double replan_period_sec_ = 0.5;
  double replan_horizon_m_ = 4.0;
  int max_replan_attempts_ = 20;

  bool map_ready_ = false;
  std::atomic_bool has_odom_{false};
  std::atomic_bool executing_{false};
  std::atomic_bool stop_requested_{false};
  std::atomic_bool grid_allocated_{false};

  // One joinable worker, never detached. Replaced before each run; joined
  // in the destructor.
  std::thread worker_;
  std::mutex worker_mutex_;

  // Current best map — owned by the executor thread (mapping tick + publishers).
  std::vector<GridCell> grid_;
  // Spatially-filtered view of grid_[*].state, rebuilt each mapping tick and
  // consumed by the SDF, the published map, and the planner snapshot. grid_ is
  // never modified by the spatial filter. Executor thread only.
  std::vector<CellState> filtered_state_;
  int grid_width_ = 0;
  int grid_height_ = 0;
  double grid_origin_x_ = 0.0;
  double grid_origin_y_ = 0.0;

  // Log-odds accumulators + touched-cell bookkeeping. Guarded by accum_mutex_
  // (cloudCallback tallies, mapping tick collapses).
  std::vector<LiveAccum> accum_;
  std::vector<int> touched_;
  std::vector<std::uint8_t> in_touched_;
  std::mutex accum_mutex_;

  // Read-only snapshot the planner consumes. Guarded by fused_mutex_ for the
  // swap; readers retain a shared_ptr and read lock-free.
  std::shared_ptr<const FusedSnapshot> fused_snapshot_;
  mutable std::mutex fused_mutex_;

  std::mutex odom_mutex_;
  geometry_msgs::msg::Pose latest_pose_;
  rclcpp::Time latest_odom_stamp_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp::TimerBase::SharedPtr mapping_timer_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdGridPlanner>());
  rclcpp::shutdown();
  return 0;
}
