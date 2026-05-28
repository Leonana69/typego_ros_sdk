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

// Live-mapping 2D grid planner. The grid is built at runtime by accumulating
// terrain_analysis output (already ground-segmented + ceiling-filtered, in the
// map frame) into a per-cell log-odds occupancy filter, instead of projecting a
// saved PCD. The planning pipeline (SDF, A*, snap, line-of-sight smoothing,
// replan) is unchanged; only the source of the grid differs.
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
    terrain_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        live_topic_, rclcpp::SensorDataQoS(),
        std::bind(&PcdGridPlanner::terrainCallback, this, std::placeholders::_1));

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
        "PCD grid planner (live mapping from %s) ready; action server on "
        "/navigate_to_pose, RViz goals on /goal_pose.",
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
  // tallied by terrainCallback (100 Hz) and collapsed to one log-odds vote
  // per mapping tick (decoupling the filter rate from the sensor rate).
  // logodds + ground_sum/ground_count persist across ticks; the counts reset.
  struct LiveAccum
  {
    float logodds = 0.0f;
    std::uint16_t obs_count = 0;
    std::uint16_t free_count = 0;
    double ground_sum = 0.0;
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

    declare_parameter<double>("grid_resolution", 0.20);
    declare_parameter<int>("grid_max_cells", 2000000);

    // Live grid extent (fixed, recentered on first odom by default).
    declare_parameter<double>("live_map_width_m", 80.0);
    declare_parameter<double>("live_map_height_m", 80.0);
    declare_parameter<double>("live_map_center_x", 0.0);
    declare_parameter<double>("live_map_center_y", 0.0);
    declare_parameter<bool>("live_map_recenter_on_first_odom", true);

    declare_parameter<double>("max_slope_deg", 25.0);
    declare_parameter<double>("max_step_height", 0.18);

    declare_parameter<double>("min_clearance", 0.35);
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
    declare_parameter<std::string>("live_topic", "/terrain_map_ext");
    declare_parameter<double>("mapping_rate_hz", 2.0);
    declare_parameter<double>("blockage_ceiling", 1.6);
    declare_parameter<double>("live_l_hit", 0.85);
    declare_parameter<double>("live_l_miss", 0.40);
    declare_parameter<double>("live_l_min", -2.0);
    declare_parameter<double>("live_l_max", 4.0);
    declare_parameter<double>("live_l_occ_thr", 1.5);
    declare_parameter<double>("live_l_free_thr", -1.0);

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

    get_parameter("live_topic", live_topic_);
    get_parameter("mapping_rate_hz", mapping_rate_hz_);
    get_parameter("blockage_ceiling", blockage_ceiling_);
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

    max_slope_rad_ = max_slope_deg_ * M_PI / 180.0;
    clearance_radius_ = std::max(min_clearance_, 0.5 * std::hypot(vehicle_length_, vehicle_width_));
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

    RCLCPP_INFO(
        get_logger(),
        "Live grid allocated: %dx%d @ %.2fm, extent %.0fx%.0f m, origin (%.2f, %.2f).",
        grid_width_, grid_height_, grid_resolution_,
        live_map_width_m_, live_map_height_m_, grid_origin_x_, grid_origin_y_);
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

  // Rebuilds grid_[*].sdf = meters to the nearest non-FREE cell, via a 2-pass
  // EDT. O(N); ~ms for a 160k-cell grid. Runs on the executor thread (mapping
  // tick) so grid_ is touched single-threaded.
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

    std::vector<double> squared(cells);
    for (std::size_t i = 0; i < cells; ++i)
    {
      squared[i] = grid_[i].state == CellState::FREE ? sentinel : 0.0;
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
      snap->state[i] = grid_[i].state;
      snap->z[i] = grid_[i].z;
      snap->sdf[i] = grid_[i].sdf;
    }
    std::lock_guard<std::mutex> lock(fused_mutex_);
    fused_snapshot_ = std::move(snap);
  }

  // 100 Hz: tally raw per-cell evidence. No log-odds here — that's collapsed to
  // one vote per cell at the mapping tick so the filter rate is decoupled from
  // the sensor rate. Touched cells are deduped into touched_ for the tick.
  void terrainCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!map_ready_)
    {
      return;
    }

    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty())
    {
      return;
    }

    std::lock_guard<std::mutex> lock(accum_mutex_);
    for (const auto& point : cloud.points)
    {
      const float intensity = point.intensity;  // height above ground, meters
      if (!std::isfinite(intensity))
      {
        continue;
      }
      bool obstacle;
      if (intensity < obstacle_height_)
      {
        obstacle = false;  // ground / free evidence
      }
      else if (intensity <= blockage_ceiling_)
      {
        obstacle = true;   // obstacle or no-data blockage evidence
      }
      else
      {
        continue;          // overhead / outlier — ignore
      }

      int gx = 0;
      int gy = 0;
      if (!worldToGrid(point.x, point.y, gx, gy))
      {
        continue;          // outside the fixed extent — dropped
      }
      const int idx = gridIndex(gx, gy);
      if (!in_touched_[idx])
      {
        in_touched_[idx] = 1;
        touched_.push_back(idx);
      }
      LiveAccum& a = accum_[idx];
      if (obstacle)
      {
        if (a.obs_count < 65535)
        {
          a.obs_count++;
        }
      }
      else
      {
        if (a.free_count < 65535)
        {
          a.free_count++;
        }
        a.ground_sum += static_cast<double>(point.z) - static_cast<double>(intensity);
        a.ground_count++;
      }
    }
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

    const int gx = idx % grid_width_;
    const int gy = idx / grid_width_;
    if (gx == 0 || gy == 0 || gx == grid_width_ - 1 || gy == grid_height_ - 1)
    {
      next = CellState::UNKNOWN;
    }
    grid_[idx].state = next;

    if (a.ground_count > 0)
    {
      grid_[idx].z = a.ground_sum / static_cast<double>(a.ground_count);
    }
  }

  // Mapping tick: collapse evidence → one log-odds step per touched cell,
  // reclassify, rebuild the SDF, swap the snapshot, and republish the map.
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

    if (updated == 0)
    {
      return;  // nothing changed; skip the EDT + publish
    }

    buildSdf();
    publishSnapshot();
    publishGridMap();
    if (publish_debug_markers_)
    {
      publishGridMarkers();
    }
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
  bool stepWithinSlope(int from_idx, int to_idx, const FusedSnapshot& snap) const
  {
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
    if (!gridTraversable(from_idx, snap) || !gridTraversable(to_idx, snap))
    {
      return false;
    }
    if (!stepWithinSlope(from_idx, to_idx, snap))
    {
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
      switch (grid_[i].state)
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
      switch (cell.state)
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

  double grid_resolution_ = 0.20;
  int grid_max_cells_ = 2000000;

  double live_map_width_m_ = 80.0;
  double live_map_height_m_ = 80.0;
  double live_map_center_x_ = 0.0;
  double live_map_center_y_ = 0.0;
  bool live_map_recenter_on_first_odom_ = true;

  double max_slope_deg_ = 25.0;
  double max_slope_rad_ = 25.0 * M_PI / 180.0;
  double max_step_height_ = 0.18;
  double min_clearance_ = 0.35;
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
  std::string live_topic_ = "/terrain_map_ext";
  double mapping_rate_hz_ = 2.0;
  double blockage_ceiling_ = 1.6;
  double live_l_hit_ = 0.85;
  double live_l_miss_ = 0.40;
  double live_l_min_ = -2.0;
  double live_l_max_ = 4.0;
  double live_l_occ_thr_ = 1.5;
  double live_l_free_thr_ = -1.0;

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
  int grid_width_ = 0;
  int grid_height_ = 0;
  double grid_origin_x_ = 0.0;
  double grid_origin_y_ = 0.0;

  // Log-odds accumulators + touched-cell bookkeeping. Guarded by accum_mutex_
  // (terrainCallback tallies, mapping tick collapses).
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
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_sub_;
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
