#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <fstream>
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
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

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

class PcdGridPlanner : public rclcpp::Node
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

  PcdGridPlanner()
      : Node("pcd_grid_planner"),
        map_cloud_(new pcl::PointCloud<pcl::PointXYZI>())
  {
    readParameters();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/state_estimation", 10,
        std::bind(&PcdGridPlanner::odomCallback, this, std::placeholders::_1));
    goal_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/goal_pose", 1,
        std::bind(&PcdGridPlanner::goalPoseCallback, this, std::placeholders::_1));

    waypoint_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("/way_point", 5);
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
        "/pcd_grid_path", rclcpp::QoS(1).transient_local());
    grid_map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/pcd_2d_map", rclcpp::QoS(1).transient_local());
    grid_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pcd_grid_markers", rclcpp::QoS(1).transient_local());
    path_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/pcd_grid_route", rclcpp::QoS(1).transient_local());

    map_ready_ = loadMap() && buildGridMap2D();
    if (map_ready_)
    {
      publishGridMap();
      publishGridMarkers();
    }

    action_server_ = rclcpp_action::create_server<NavigateToPose>(
        this,
        "navigate_to_pose",
        std::bind(&PcdGridPlanner::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&PcdGridPlanner::handleCancel, this, std::placeholders::_1),
        std::bind(&PcdGridPlanner::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "PCD grid planner action server ready on /navigate_to_pose");
    RCLCPP_INFO(get_logger(), "PCD grid planner also accepts RViz goals on /goal_pose");
  }

private:
  struct GridCell
  {
    int count = 0;
    int low_count = 0;
    int obstacle_count = 0;
    double min_z = std::numeric_limits<double>::infinity();
    double max_z = -std::numeric_limits<double>::infinity();
    double low_sum_z = 0.0;
    double z = 0.0;
    bool raw_free = false;
    bool traversable = false;
    bool inflated = false;
  };

  struct GridCandidate
  {
    int index = -1;
    double distance = 0.0;
  };

  void readParameters()
  {
    declare_parameter<std::string>("map_file", "");
    declare_parameter<std::string>("map_frame", "map");

    declare_parameter<double>("map_voxel_size", 0.08);
    declare_parameter<double>("grid_resolution", 0.20);
    declare_parameter<double>("grid_padding", 0.2);
    declare_parameter<double>("grid_obstacle_inflation_radius", -1.0);
    declare_parameter<int>("grid_max_cells", 2000000);

    declare_parameter<double>("max_slope_deg", 25.0);
    declare_parameter<double>("max_step_height", 0.18);

    declare_parameter<double>("min_clearance", 0.35);
    declare_parameter<double>("obstacle_height", 0.15);
    declare_parameter<double>("vehicle_height", 1.0);
    declare_parameter<double>("vehicle_length", 0.7);
    declare_parameter<double>("vehicle_width", 0.3);

    declare_parameter<double>("start_snap_radius", 6.0);
    declare_parameter<double>("goal_snap_radius", 2.0);
    declare_parameter<double>("lookahead_distance", 2.5);
    declare_parameter<double>("waypoint_reach_radius", 0.6);
    declare_parameter<double>("goal_tolerance", 0.35);
    declare_parameter<double>("waypoint_rate", 5.0);

    get_parameter("map_file", map_file_);
    get_parameter("map_frame", map_frame_);

    get_parameter("map_voxel_size", map_voxel_size_);
    get_parameter("grid_resolution", grid_resolution_);
    get_parameter("grid_padding", grid_padding_);
    get_parameter("grid_obstacle_inflation_radius", grid_obstacle_inflation_radius_);
    get_parameter("grid_max_cells", grid_max_cells_);

    get_parameter("max_slope_deg", max_slope_deg_);
    get_parameter("max_step_height", max_step_height_);

    get_parameter("min_clearance", min_clearance_);
    get_parameter("obstacle_height", obstacle_height_);
    get_parameter("vehicle_height", vehicle_height_);
    get_parameter("vehicle_length", vehicle_length_);
    get_parameter("vehicle_width", vehicle_width_);

    get_parameter("start_snap_radius", start_snap_radius_);
    get_parameter("goal_snap_radius", goal_snap_radius_);
    get_parameter("lookahead_distance", lookahead_distance_);
    get_parameter("waypoint_reach_radius", waypoint_reach_radius_);
    get_parameter("goal_tolerance", goal_tolerance_);
    get_parameter("waypoint_rate", waypoint_rate_);

    max_slope_rad_ = max_slope_deg_ * M_PI / 180.0;
    clearance_radius_ = std::max(min_clearance_, 0.5 * std::hypot(vehicle_length_, vehicle_width_));

    waypoint_rate_ = std::max(0.5, waypoint_rate_);
    grid_resolution_ = std::max(0.05, grid_resolution_);
    grid_padding_ = std::max(0.0, grid_padding_);
    grid_max_cells_ = std::max(1000, grid_max_cells_);
    if (grid_obstacle_inflation_radius_ < 0.0)
    {
      grid_obstacle_inflation_radius_ = clearance_radius_;
    }
  }

  bool loadMap()
  {
    if (map_file_.empty())
    {
      RCLCPP_ERROR(get_logger(), "map_file is empty; PCD grid planner cannot start.");
      return false;
    }
    std::ifstream input(map_file_);
    if (!input.good())
    {
      RCLCPP_ERROR(get_logger(), "Cannot open PCD map file: %s", map_file_.c_str());
      return false;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    if (pcl::io::loadPCDFile<pcl::PointXYZI>(map_file_, *raw_cloud) == -1)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr xyz_cloud(new pcl::PointCloud<pcl::PointXYZ>());
      if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_file_, *xyz_cloud) == -1)
      {
        RCLCPP_ERROR(get_logger(), "Failed to load PCD map file: %s", map_file_.c_str());
        return false;
      }
      raw_cloud->clear();
      raw_cloud->reserve(xyz_cloud->size());
      for (const auto& p : xyz_cloud->points)
      {
        pcl::PointXYZI out;
        out.x = p.x;
        out.y = p.y;
        out.z = p.z;
        out.intensity = 0.0f;
        raw_cloud->push_back(out);
      }
    }

    std::vector<int> valid_indices;
    pcl::removeNaNFromPointCloud(*raw_cloud, *raw_cloud, valid_indices);
    if (raw_cloud->empty())
    {
      RCLCPP_ERROR(get_logger(), "PCD map has no valid points: %s", map_file_.c_str());
      return false;
    }

    if (map_voxel_size_ > 0.0)
    {
      pcl::VoxelGrid<pcl::PointXYZI> filter;
      filter.setLeafSize(
          static_cast<float>(map_voxel_size_),
          static_cast<float>(map_voxel_size_),
          static_cast<float>(map_voxel_size_));
      filter.setInputCloud(raw_cloud);
      filter.filter(*map_cloud_);
    }
    else
    {
      *map_cloud_ = *raw_cloud;
    }

    if (map_cloud_->empty())
    {
      RCLCPP_ERROR(get_logger(), "PCD map became empty after voxel filtering: %s", map_file_.c_str());
      return false;
    }

    RCLCPP_INFO(
        get_logger(),
        "Loaded PCD map %s with %zu points after filtering.",
        map_file_.c_str(), map_cloud_->size());
    return true;
  }

  bool buildGridMap2D()
  {
    if (map_cloud_->empty())
    {
      RCLCPP_ERROR(get_logger(), "Cannot build 2D grid from an empty PCD map.");
      return false;
    }

    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const auto& point : map_cloud_->points)
    {
      min_x = std::min(min_x, static_cast<double>(point.x));
      min_y = std::min(min_y, static_cast<double>(point.y));
      max_x = std::max(max_x, static_cast<double>(point.x));
      max_y = std::max(max_y, static_cast<double>(point.y));
    }

    grid_origin_x_ = min_x - grid_padding_;
    grid_origin_y_ = min_y - grid_padding_;
    grid_width_ = static_cast<int>(std::ceil((max_x - min_x + 2.0 * grid_padding_) / grid_resolution_)) + 1;
    grid_height_ = static_cast<int>(std::ceil((max_y - min_y + 2.0 * grid_padding_) / grid_resolution_)) + 1;
    const long long cell_count = static_cast<long long>(grid_width_) * static_cast<long long>(grid_height_);
    if (grid_width_ <= 0 || grid_height_ <= 0 || cell_count <= 0 || cell_count > grid_max_cells_)
    {
      RCLCPP_ERROR(
          get_logger(),
          "2D grid size is invalid or too large: width=%d height=%d cells=%lld max=%d.",
          grid_width_, grid_height_, cell_count, grid_max_cells_);
      return false;
    }

    grid_.clear();
    grid_.resize(static_cast<std::size_t>(cell_count));

    for (const auto& point : map_cloud_->points)
    {
      int gx = 0;
      int gy = 0;
      if (!worldToGrid(point.x, point.y, gx, gy))
      {
        continue;
      }
      GridCell& cell = grid_[gridIndex(gx, gy)];
      cell.count++;
      cell.min_z = std::min(cell.min_z, static_cast<double>(point.z));
      cell.max_z = std::max(cell.max_z, static_cast<double>(point.z));
    }

    for (const auto& point : map_cloud_->points)
    {
      int gx = 0;
      int gy = 0;
      if (!worldToGrid(point.x, point.y, gx, gy))
      {
        continue;
      }
      GridCell& cell = grid_[gridIndex(gx, gy)];
      if (cell.count == 0 || !std::isfinite(cell.min_z))
      {
        continue;
      }

      const double dz = point.z - cell.min_z;
      if (dz <= max_step_height_)
      {
        cell.low_count++;
        cell.low_sum_z += point.z;
      }
      if (dz > obstacle_height_ && dz < vehicle_height_)
      {
        cell.obstacle_count++;
      }
    }

    std::vector<int> obstacle_cells;
    obstacle_cells.reserve(grid_.size() / 8);
    std::size_t raw_free_count = 0;
    for (std::size_t i = 0; i < grid_.size(); ++i)
    {
      GridCell& cell = grid_[i];
      if (cell.low_count > 0)
      {
        cell.z = cell.low_sum_z / static_cast<double>(cell.low_count);
      }
      else if (std::isfinite(cell.min_z))
      {
        cell.z = cell.min_z;
      }

      const bool has_obstacle = cell.obstacle_count > 0;
      if (has_obstacle)
      {
        obstacle_cells.push_back(static_cast<int>(i));
      }

      cell.raw_free = !has_obstacle;
      if (cell.raw_free)
      {
        raw_free_count++;
      }
    }

    const int inflate_cells = static_cast<int>(
        std::ceil(grid_obstacle_inflation_radius_ / grid_resolution_));
    for (int obstacle_index : obstacle_cells)
    {
      const int ox = obstacle_index % grid_width_;
      const int oy = obstacle_index / grid_width_;
      for (int dy = -inflate_cells; dy <= inflate_cells; ++dy)
      {
        for (int dx = -inflate_cells; dx <= inflate_cells; ++dx)
        {
          const int nx = ox + dx;
          const int ny = oy + dy;
          if (!inGrid(nx, ny))
          {
            continue;
          }
          const double dist = std::hypot(dx * grid_resolution_, dy * grid_resolution_);
          if (dist <= grid_obstacle_inflation_radius_)
          {
            grid_[gridIndex(nx, ny)].inflated = true;
          }
        }
      }
    }

    std::size_t traversable_count = 0;
    std::size_t inflated_free_count = 0;
    for (GridCell& cell : grid_)
    {
      cell.traversable = cell.raw_free && !cell.inflated;
      if (cell.traversable)
      {
        traversable_count++;
      }
      else if (cell.raw_free && cell.inflated)
      {
        inflated_free_count++;
      }
    }

    RCLCPP_INFO(
        get_logger(),
        "Built 2D PCD grid: %dx%d cells, res=%.2fm, raw_free=%zu, traversable=%zu, obstacle_cells=%zu, inflated_free=%zu.",
        grid_width_, grid_height_, grid_resolution_, raw_free_count, traversable_count,
        obstacle_cells.size(), inflated_free_count);
    if (traversable_count == 0)
    {
      RCLCPP_ERROR(get_logger(), "2D PCD grid has no traversable cells.");
      return false;
    }
    return true;
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_pose_ = msg->pose.pose;
    latest_odom_stamp_ = msg->header.stamp;
    has_odom_.store(true);
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
    std::thread{std::bind(&PcdGridPlanner::executeTopicGoal, this, *msg)}.detach();
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
    if (executing_.load())
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
    executing_.store(true);
    std::thread{std::bind(&PcdGridPlanner::executeGoal, this, goal_handle)}.detach();
  }

  void executeGoal(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    geometry_msgs::msg::Point final_goal = goal->pose.pose.position;
    geometry_msgs::msg::Point current;
    geometry_msgs::msg::PoseStamped current_pose;
    if (!currentPosition(current, current_pose))
    {
      abortGoal(goal_handle, "No current pose available.");
      return;
    }

    std::vector<geometry_msgs::msg::Point> route;
    std::string error;
    if (!planGridRoute(current, final_goal, route, error))
    {
      abortGoal(goal_handle, error);
      return;
    }

    publishRoute(route);
    std::size_t route_index = route.size() > 1 ? 1 : 0;
    const rclcpp::Time start_time = now();
    rclcpp::Rate rate(waypoint_rate_);

    while (rclcpp::ok())
    {
      if (goal_handle->is_canceling())
      {
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->canceled(result);
        executing_.store(false);
        RCLCPP_INFO(get_logger(), "PCD-grid navigation canceled.");
        return;
      }

      if (!currentPosition(current, current_pose))
      {
        abortGoal(goal_handle, "Lost current pose.");
        return;
      }

      if (pointDistXY(current, route.back()) <= goal_tolerance_)
      {
        publishWaypoint(route.back());
        auto result = std::make_shared<NavigateToPose::Result>();
        goal_handle->succeed(result);
        executing_.store(false);
        RCLCPP_INFO(get_logger(), "PCD-grid navigation succeeded.");
        return;
      }

      const geometry_msgs::msg::Point waypoint = selectWaypoint(current, route, route_index);
      publishWaypoint(waypoint);
      publishFeedback(goal_handle, current_pose, route, route_index, start_time);
      rate.sleep();
    }

    abortGoal(goal_handle, "ROS shutdown during navigation.");
  }

  void executeTopicGoal(const geometry_msgs::msg::PoseStamped goal_pose)
  {
    geometry_msgs::msg::Point final_goal = goal_pose.pose.position;
    geometry_msgs::msg::Point current;
    geometry_msgs::msg::PoseStamped current_pose;
    if (!currentPosition(current, current_pose))
    {
      executing_.store(false);
      RCLCPP_ERROR(get_logger(), "RViz /goal_pose aborted: no current pose available.");
      return;
    }

    std::vector<geometry_msgs::msg::Point> route;
    std::string error;
    if (!planGridRoute(current, final_goal, route, error))
    {
      executing_.store(false);
      RCLCPP_ERROR(get_logger(), "RViz /goal_pose aborted: %s", error.c_str());
      return;
    }

    publishRoute(route);
    std::size_t route_index = route.size() > 1 ? 1 : 0;
    rclcpp::Rate rate(waypoint_rate_);

    while (rclcpp::ok())
    {
      if (!currentPosition(current, current_pose))
      {
        executing_.store(false);
        RCLCPP_ERROR(get_logger(), "RViz /goal_pose aborted: lost current pose.");
        return;
      }

      if (pointDistXY(current, route.back()) <= goal_tolerance_)
      {
        publishWaypoint(route.back());
        executing_.store(false);
        RCLCPP_INFO(get_logger(), "RViz /goal_pose navigation succeeded.");
        return;
      }

      const geometry_msgs::msg::Point waypoint = selectWaypoint(current, route, route_index);
      publishWaypoint(waypoint);
      rate.sleep();
    }

    executing_.store(false);
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

  geometry_msgs::msg::Point gridCenter(int index) const
  {
    const int gx = index % grid_width_;
    const int gy = index / grid_width_;
    geometry_msgs::msg::Point point;
    point.x = grid_origin_x_ + (static_cast<double>(gx) + 0.5) * grid_resolution_;
    point.y = grid_origin_y_ + (static_cast<double>(gy) + 0.5) * grid_resolution_;
    point.z = grid_[index].z;
    return point;
  }

  bool gridTraversable(int index) const
  {
    return index >= 0 && index < static_cast<int>(grid_.size()) && grid_[index].traversable;
  }

  bool gridMoveAllowed(int from_idx, int to_idx) const
  {
    if (!gridTraversable(from_idx) || !gridTraversable(to_idx))
    {
      return false;
    }
    const int fx = from_idx % grid_width_;
    const int fy = from_idx / grid_width_;
    const int tx = to_idx % grid_width_;
    const int ty = to_idx / grid_width_;
    const int dx = tx - fx;
    const int dy = ty - fy;
    if (std::abs(dx) == 1 && std::abs(dy) == 1)
    {
      const int side_a = gridIndex(fx + dx, fy);
      const int side_b = gridIndex(fx, fy + dy);
      if (!gridTraversable(side_a) || !gridTraversable(side_b))
      {
        return false;
      }
    }
    return true;
  }

  bool gridLineTraversableWorld(
      const geometry_msgs::msg::Point& from,
      const geometry_msgs::msg::Point& to) const
  {
    const double length = pointDistXY(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * grid_resolution_))));
    int previous_idx = -1;
    for (int i = 1; i <= steps; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      const geometry_msgs::msg::Point sample = lerpPoint(from, to, t);
      int gx = 0;
      int gy = 0;
      if (!worldToGrid(sample.x, sample.y, gx, gy))
      {
        return false;
      }
      const int idx = gridIndex(gx, gy);
      if (!gridTraversable(idx))
      {
        return false;
      }
      if (previous_idx >= 0 && previous_idx != idx && !gridMoveAllowed(previous_idx, idx))
      {
        return false;
      }
      previous_idx = idx;
    }
    return true;
  }

  bool gridLineReachableFromPose(
      const geometry_msgs::msg::Point& from,
      const geometry_msgs::msg::Point& to,
      double allowed_initial_inflation) const
  {
    const double length = pointDistXY(from, to);
    const int steps = std::max(1, static_cast<int>(std::ceil(length / (0.5 * grid_resolution_))));
    int previous_idx = -1;
    bool previous_traversable = false;
    for (int i = 1; i <= steps; ++i)
    {
      const double t = static_cast<double>(i) / static_cast<double>(steps);
      const geometry_msgs::msg::Point sample = lerpPoint(from, to, t);
      int gx = 0;
      int gy = 0;
      if (!worldToGrid(sample.x, sample.y, gx, gy))
      {
        return false;
      }
      const int idx = gridIndex(gx, gy);
      const double distance_from_pose = pointDistXY(from, sample);
      const bool traversable = gridTraversable(idx);
      const bool allowed_initial_cell =
          distance_from_pose <= allowed_initial_inflation &&
          grid_[idx].raw_free &&
          grid_[idx].inflated &&
          grid_[idx].obstacle_count == 0;
      if (!traversable && !allowed_initial_cell)
      {
        return false;
      }
      if (previous_idx >= 0 && previous_idx != idx && previous_traversable && traversable &&
          !gridMoveAllowed(previous_idx, idx))
      {
        return false;
      }
      previous_idx = idx;
      previous_traversable = traversable;
    }
    return true;
  }

  std::vector<GridCandidate> gridSnapCandidates(
      const geometry_msgs::msg::Point& point,
      double radius,
      std::size_t max_candidates) const
  {
    int center_x = 0;
    int center_y = 0;
    if (!worldToGrid(point.x, point.y, center_x, center_y))
    {
      center_x = static_cast<int>(std::floor((point.x - grid_origin_x_) / grid_resolution_));
      center_y = static_cast<int>(std::floor((point.y - grid_origin_y_) / grid_resolution_));
      center_x = std::max(0, std::min(grid_width_ - 1, center_x));
      center_y = std::max(0, std::min(grid_height_ - 1, center_y));
    }

    std::vector<GridCandidate> candidates;
    const int radius_cells = static_cast<int>(std::ceil(radius / grid_resolution_));
    for (int dy = -radius_cells; dy <= radius_cells; ++dy)
    {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx)
      {
        const int gx = center_x + dx;
        const int gy = center_y + dy;
        if (!inGrid(gx, gy))
        {
          continue;
        }
        const int idx = gridIndex(gx, gy);
        if (!gridTraversable(idx))
        {
          continue;
        }

        const geometry_msgs::msg::Point candidate_point = gridCenter(idx);
        const double candidate_dist = pointDistXY(point, candidate_point);
        if (candidate_dist > radius)
        {
          continue;
        }

        GridCandidate candidate;
        candidate.index = idx;
        candidate.distance = candidate_dist;
        candidates.push_back(candidate);
      }
    }

    if (candidates.empty())
    {
      return candidates;
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

  bool searchGridToCandidates(
      const std::vector<GridCandidate>& start_candidates,
      const std::vector<GridCandidate>& goal_candidates,
      std::vector<int>& cell_path,
      GridCandidate& selected_start,
      GridCandidate& selected_goal)
  {
    if (start_candidates.empty() || goal_candidates.empty())
    {
      return false;
    }

    const int cell_count = static_cast<int>(grid_.size());
    std::vector<double> g_score(cell_count, std::numeric_limits<double>::infinity());
    std::vector<int> parent(cell_count, -1);
    std::vector<int> source(cell_count, -1);
    std::vector<bool> closed(cell_count, false);

    typedef std::pair<double, int> QueueItem;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    for (std::size_t i = 0; i < start_candidates.size(); ++i)
    {
      const GridCandidate& candidate = start_candidates[i];
      if (!gridTraversable(candidate.index) || candidate.distance >= g_score[candidate.index])
      {
        continue;
      }
      g_score[candidate.index] = candidate.distance;
      source[candidate.index] = static_cast<int>(i);
      open.push(QueueItem(g_score[candidate.index], candidate.index));
    }

    const int dirs[8][2] = {
        {1, 0}, {-1, 0}, {0, 1}, {0, -1},
        {1, 1}, {1, -1}, {-1, 1}, {-1, -1}};

    while (!open.empty())
    {
      const int current = open.top().second;
      open.pop();
      if (closed[current])
      {
        continue;
      }
      closed[current] = true;

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
        if (closed[next] || !gridMoveAllowed(current, next))
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
          open.push(QueueItem(tentative, next));
        }
      }
    }

    double best_score = std::numeric_limits<double>::infinity();
    int best_goal_idx = -1;
    for (std::size_t i = 0; i < goal_candidates.size(); ++i)
    {
      const GridCandidate& candidate = goal_candidates[i];
      if (!gridTraversable(candidate.index) || !std::isfinite(g_score[candidate.index]))
      {
        continue;
      }
      const double score = g_score[candidate.index] + candidate.distance;
      if (score < best_score)
      {
        best_score = score;
        best_goal_idx = static_cast<int>(i);
      }
    }

    if (best_goal_idx < 0)
    {
      return false;
    }

    selected_goal = goal_candidates[best_goal_idx];
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
    const std::vector<GridCandidate> raw_start_candidates =
        gridSnapCandidates(start, start_snap_radius_, 96);
    std::vector<GridCandidate> start_candidates;
    start_candidates.reserve(raw_start_candidates.size());
    const double start_connector_tolerance = clearance_radius_ + grid_resolution_;
    for (const GridCandidate& candidate : raw_start_candidates)
    {
      if (gridLineReachableFromPose(start, gridCenter(candidate.index), start_connector_tolerance))
      {
        start_candidates.push_back(candidate);
      }
    }
    if (start_candidates.size() > 16)
    {
      start_candidates.resize(16);
    }
    const std::vector<GridCandidate> goal_candidates =
        gridSnapCandidates(requested_goal, goal_snap_radius_, 32);

    if (start_candidates.empty())
    {
      error = "No traversable 2D grid cell with a clear connector near current pose.";
      return false;
    }
    if (goal_candidates.empty())
    {
      error = "No traversable 2D grid cell near requested goal.";
      return false;
    }

    std::vector<int> cell_path;
    GridCandidate selected_start;
    GridCandidate selected_goal;
    if (!searchGridToCandidates(start_candidates, goal_candidates, cell_path, selected_start, selected_goal))
    {
      error = "No reachable 2D grid path from current pose to requested goal.";
      return false;
    }
    const int goal_idx = selected_goal.index;
    const double start_snap = selected_start.distance;
    const double goal_snap = selected_goal.distance;

    route.clear();
    route.push_back(start);
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
        point.z = start.z;
        if (route.empty() || pointDistXY(route.back(), point) > 1e-3)
        {
          route.push_back(point);
        }
      }
    }

    geometry_msgs::msg::Point final_point = gridCenter(goal_idx);
    final_point.z = start.z;
    if (gridLineTraversableWorld(final_point, requested_goal))
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

    RCLCPP_INFO(
        get_logger(),
        "2D PCD grid route planned: %zu waypoints, cells=%zu, start_snap=%.2fm, goal_snap=%.2fm.",
        route.size(), cell_path.size(), start_snap, goal_snap);
    return route.size() >= 2;
  }

  void abortGoal(const std::shared_ptr<GoalHandleNavigateToPose>& goal_handle, const std::string& message)
  {
    auto result = std::make_shared<NavigateToPose::Result>();
    if (goal_handle->is_active())
    {
      goal_handle->abort(result);
    }
    executing_.store(false);
    RCLCPP_ERROR(get_logger(), "PCD-grid navigation aborted: %s", message.c_str());
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
    map.data.resize(grid_.size(), 0);

    for (std::size_t i = 0; i < grid_.size(); ++i)
    {
      if (grid_[i].obstacle_count > 0 || (grid_[i].raw_free && grid_[i].inflated))
      {
        map.data[i] = 100;
      }
      else
      {
        map.data[i] = 0;
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

    visualization_msgs::msg::Marker free_marker;
    free_marker.header.frame_id = map_frame_;
    free_marker.header.stamp = now();
    free_marker.ns = "pcd_grid_free";
    free_marker.id = 0;
    free_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    free_marker.action = visualization_msgs::msg::Marker::ADD;
    free_marker.pose.orientation.w = 1.0;
    free_marker.scale.x = grid_resolution_;
    free_marker.scale.y = grid_resolution_;
    free_marker.scale.z = 0.03;
    free_marker.color.r = 0.1f;
    free_marker.color.g = 0.8f;
    free_marker.color.b = 0.55f;
    free_marker.color.a = 0.28f;

    visualization_msgs::msg::Marker blocked_marker;
    blocked_marker.header = free_marker.header;
    blocked_marker.ns = "pcd_grid_blocked";
    blocked_marker.id = 1;
    blocked_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    blocked_marker.action = visualization_msgs::msg::Marker::ADD;
    blocked_marker.pose.orientation.w = 1.0;
    blocked_marker.scale.x = grid_resolution_;
    blocked_marker.scale.y = grid_resolution_;
    blocked_marker.scale.z = 0.05;
    blocked_marker.color.r = 1.0f;
    blocked_marker.color.g = 0.25f;
    blocked_marker.color.b = 0.05f;
    blocked_marker.color.a = 0.35f;

    for (std::size_t i = 0; i < grid_.size(); ++i)
    {
      if (grid_[i].traversable)
      {
        geometry_msgs::msg::Point point = gridCenter(static_cast<int>(i));
        point.z += 0.02;
        free_marker.points.push_back(point);
      }
      else if (grid_[i].obstacle_count > 0 || (grid_[i].raw_free && grid_[i].inflated))
      {
        geometry_msgs::msg::Point point = gridCenter(static_cast<int>(i));
        point.z += 0.04;
        blocked_marker.points.push_back(point);
      }
    }

    array.markers.push_back(free_marker);
    array.markers.push_back(blocked_marker);
    grid_marker_pub_->publish(array);
  }

  std::string map_file_;
  std::string map_frame_;

  double map_voxel_size_ = 0.08;
  double grid_resolution_ = 0.20;
  double grid_padding_ = 0.2;
  double grid_obstacle_inflation_radius_ = -1.0;
  int grid_max_cells_ = 2000000;
  double max_slope_deg_ = 25.0;
  double max_slope_rad_ = 25.0 * M_PI / 180.0;
  double max_step_height_ = 0.18;
  double min_clearance_ = 0.35;
  double obstacle_height_ = 0.15;
  double vehicle_height_ = 1.0;
  double vehicle_length_ = 0.7;
  double vehicle_width_ = 0.3;
  double clearance_radius_ = 0.35;

  double start_snap_radius_ = 6.0;
  double goal_snap_radius_ = 2.0;
  double lookahead_distance_ = 2.5;
  double waypoint_reach_radius_ = 0.6;
  double goal_tolerance_ = 0.35;
  double waypoint_rate_ = 5.0;

  bool map_ready_ = false;
  std::atomic_bool has_odom_{false};
  std::atomic_bool executing_{false};

  pcl::PointCloud<pcl::PointXYZI>::Ptr map_cloud_;

  std::vector<GridCell> grid_;
  int grid_width_ = 0;
  int grid_height_ = 0;
  double grid_origin_x_ = 0.0;
  double grid_origin_y_ = 0.0;

  std::mutex odom_mutex_;
  geometry_msgs::msg::Pose latest_pose_;
  rclcpp::Time latest_odom_stamp_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr waypoint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_map_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr grid_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr path_marker_pub_;
  rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcdGridPlanner>());
  rclcpp::shutdown();
  return 0;
}
