#include <cmath>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iostream>
#include <mutex>
#include <atomic>
#include <stdexcept>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/path.hpp>

#include "local_planner/srv/set_speed.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <sensor_msgs/msg/imu.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "rmw/types.h"
#include "rmw/qos_profiles.h"

using namespace std;


#define PLOTPATHSET 1

// Action server type definition
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ServerGoalHandle<NavigateToPose>;

// Goal source tracking
enum class GoalSource { NONE, WAYPOINT, ACTION_SERVER };
GoalSource goalSource_ = GoalSource::NONE;

// Action server and goal state variables
rclcpp_action::Server<NavigateToPose>::SharedPtr action_server_;
std::shared_ptr<GoalHandleNavigateToPose> current_goal_handle_;
rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr pubJoy_;
rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pubSpeedConfig_;
bool has_active_goal_ = false;
double goal_tolerance_ = 0.3;  // meters

string pathFolder;
string vehicleFrame = "vehicle";
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = false;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre1 = 0.15;
double costHeightThre2 = 0.1;
bool useCost = false;
int slowPathNumThre = 5;
int slowGroupNumThre = 1;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.25;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
double freezeAng = 90.0;
double freezeTime = 2.0;
double freezeStartTime = 0;
int freezeStatus = 0;
double omniDirGoalThre = 1.0;
double goalClearRange = 0.5;
double goalBehindRange = 0.8;
double goalX = 0;
double goalY = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.45;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr boundaryCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr addedObstacles(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];
#if PLOTPATHSET == 1
pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
pcl::PointCloud<pcl::PointXYZI>::Ptr freePaths(new pcl::PointCloud<pcl::PointXYZI>());
#endif

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
int clearPathPerGroupNum[36 * groupNum] = {0};
float pathPenaltyPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

std::atomic<bool> newLaserCloud{false};
std::atomic<bool> newTerrainCloud{false};

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> laserDwzFilter, terrainDwzFilter;
rclcpp::Node::SharedPtr nh;
std::mutex poseMtx;
std::atomic<double> lastScanReceiveTime{0};

static float clampedAutonomySpeed()
{
  float s = static_cast<float>(autonomySpeed / maxSpeed);
  if (s < 0.0f) s = 0.0f;
  else if (s > 1.0f) s = 1.0f;
  return s;
}

void publishJoy(bool autonomy)
{
  sensor_msgs::msg::Joy joy;
  joy.axes = {0, 0, autonomy ? -1.0f : 0.0f, 0, autonomy ? 1.0f : 0.0f, 1.0f, 0, 0};
  joy.buttons = {0, 0, 0, 0, 0, 0, 0, autonomy ? 1 : 0, 0, 0, 0};
  joy.header.stamp = nh->now();
  joy.header.frame_id = "action_server";
  pubJoy_->publish(joy);
}

static void publishSpeedConfig()
{
  std_msgs::msg::Float32MultiArray msg;
  msg.data = {static_cast<float>(maxSpeed), static_cast<float>(autonomySpeed)};
  pubSpeedConfig_->publish(msg);
}

void activateGoal(double x, double y, GoalSource source)
{
  goalX = x;
  goalY = y;
  goalSource_ = source;
  autonomyMode = true;
  joySpeed = clampedAutonomySpeed();
  publishJoy(true);
  RCLCPP_INFO(nh->get_logger(), "Goal activated: x=%.2f, y=%.2f", x, y);
}

void clearGoal()
{
  if (goalSource_ == GoalSource::ACTION_SERVER && has_active_goal_
      && current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<NavigateToPose::Result>();
    current_goal_handle_->succeed(result);
  }
  goalSource_ = GoalSource::NONE;
  has_active_goal_ = false;
  current_goal_handle_ = nullptr;
}

void cancelGoal()
{
  if (goalSource_ == GoalSource::ACTION_SERVER && has_active_goal_
      && current_goal_handle_ && current_goal_handle_->is_active()) {
    auto result = std::make_shared<NavigateToPose::Result>();
    current_goal_handle_->abort(result);
  }
  goalSource_ = GoalSource::NONE;
  has_active_goal_ = false;
  current_goal_handle_ = nullptr;
  autonomyMode = false;
  publishJoy(false);
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  std::lock_guard<std::mutex> lock(poseMtx);
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odom->pose.pose.position.z;
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloud2)
{
  if (!useTerrainAnalysis) {
    float snapVX, snapVY;
    {
      std::lock_guard<std::mutex> lock(poseMtx);
      snapVX = vehicleX;
      snapVY = vehicleY;
    }
    lastScanReceiveTime.store(nh->now().seconds());

    laserCloud->clear();
    pcl::fromROSMsg(*laserCloud2, *laserCloud);

    pcl::PointXYZI point;
    laserCloudCrop->clear();
    size_t laserCloudSize = laserCloud->points.size();
    laserCloudCrop->reserve(laserCloudSize);
    float adjacentRangeSq = adjacentRange * adjacentRange;
    for (size_t i = 0; i < laserCloudSize; i++) {
      const auto& srcPt = laserCloud->points[i];

      float dX = srcPt.x - snapVX;
      float dY = srcPt.y - snapVY;
      float disSq = dX * dX + dY * dY;
      if (disSq < adjacentRangeSq) {
        point.x = srcPt.x;
        point.y = srcPt.y;
        point.z = srcPt.z;
        point.intensity = srcPt.intensity;
        laserCloudCrop->push_back(point);
      }
    }

    laserCloudDwz->clear();
    laserDwzFilter.setInputCloud(laserCloudCrop);
    laserDwzFilter.filter(*laserCloudDwz);

    newLaserCloud = true;
  }
}

void terrainCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2)
{
  if (useTerrainAnalysis) {
    float snapVX, snapVY;
    {
      std::lock_guard<std::mutex> lock(poseMtx);
      snapVX = vehicleX;
      snapVY = vehicleY;
    }
    lastScanReceiveTime.store(nh->now().seconds());

    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    float adjacentRangeSq = adjacentRange * adjacentRange;
    size_t terrainCloudSize = terrainCloud->points.size();
    terrainCloudCrop->reserve(terrainCloudSize);
    for (size_t i = 0; i < terrainCloudSize; i++) {
      const auto& srcPt = terrainCloud->points[i];

      float dX = srcPt.x - snapVX;
      float dY = srcPt.y - snapVY;
      float disSq = dX * dX + dY * dY;
      if (disSq < adjacentRangeSq && (srcPt.intensity > obstacleHeightThre || (srcPt.intensity > groundHeightThre && useCost))) {
        point.x = srcPt.x;
        point.y = srcPt.y;
        point.z = srcPt.z;
        point.intensity = srcPt.intensity;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);

  if (goalSource_ != GoalSource::NONE) {
    // Goal active (waypoint or action server): only allow manual override
    if (joy->axes[2] > -0.1 && joySpeedRaw > 0.1) {
      RCLCPP_INFO(nh->get_logger(), "Manual override - cancelling goal");
      cancelGoal();
      joySpeed = joySpeedRaw;
      if (joySpeed > 1.0) joySpeed = 1.0;
    }
  } else {
    // No active goal: normal joystick control
    if (joy->axes[2] > -0.1) {
      autonomyMode = false;
    } else {
      autonomyMode = true;
    }

    if (autonomyMode) {
      joySpeed = clampedAutonomySpeed();
    } else {
      joySpeed = joySpeedRaw;
      if (joySpeed > 1.0) joySpeed = 1.0;
      if (joy->axes[4] == 0) joySpeed = 0;
    }

    if (joySpeed > 0) {
      joyDir = atan2(joy->axes[3], joy->axes[4]) * 180 / M_PI;
      if (joy->axes[4] < 0) joyDir *= -1;
    }

    if (joy->axes[4] < 0 && !twoWayDrive) joySpeed = 0;
  }

  if (joy->axes[5] > -0.1) {
    checkObstacle = true;
  } else {
    checkObstacle = false;
  }
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal)
{
  if (goalSource_ == GoalSource::ACTION_SERVER) return;
  activateGoal(goal->point.x, goal->point.y, GoalSource::WAYPOINT);
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && !has_active_goal_ && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void setSpeedHandler(
  const std::shared_ptr<local_planner::srv::SetSpeed::Request> request,
  std::shared_ptr<local_planner::srv::SetSpeed::Response> response)
{
  double newMaxSpeed = request->max_speed;
  double newAutonomySpeed = request->autonomy_speed;
  bool changeMax = (newMaxSpeed >= 0);
  bool changeAutonomy = (newAutonomySpeed >= 0);

  if (!changeMax && !changeAutonomy) {
    response->success = true;
    response->message = "No changes requested. Returning current values.";
    response->current_max_speed = maxSpeed;
    response->current_autonomy_speed = autonomySpeed;
    return;
  }

  if (!changeMax) newMaxSpeed = maxSpeed;
  if (!changeAutonomy) newAutonomySpeed = autonomySpeed;

  if (newMaxSpeed <= 0) {
    response->success = false;
    response->message = "max_speed must be > 0, got " + std::to_string(newMaxSpeed);
    response->current_max_speed = maxSpeed;
    response->current_autonomy_speed = autonomySpeed;
    return;
  }
  if (newAutonomySpeed <= 0) {
    response->success = false;
    response->message = "autonomy_speed must be > 0, got " + std::to_string(newAutonomySpeed);
    response->current_max_speed = maxSpeed;
    response->current_autonomy_speed = autonomySpeed;
    return;
  }
  if (newAutonomySpeed > newMaxSpeed) {
    response->success = false;
    response->message = "autonomy_speed (" + std::to_string(newAutonomySpeed) +
                        ") must be <= max_speed (" + std::to_string(newMaxSpeed) + ")";
    response->current_max_speed = maxSpeed;
    response->current_autonomy_speed = autonomySpeed;
    return;
  }

  maxSpeed = newMaxSpeed;
  autonomySpeed = newAutonomySpeed;

  if (autonomyMode) {
    joySpeed = clampedAutonomySpeed();
  }

  publishSpeedConfig();

  RCLCPP_INFO(nh->get_logger(), "Speed updated: maxSpeed=%.3f, autonomySpeed=%.3f",
              maxSpeed, autonomySpeed);

  response->success = true;
  response->message = "Speed parameters updated successfully.";
  response->current_max_speed = maxSpeed;
  response->current_autonomy_speed = autonomySpeed;
}

void boundaryHandler(const geometry_msgs::msg::PolygonStamped::ConstSharedPtr boundary)
{
  boundaryCloud->clear();
  pcl::PointXYZI point, point1, point2;
  size_t boundarySize = boundary->polygon.points.size();

  if (boundarySize >= 1) {
    point2.x = boundary->polygon.points[0].x;
    point2.y = boundary->polygon.points[0].y;
    point2.z = boundary->polygon.points[0].z;
  }

  for (size_t i = 0; i < boundarySize; i++) {
    point1 = point2;

    point2.x = boundary->polygon.points[i].x;
    point2.y = boundary->polygon.points[i].y;
    point2.z = boundary->polygon.points[i].z;

    if (point1.z == point2.z) {
      float disX = point1.x - point2.x;
      float disY = point1.y - point2.y;
      float dis = sqrt(disX * disX + disY * disY);

      int pointNum = int(dis / terrainVoxelSize) + 1;
      for (int pointID = 0; pointID < pointNum; pointID++) {
        point.x = float(pointID) / float(pointNum) * point1.x + (1.0 - float(pointID) / float(pointNum)) * point2.x;
        point.y = float(pointID) / float(pointNum) * point1.y + (1.0 - float(pointID) / float(pointNum)) * point2.y;
        point.z = 0;
        point.intensity = 100.0;

        for (int j = 0; j < pointPerPathThre; j++) {
          boundaryCloud->push_back(point);
        }
      }
    }
  }
}

void addedObstaclesHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr addedObstacles2)
{
  addedObstacles->clear();
  pcl::fromROSMsg(*addedObstacles2, *addedObstacles);

  size_t addedObstaclesSize = addedObstacles->points.size();
  for (size_t i = 0; i < addedObstaclesSize; i++) {
    addedObstacles->points[i].intensity = 200.0;
  }
}

void checkObstacleHandler(const std_msgs::msg::Bool::ConstSharedPtr checkObs)
{
  double checkObsTime = nh->now().seconds();
  if (autonomyMode && checkObsTime - joyTime > joyToCheckObstacleDelay) {
    checkObstacle = checkObs->data;
  }
}

int readPlyHeader(FILE *filePtr)
{
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      throw std::runtime_error("Error reading PLY header");
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        throw std::runtime_error("Error reading PLY header vertex count");
      }
    }
  }

  return pointNum;
}

void readStartPaths()
{
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("Cannot open " + fileName);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      fclose(filePtr);
      throw std::runtime_error("Error reading data from " + fileName);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

#if PLOTPATHSET == 1
void readPaths()
{
  string fileName = pathFolder + "/paths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("Cannot open " + fileName);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZI point;
  int pointSkipNum = 30;
  int pointSkipCount = 0;
  int val1, val2, val3, val4, val5, pathID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%f", &point.intensity);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      fclose(filePtr);
      throw std::runtime_error("Error reading data from " + fileName);
    }

    if (pathID >= 0 && pathID < pathNum) {
      pointSkipCount++;
      if (pointSkipCount > pointSkipNum) {
        paths[pathID]->push_back(point);
        pointSkipCount = 0;
      }
    }
  }

  fclose(filePtr);
}
#endif

void readPathList()
{
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("Cannot open " + fileName);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    fclose(filePtr);
    throw std::runtime_error("Incorrect path number in " + fileName);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      fclose(filePtr);
      throw std::runtime_error("Error reading data from " + fileName);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / M_PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences()
{
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    throw std::runtime_error("Cannot open " + fileName);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      fclose(filePtr);
      throw std::runtime_error("Error reading grid voxel ID from " + fileName);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        fclose(filePtr);
        throw std::runtime_error("Error reading path ID from " + fileName);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 && pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

// Action server callbacks
rclcpp_action::GoalResponse handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const NavigateToPose::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(nh->get_logger(), "Received navigation goal request to x: %.2f, y: %.2f",
              goal->pose.pose.position.x, goal->pose.pose.position.y);
  
  // Accept all goals
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(
  const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  (void)goal_handle;
  RCLCPP_INFO(nh->get_logger(), "Received request to cancel navigation goal");
  cancelGoal();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<GoalHandleNavigateToPose> goal_handle)
{
  if (goalSource_ != GoalSource::NONE) {
    cancelGoal();
  }

  current_goal_handle_ = goal_handle;
  has_active_goal_ = true;

  const auto goal = goal_handle->get_goal();
  activateGoal(goal->pose.pose.position.x, goal->pose.pose.position.y,
               GoalSource::ACTION_SERVER);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");

  nh->declare_parameter<std::string>("pathFolder", pathFolder);
  nh->declare_parameter<std::string>("vehicleFrame", vehicleFrame);
  nh->declare_parameter<double>("vehicleLength", vehicleLength);
  nh->declare_parameter<double>("vehicleWidth", vehicleWidth);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
  nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
  nh->declare_parameter<bool>("useTerrainAnalysis", useTerrainAnalysis);
  nh->declare_parameter<bool>("checkObstacle", checkObstacle);
  nh->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
  nh->declare_parameter<double>("adjacentRange", adjacentRange);
  nh->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
  nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
  nh->declare_parameter<double>("costHeightThre1", costHeightThre1);
  nh->declare_parameter<double>("costHeightThre2", costHeightThre2);
  nh->declare_parameter<bool>("useCost", useCost);
  nh->declare_parameter<int>("slowPathNumThre", slowPathNumThre);
  nh->declare_parameter<int>("slowGroupNumThre", slowGroupNumThre);
  nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
  nh->declare_parameter<double>("minRelZ", minRelZ);
  nh->declare_parameter<double>("maxRelZ", maxRelZ);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("dirWeight", dirWeight);
  nh->declare_parameter<double>("dirThre", dirThre);
  nh->declare_parameter<bool>("dirToVehicle", dirToVehicle);
  nh->declare_parameter<double>("pathScale", pathScale);
  nh->declare_parameter<double>("minPathScale", minPathScale);
  nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
  nh->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
  nh->declare_parameter<double>("minPathRange", minPathRange);
  nh->declare_parameter<double>("pathRangeStep", pathRangeStep);
  nh->declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed);
  nh->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nh->declare_parameter<double>("freezeAng", freezeAng);  
  nh->declare_parameter<double>("freezeTime", freezeTime);
  nh->declare_parameter<double>("omniDirGoalThre", omniDirGoalThre);
  nh->declare_parameter<double>("goalClearRange", goalClearRange);
  nh->declare_parameter<double>("goalBehindRange", goalBehindRange);
  nh->declare_parameter<double>("goalX", goalX);
  nh->declare_parameter<double>("goalY", goalY);
  nh->declare_parameter<double>("goalTolerance", goal_tolerance_);

  nh->get_parameter("pathFolder", pathFolder);
  nh->get_parameter("vehicleFrame", vehicleFrame);
  nh->get_parameter("vehicleLength", vehicleLength);
  nh->get_parameter("vehicleWidth", vehicleWidth);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("laserVoxelSize", laserVoxelSize);
  nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
  nh->get_parameter("useTerrainAnalysis", useTerrainAnalysis);
  nh->get_parameter("checkObstacle", checkObstacle);
  nh->get_parameter("checkRotObstacle", checkRotObstacle);
  nh->get_parameter("adjacentRange", adjacentRange);
  nh->get_parameter("obstacleHeightThre", obstacleHeightThre);
  nh->get_parameter("groundHeightThre", groundHeightThre);
  nh->get_parameter("costHeightThre1", costHeightThre1);
  nh->get_parameter("costHeightThre2", costHeightThre2);
  nh->get_parameter("useCost", useCost);
  nh->get_parameter("slowPathNumThre", slowPathNumThre);
  nh->get_parameter("slowGroupNumThre", slowGroupNumThre);
  nh->get_parameter("pointPerPathThre", pointPerPathThre);
  nh->get_parameter("minRelZ", minRelZ);
  nh->get_parameter("maxRelZ", maxRelZ);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("dirWeight", dirWeight);
  nh->get_parameter("dirThre", dirThre);
  nh->get_parameter("dirToVehicle", dirToVehicle);
  nh->get_parameter("pathScale", pathScale);
  nh->get_parameter("minPathScale", minPathScale);
  nh->get_parameter("pathScaleStep", pathScaleStep);
  nh->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
  nh->get_parameter("minPathRange", minPathRange);
  nh->get_parameter("pathRangeStep", pathRangeStep);
  nh->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
  nh->get_parameter("pathCropByGoal", pathCropByGoal);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nh->get_parameter("freezeAng", freezeAng);
  nh->get_parameter("freezeTime", freezeTime);
  nh->get_parameter("omniDirGoalThre", omniDirGoalThre);
  nh->get_parameter("goalClearRange", goalClearRange);
  nh->get_parameter("goalBehindRange", goalBehindRange);
  nh->get_parameter("goalX", goalX);
  nh->get_parameter("goalY", goalY);
  nh->get_parameter("goalTolerance", goal_tolerance_);

  // Validate safety-critical parameters
  if (maxSpeed <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "maxSpeed must be > 0, got %f", maxSpeed);
    rclcpp::shutdown();
    return 1;
  }
  if (adjacentRange <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "adjacentRange must be > 0, got %f", adjacentRange);
    rclcpp::shutdown();
    return 1;
  }
  if (vehicleLength <= 0 || vehicleWidth <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "vehicleLength and vehicleWidth must be > 0, got %f, %f",
        vehicleLength, vehicleWidth);
    rclcpp::shutdown();
    return 1;
  }
  if (obstacleHeightThre < 0) {
    RCLCPP_FATAL(nh->get_logger(), "obstacleHeightThre must be >= 0, got %f", obstacleHeightThre);
    rclcpp::shutdown();
    return 1;
  }
  if (groundHeightThre < 0) {
    RCLCPP_FATAL(nh->get_logger(), "groundHeightThre must be >= 0, got %f", groundHeightThre);
    rclcpp::shutdown();
    return 1;
  }
  if (laserVoxelSize <= 0 || terrainVoxelSize <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "Voxel sizes must be > 0, got laser=%f terrain=%f",
        laserVoxelSize, terrainVoxelSize);
    rclcpp::shutdown();
    return 1;
  }
  if (pathScale <= 0 || minPathScale <= 0 || minPathRange <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "pathScale, minPathScale, minPathRange must be > 0");
    rclcpp::shutdown();
    return 1;
  }

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/registered_scan", rclcpp::SensorDataQoS(), laserCloudHandler);

  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/terrain_map", 5, terrainCloudHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped> ("/way_point", 5, goalHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subBoundary = nh->create_subscription<geometry_msgs::msg::PolygonStamped>("/navigation_boundary", 5, boundaryHandler);

  auto subAddedObstacles = nh->create_subscription<sensor_msgs::msg::PointCloud2>("/added_obstacles", 5, addedObstaclesHandler);

  auto subCheckObstacle = nh->create_subscription<std_msgs::msg::Bool>("/check_obstacle", 5, checkObstacleHandler);

  // Create joy publisher for action server to activate the full pipeline
  pubJoy_ = nh->create_publisher<sensor_msgs::msg::Joy>("/joy", 5);

  // Speed config publisher — transient_local so pathFollower gets the last value on startup
  pubSpeedConfig_ = nh->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/speed_config", rclcpp::QoS(1).transient_local());

  // Create action server for NavigateToPose
  action_server_ = rclcpp_action::create_server<NavigateToPose>(
    nh,
    "navigate_to_pose",
    handle_goal,
    handle_cancel,
    handle_accepted);

  RCLCPP_INFO(nh->get_logger(), "NavigateToPose action server created");

  auto setSpeedService = nh->create_service<local_planner::srv::SetSpeed>(
    "/local_planner/set_speed", setSpeedHandler);

  auto pubSlowDown = nh->create_publisher<std_msgs::msg::Int8> ("/slow_down", 5);
  std_msgs::msg::Int8 slow;

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  nav_msgs::msg::Path path;

  #if PLOTPATHSET == 1
  auto pubFreePaths = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/free_paths", 2);
  #endif

  //auto pubLaserCloud = nh->create_publisher<sensor_msgs::msg::PointCloud2> ("/stacked_scans", 2);

  RCLCPP_INFO(nh->get_logger(), "Reading path files.");

  if (autonomyMode) {
    joySpeed = clampedAutonomySpeed();
  }

  publishSpeedConfig();

  for (int i = 0; i < laserCloudStackNum; i++) {
    laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  #if PLOTPATHSET == 1
  for (int i = 0; i < pathNum; i++) {
    paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
  }
  #endif
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  laserDwzFilter.setLeafSize(laserVoxelSize, laserVoxelSize, laserVoxelSize);
  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize, terrainVoxelSize);

  try {
    readStartPaths();
    #if PLOTPATHSET == 1
    readPaths();
    #endif
    readPathList();
    readCorrespondences();
  } catch (const std::runtime_error& e) {
    RCLCPP_FATAL(nh->get_logger(), "Failed to load path files: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(nh->get_logger(), "Initialization complete.");

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (newLaserCloud || newTerrainCloud) {
      // Snapshot pose state for thread-safety
      float vehicleX, vehicleY, vehicleZ, vehicleRoll, vehiclePitch, vehicleYaw;
      double odomTime;
      {
        std::lock_guard<std::mutex> lock(poseMtx);
        vehicleX = ::vehicleX;
        vehicleY = ::vehicleY;
        vehicleZ = ::vehicleZ;
        vehicleRoll = ::vehicleRoll;
        vehiclePitch = ::vehiclePitch;
        vehicleYaw = ::vehicleYaw;
        odomTime = ::odomTime;
      }

      if (newLaserCloud) {
        newLaserCloud = false;

        laserCloudStack[laserCloudCount]->clear();
        *laserCloudStack[laserCloudCount] = *laserCloudDwz;
        laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;

        plannerCloud->clear();
        for (int i = 0; i < laserCloudStackNum; i++) {
          *plannerCloud += *laserCloudStack[i];
        }
      }

      if (newTerrainCloud) {
        newTerrainCloud = false;

        plannerCloud->clear();
        *plannerCloud = *terrainCloudDwz;
      }

      float sinVehicleYaw = sin(vehicleYaw);
      float cosVehicleYaw = cos(vehicleYaw);
      float adjacentRangeSq = adjacentRange * adjacentRange;

      pcl::PointXYZI point;
      plannerCloudCrop->clear();
      size_t plannerCloudSize = plannerCloud->points.size();
      size_t boundaryCloudSize = boundaryCloud->points.size();
      size_t addedObstaclesSize = addedObstacles->points.size();
      plannerCloudCrop->reserve(plannerCloudSize + boundaryCloudSize + addedObstaclesSize);
      for (size_t i = 0; i < plannerCloudSize; i++) {
        float pointX1 = plannerCloud->points[i].x - vehicleX;
        float pointY1 = plannerCloud->points[i].y - vehicleY;
        float pointZ1 = plannerCloud->points[i].z - vehicleZ;

        point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
        point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
        point.z = pointZ1;
        point.intensity = plannerCloud->points[i].intensity;

        float disSq = point.x * point.x + point.y * point.y;
        if (disSq < adjacentRangeSq && ((point.z > minRelZ && point.z < maxRelZ) || useTerrainAnalysis)) {
          plannerCloudCrop->push_back(point);
        }
      }

      for (size_t i = 0; i < boundaryCloudSize; i++) {
        point.x = ((boundaryCloud->points[i].x - vehicleX) * cosVehicleYaw
                + (boundaryCloud->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(boundaryCloud->points[i].x - vehicleX) * sinVehicleYaw
                + (boundaryCloud->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = boundaryCloud->points[i].z;
        point.intensity = boundaryCloud->points[i].intensity;

        float disSq = point.x * point.x + point.y * point.y;
        if (disSq < adjacentRangeSq) {
          plannerCloudCrop->push_back(point);
        }
      }

      for (size_t i = 0; i < addedObstaclesSize; i++) {
        point.x = ((addedObstacles->points[i].x - vehicleX) * cosVehicleYaw
                + (addedObstacles->points[i].y - vehicleY) * sinVehicleYaw);
        point.y = (-(addedObstacles->points[i].x - vehicleX) * sinVehicleYaw
                + (addedObstacles->points[i].y - vehicleY) * cosVehicleYaw);
        point.z = addedObstacles->points[i].z;
        point.intensity = addedObstacles->points[i].intensity;

        float disSq = point.x * point.x + point.y * point.y;
        if (disSq < adjacentRangeSq) {
          plannerCloudCrop->push_back(point);
        }
      }

      float pathRange = adjacentRange;
      if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
      if (pathRange < minPathRange) pathRange = minPathRange;
      float relativeGoalDis = adjacentRange;

      int preSelectedGroupID = -1;
      if (autonomyMode) {
        float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw + (goalY - vehicleY) * sinVehicleYaw);
        float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw + (goalY - vehicleY) * cosVehicleYaw);

        relativeGoalDis = sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
        joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / M_PI;
        
        if (fabs(joyDir) > freezeAng && relativeGoalDis < goalBehindRange) {
          relativeGoalDis = 0;
          joyDir = 0;
        }
        
        if (fabs(joyDir) > freezeAng && freezeStatus == 0) {
          freezeStartTime = odomTime;
          freezeStatus = 1;
        } else if (odomTime - freezeStartTime > freezeTime && freezeStatus == 1) {
          freezeStatus = 2;
        } else if (fabs(joyDir) <= freezeAng && freezeStatus == 2) {
          freezeStatus = 0;
        }

        if (!twoWayDrive) {
          if (joyDir > 95.0) {
            joyDir = 95.0;
            preSelectedGroupID = 0;
          } else if (joyDir < -95.0) {
            joyDir = -95.0;
            preSelectedGroupID = 6;
          }
        }
      } else {
        freezeStatus = 0;
      }

      if (freezeStatus == 1 && autonomyMode) {
        relativeGoalDis = 0;
        joyDir = 0;
      }

      bool pathFound = false;
      float defPathScale = pathScale;
      if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
      if (pathScale < minPathScale) pathScale = minPathScale;

      while (pathScale >= minPathScale && pathRange >= minPathRange) {
        for (int i = 0; i < 36 * pathNum; i++) {
          clearPathList[i] = 0;
          pathPenaltyList[i] = 0;
        }
        for (int i = 0; i < 36 * groupNum; i++) {
          clearPathPerGroupScore[i] = 0;
          clearPathPerGroupNum[i] = 0;
          pathPenaltyPerGroupScore[i] = 0;
        }

        float minObsAngCW = -180.0;
        float minObsAngCCW = 180.0;
        float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 + vehicleWidth / 2.0 * vehicleWidth / 2.0);
        float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / M_PI;
        float pathRangeScaled = pathRange / pathScale;
        float goalRangeScaled = (relativeGoalDis + goalClearRange) / pathScale;
        float diameterScaled = diameter / pathScale;
        float pathRangeScaledSq = pathRangeScaled * pathRangeScaled;
        float goalRangeScaledSq = goalRangeScaled * goalRangeScaled;
        float diameterScaledSq = diameterScaled * diameterScaled;
        // Pre-compute sin/cos lookup table for 36 rotation directions
        float rotCosTable[36], rotSinTable[36];
        for (int rotDir = 0; rotDir < 36; rotDir++) {
          float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
          rotCosTable[rotDir] = cos(rotAng);
          rotSinTable[rotDir] = sin(rotAng);
        }

        size_t plannerCloudCropSize = plannerCloudCrop->points.size();
        for (size_t i = 0; i < plannerCloudCropSize; i++) {
          float x = plannerCloudCrop->points[i].x / pathScale;
          float y = plannerCloudCrop->points[i].y / pathScale;
          float h = plannerCloudCrop->points[i].intensity;
          float disSq = x * x + y * y;

          if (disSq < pathRangeScaledSq && (disSq <= goalRangeScaledSq || !pathCropByGoal) && checkObstacle) {
            for (int rotDir = 0; rotDir < 36; rotDir++) {
              float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
              if (angDiff > 180.0) {
                angDiff = 360.0 - angDiff;
              }
              if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                  ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
                continue;
              }

              float cosRot = rotCosTable[rotDir];
              float sinRot = rotSinTable[rotDir];
              float x2 = cosRot * x + sinRot * y;
              float y2 = -sinRot * x + cosRot * y;

              float scaleY = x2 / gridVoxelOffsetX + searchRadius / gridVoxelOffsetY 
                             * (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

              int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) / gridVoxelSize);
              int indY = int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) / gridVoxelSize);
              if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 && indY < gridVoxelNumY) {
                int ind = gridVoxelNumY * indX + indY;
                size_t blockedPathByVoxelNum = correspondences[ind].size();
                for (size_t j = 0; j < blockedPathByVoxelNum; j++) {
                  if (h > obstacleHeightThre || !useTerrainAnalysis) {
                    clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                  } else {
                    if (pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] < h && h > groundHeightThre) {
                      pathPenaltyList[pathNum * rotDir + correspondences[ind][j]] = h;
                    }
                  }
                }
              }
            }
          }

          if (disSq < diameterScaledSq && (fabs(x) > vehicleLength / pathScale / 2.0 || fabs(y) > vehicleWidth / pathScale / 2.0) &&
              (h > obstacleHeightThre || !useTerrainAnalysis) && checkRotObstacle) {
            float angObs = atan2(y, x) * 180.0 / M_PI;
            if (angObs > 0) {
              if (minObsAngCCW > angObs - angOffset) minObsAngCCW = angObs - angOffset;
              if (minObsAngCW < angObs + angOffset - 180.0) minObsAngCW = angObs + angOffset - 180.0;
            } else {
              if (minObsAngCW < angObs + angOffset) minObsAngCW = angObs + angOffset;
              if (minObsAngCCW > 180.0 + angObs - angOffset) minObsAngCCW = 180.0 + angObs - angOffset;
            }
          }
        }

        if (minObsAngCW > 0) minObsAngCW = 0;
        if (minObsAngCCW < 0) minObsAngCCW = 0;

        for (int i = 0; i < 36 * pathNum; i++) {
          int rotDir = int(i / pathNum);
          float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
          if (angDiff > 180.0) {
            angDiff = 360.0 - angDiff;
          }
          if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
              ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle)) {
            continue;
          }

          if (clearPathList[i] < pointPerPathThre) {
            float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] - (10.0 * rotDir - 180.0));
            if (dirDiff > 360.0) {
              dirDiff -= 360.0;
            }
            if (dirDiff > 180.0) {
              dirDiff = 360.0 - dirDiff;
            }

            float rotDirW;
            if (rotDir < 18) rotDirW = fabs(fabs(rotDir - 9) + 1);
            else rotDirW = fabs(fabs(rotDir - 27) + 1);
            float groupDirW = 4  - fabs(pathList[i % pathNum] - 3);
            float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW * rotDirW * rotDirW * rotDirW;
            if (relativeGoalDis < omniDirGoalThre) score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW * groupDirW;
            if (score > 0) {
              clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += score;
              clearPathPerGroupNum[groupNum * rotDir + pathList[i % pathNum]]++;
              pathPenaltyPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] += pathPenaltyList[i];
            }
          }
        }

        int selectedGroupID = -1;
        if (preSelectedGroupID >= 0) {
          selectedGroupID = preSelectedGroupID;
        } else {
          float maxScore = 0;
          for (int i = 0; i < 36 * groupNum; i++) {
            int rotDir = int(i / groupNum);
            float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            if (maxScore < clearPathPerGroupScore[i] && ((rotAng * 180.0 / M_PI > minObsAngCW && rotAng * 180.0 / M_PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
              maxScore = clearPathPerGroupScore[i];
              selectedGroupID = i;
            }
          }
        }

        float penaltyScore = 0;
        if (selectedGroupID >= 0) {
          int selectedPathNum = clearPathPerGroupNum[selectedGroupID];
          if (selectedPathNum > 0) {
            penaltyScore = pathPenaltyPerGroupScore[selectedGroupID] / selectedPathNum;
          }

          if (penaltyScore > costHeightThre1) slow.data = 1;
          else if (penaltyScore > costHeightThre2) slow.data = 2;
          else if (selectedPathNum < slowPathNumThre && fabs(selectedGroupID - 129) > slowGroupNumThre) slow.data = 3;
          else slow.data = 0;
          pubSlowDown->publish(slow);
        }

        if (selectedGroupID >= 0) {
          int rotDir = int(selectedGroupID / groupNum);
          float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;

          selectedGroupID = selectedGroupID % groupNum;
          size_t selectedPathLength = startPaths[selectedGroupID]->points.size();
          path.poses.resize(selectedPathLength);
          for (size_t i = 0; i < selectedPathLength; i++) {
            float x = startPaths[selectedGroupID]->points[i].x;
            float y = startPaths[selectedGroupID]->points[i].y;
            float z = startPaths[selectedGroupID]->points[i].z;
            float dis = sqrt(x * x + y * y);

            if (dis <= pathRange / pathScale && dis <= relativeGoalDis / pathScale) {
              path.poses[i].pose.position.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
              path.poses[i].pose.position.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
              path.poses[i].pose.position.z = pathScale * z;
            } else {
              path.poses.resize(i);
              break;
            }
          }

          path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          path.header.frame_id = vehicleFrame;
          pubPath->publish(path);

          // Unified goal-reaching check for all sources (waypoint and action server)
          if (goalSource_ != GoalSource::NONE) {
            float distanceToGoal = sqrt((goalX - vehicleX) * (goalX - vehicleX) +
                                       (goalY - vehicleY) * (goalY - vehicleY));

            // Publish action server feedback if applicable
            if (goalSource_ == GoalSource::ACTION_SERVER && has_active_goal_
                && current_goal_handle_ && current_goal_handle_->is_active()) {
              auto feedback = std::make_shared<NavigateToPose::Feedback>();

              feedback->current_pose.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
              feedback->current_pose.header.frame_id = "map";
              feedback->current_pose.pose.position.x = vehicleX;
              feedback->current_pose.pose.position.y = vehicleY;
              feedback->current_pose.pose.position.z = vehicleZ;

              tf2::Quaternion q;
              q.setRPY(vehicleRoll, vehiclePitch, vehicleYaw);
              feedback->current_pose.pose.orientation.x = q.x();
              feedback->current_pose.pose.orientation.y = q.y();
              feedback->current_pose.pose.orientation.z = q.z();
              feedback->current_pose.pose.orientation.w = q.w();

              feedback->distance_remaining = distanceToGoal;

              auto duration = rclcpp::Duration::from_seconds(odomTime);
              feedback->navigation_time.sec = duration.seconds();
              feedback->navigation_time.nanosec = duration.nanoseconds() % 1000000000;

              current_goal_handle_->publish_feedback(feedback);
            }

            // Goal reached — clear goal (succeeds action server if applicable)
            if (distanceToGoal < goal_tolerance_) {
              RCLCPP_INFO(nh->get_logger(), "Goal reached! Distance: %.2f m", distanceToGoal);
              clearGoal();
            }
          }

          #if PLOTPATHSET == 1
          freePaths->clear();
          for (int i = 0; i < 36 * pathNum; i++) {
            int rotDir = int(i / pathNum);
            float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
            float rotDeg = 10.0 * rotDir;
            if (rotDeg > 180.0) rotDeg -= 360.0;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) || (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) && fabs(joyDir) > 90.0 && dirToVehicle) || 
                !((rotAng * 180.0 / M_PI > minObsAngCW && rotAng * 180.0 / M_PI < minObsAngCCW) || 
                (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) || !checkRotObstacle)) {
              continue;
            }

            if (clearPathList[i] < pointPerPathThre) {
              size_t freePathLength = paths[i % pathNum]->points.size();
              for (size_t j = 0; j < freePathLength; j++) {
                point = paths[i % pathNum]->points[j];

                float x = point.x;
                float y = point.y;
                float z = point.z;

                float dis = sqrt(x * x + y * y);
                if (dis <= pathRange / pathScale && (dis <= (relativeGoalDis + goalClearRange) / pathScale || !pathCropByGoal)) {
                  point.x = pathScale * (cos(rotAng) * x - sin(rotAng) * y);
                  point.y = pathScale * (sin(rotAng) * x + cos(rotAng) * y);
                  point.z = pathScale * z;
                  point.intensity = 1.0;

                  freePaths->push_back(point);
                }
              }
            }
          }

          sensor_msgs::msg::PointCloud2 freePaths2;
          pcl::toROSMsg(*freePaths, freePaths2);
          freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
          freePaths2.header.frame_id = vehicleFrame;
          pubFreePaths->publish(freePaths2);
          #endif
        }

        if (selectedGroupID < 0) {
          if (pathScale >= minPathScale + pathScaleStep) {
            pathScale -= pathScaleStep;
            pathRange = adjacentRange * pathScale / defPathScale;
          } else {
            pathRange -= pathRangeStep;
          }
        } else {
          pathFound = true;
          break;
        }
      }
      pathScale = defPathScale;

      if (!pathFound) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;

        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        path.header.frame_id = vehicleFrame;
        pubPath->publish(path);

        #if PLOTPATHSET == 1
        freePaths->clear();
        sensor_msgs::msg::PointCloud2 freePaths2;
        pcl::toROSMsg(*freePaths, freePaths2);
        freePaths2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        freePaths2.header.frame_id = vehicleFrame;
        pubFreePaths->publish(freePaths2);
        #endif
      }

      /*sensor_msgs::msg::PointCloud2 plannerCloud2;
      pcl::toROSMsg(*plannerCloudCrop, plannerCloud2);
      plannerCloud2.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      plannerCloud2.header.frame_id = vehicleFrame;
      pubLaserCloud->publish(plannerCloud2);*/
    }

    // Staleness watchdog: publish stop path if scan data is stale
    {
      double lastRecv = lastScanReceiveTime.load();
      if (lastRecv > 0 && nh->now().seconds() - lastRecv > 0.5) {
        path.poses.resize(1);
        path.poses[0].pose.position.x = 0;
        path.poses[0].pose.position.y = 0;
        path.poses[0].pose.position.z = 0;
        path.header.stamp = nh->now();
        path.header.frame_id = vehicleFrame;
        pubPath->publish(path);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
            "Scan data stale, publishing stop path");
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
