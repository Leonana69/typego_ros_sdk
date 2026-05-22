#include <cmath>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <mutex>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/clock.hpp"
#include "builtin_interfaces/msg/time.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/twist.hpp>


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


string vehicleFrame = "vehicle";
double sensorOffsetX = 0;
double sensorOffsetY = 0;
int pubSkipNum = 1;
int pubSkipCount = 0;
bool twoWayDrive = true;
double lookAheadDis = 0.5;
double yawRateGain = 7.5;
double stopYawRateGain = 7.5;
double maxYawRate = 45.0;
double maxSpeed = 1.0;
double maxAccel = 1.0;
double switchTimeThre = 1.0;
double dirDiffThre = 0.1;
double omniDirGoalThre = 1.0;
double omniDirDiffThre = 1.5;
double stopDisThre = 0.2;
double slowDwnDisThre = 1.0;
bool useInclRateToSlow = false;
double inclRateThre = 120.0;
double slowRate1 = 0.25;
double slowRate2 = 0.5;
double slowRate3 = 0.75;
double slowTime1 = 2.0;
double slowTime2 = 2.0;
bool useInclToStop = false;
double inclThre = 45.0;
double stopTime = 5.0;
bool noRotAtStop = false;
bool noRotAtGoal = true;
bool omniDirDrive = false;
bool manualMode = false;
bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
int oneWayTurnSign = 0;

float joySpeed = 0;
float joySpeedRaw = 0;
float joyYaw = 0;
float joyManualFwd = 0;
float joyManualLeft = 0;
float joyManualYaw = 0;
std::atomic<int> safetyStop{0};
std::atomic<int> slowDown{0};

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 0;
float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleXRec = 0;
float vehicleYRec = 0;
float vehicleZRec = 0;
float vehicleRollRec = 0;
float vehiclePitchRec = 0;
float vehicleYawRec = 0;

float vehicleYawRate = 0;
float vehicleSpeed = 0;

double odomTime = 0;
double joyTime = 0;
double slowInitTime = 0;
double stopInitTime = 0.0;
int pathPointID = 0;
std::atomic<bool> pathInit{false};
bool navFwd = true;
double switchTime = 0;

nav_msgs::msg::Path path;
rclcpp::Node::SharedPtr nh;
std::mutex poseMtx;
std::atomic<double> lastOdomReceiveTime{0};

void odomHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odomIn)
{
  std::lock_guard<std::mutex> lock(poseMtx);
  lastOdomReceiveTime.store(nh->now().seconds());
  odomTime = rclcpp::Time(odomIn->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odomIn->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odomIn->pose.pose.position.x - cos(yaw) * sensorOffsetX + sin(yaw) * sensorOffsetY;
  vehicleY = odomIn->pose.pose.position.y - sin(yaw) * sensorOffsetX - cos(yaw) * sensorOffsetY;
  vehicleZ = odomIn->pose.pose.position.z;

  if ((fabs(roll) > inclThre * M_PI / 180.0 || fabs(pitch) > inclThre * M_PI / 180.0) && useInclToStop) {
    stopInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }

  if ((fabs(odomIn->twist.twist.angular.x) > inclRateThre * M_PI / 180.0 || fabs(odomIn->twist.twist.angular.y) > inclRateThre * M_PI / 180.0) && useInclRateToSlow) {
    slowInitTime = rclcpp::Time(odomIn->header.stamp).seconds();
  }
}

void pathHandler(const nav_msgs::msg::Path::ConstSharedPtr pathIn)
{
  std::lock_guard<std::mutex> lock(poseMtx);
  size_t pathSize = pathIn->poses.size();
  path.poses.resize(pathSize);
  for (size_t i = 0; i < pathSize; i++) {
    path.poses[i].pose.position.x = pathIn->poses[i].pose.position.x;
    path.poses[i].pose.position.y = pathIn->poses[i].pose.position.y;
    path.poses[i].pose.position.z = pathIn->poses[i].pose.position.z;
  }

  vehicleXRec = vehicleX;
  vehicleYRec = vehicleY;
  vehicleZRec = vehicleZ;
  vehicleRollRec = vehicleRoll;
  vehiclePitchRec = vehiclePitch;
  vehicleYawRec = vehicleYaw;

  pathPointID = 0;
  pathInit = true;
}

static float clampedAutonomySpeed()
{
  float s = static_cast<float>(autonomySpeed / maxSpeed);
  if (s < 0.0f) s = 0.0f;
  else if (s > 1.0f) s = 1.0f;
  return s;
}

void joystickHandler(const sensor_msgs::msg::Joy::ConstSharedPtr joy)
{
  joyTime = nh->now().seconds();
  joySpeedRaw = sqrt(joy->axes[3] * joy->axes[3] + joy->axes[4] * joy->axes[4]);

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

  joyYaw = joy->axes[3];
  if (joySpeed == 0 && noRotAtStop) joyYaw = 0;

  if (joy->axes[4] < 0 && !twoWayDrive) {
    joySpeed = 0;
    joyYaw = 0;
  }

  joyManualFwd = joy->axes[4];
  joyManualLeft = joy->axes[3];
  joyManualYaw = joy->axes[0];

  if (joy->axes[5] > -0.1) {
    manualMode = false;
  } else {
    manualMode = true;
  }
}

void speedHandler(const std_msgs::msg::Float32::ConstSharedPtr speed)
{
  double speedTime = nh->now().seconds();
  if (autonomyMode && speedTime - joyTime > joyToSpeedDelay && joySpeedRaw == 0) {
    joySpeed = speed->data / maxSpeed;

    if (joySpeed < 0) joySpeed = 0;
    else if (joySpeed > 1.0) joySpeed = 1.0;
  }
}

void stopHandler(const std_msgs::msg::Int8::ConstSharedPtr stop)
{
  safetyStop = stop->data;
}

void slowDownHandler(const std_msgs::msg::Int8::ConstSharedPtr slow)
{
  slowDown = slow->data;
}

void speedConfigHandler(const std_msgs::msg::Float32MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() < 2) return;
  double newMax = msg->data[0];
  double newAutonomy = msg->data[1];
  if (newMax <= 0 || newAutonomy <= 0) return;

  maxSpeed = newMax;
  autonomySpeed = newAutonomy;
  if (autonomyMode) {
    joySpeed = clampedAutonomySpeed();
  }
  RCLCPP_INFO(nh->get_logger(), "Speed config updated: maxSpeed=%.3f, autonomySpeed=%.3f",
              maxSpeed, autonomySpeed);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("pathFollower");

  nh->declare_parameter<string>("vehicleFrame", vehicleFrame);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<int>("pubSkipNum", pubSkipNum);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("lookAheadDis", lookAheadDis);
  nh->declare_parameter<double>("yawRateGain", yawRateGain);
  nh->declare_parameter<double>("stopYawRateGain", stopYawRateGain);
  nh->declare_parameter<double>("maxYawRate", maxYawRate);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("maxAccel", maxAccel);
  nh->declare_parameter<double>("switchTimeThre", switchTimeThre);
  nh->declare_parameter<double>("dirDiffThre", dirDiffThre);
  nh->declare_parameter<double>("omniDirGoalThre", omniDirGoalThre);
  nh->declare_parameter<double>("omniDirDiffThre", omniDirDiffThre);
  nh->declare_parameter<double>("stopDisThre", stopDisThre);
  nh->declare_parameter<double>("slowDwnDisThre", slowDwnDisThre);
  nh->declare_parameter<bool>("useInclRateToSlow", useInclRateToSlow);
  nh->declare_parameter<double>("inclRateThre", inclRateThre);
  nh->declare_parameter<double>("slowRate1", slowRate1);
  nh->declare_parameter<double>("slowRate2", slowRate2);
  nh->declare_parameter<double>("slowRate3", slowRate3);
  nh->declare_parameter<double>("slowTime1", slowTime1);
  nh->declare_parameter<double>("slowTime2", slowTime2);
  nh->declare_parameter<bool>("useInclToStop", useInclToStop);
  nh->declare_parameter<double>("inclThre", inclThre);
  nh->declare_parameter<double>("stopTime", stopTime);
  nh->declare_parameter<bool>("noRotAtStop", noRotAtStop);
  nh->declare_parameter<bool>("noRotAtGoal", noRotAtGoal);
  nh->declare_parameter<bool>("omniDirDrive", omniDirDrive);
  nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);

  nh->get_parameter("vehicleFrame", vehicleFrame);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("pubSkipNum", pubSkipNum);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("lookAheadDis", lookAheadDis);
  nh->get_parameter("yawRateGain", yawRateGain);
  nh->get_parameter("stopYawRateGain", stopYawRateGain);
  nh->get_parameter("maxYawRate", maxYawRate);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("maxAccel", maxAccel);
  nh->get_parameter("switchTimeThre", switchTimeThre);
  nh->get_parameter("dirDiffThre", dirDiffThre);
  nh->get_parameter("omniDirGoalThre", omniDirGoalThre);
  nh->get_parameter("omniDirDiffThre", omniDirDiffThre);
  nh->get_parameter("stopDisThre", stopDisThre);
  nh->get_parameter("slowDwnDisThre", slowDwnDisThre);
  nh->get_parameter("useInclRateToSlow", useInclRateToSlow);
  nh->get_parameter("inclRateThre", inclRateThre);
  nh->get_parameter("slowRate1", slowRate1);
  nh->get_parameter("slowRate2", slowRate2);
  nh->get_parameter("slowRate3", slowRate3);
  nh->get_parameter("slowTime1", slowTime1);
  nh->get_parameter("slowTime2", slowTime2);
  nh->get_parameter("useInclToStop", useInclToStop);
  nh->get_parameter("inclThre", inclThre);
  nh->get_parameter("stopTime", stopTime);
  nh->get_parameter("noRotAtStop", noRotAtStop);
  nh->get_parameter("noRotAtGoal", noRotAtGoal);
  nh->get_parameter("omniDirDrive", omniDirDrive);
  nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);

  // Validate safety-critical parameters
  if (maxSpeed <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "maxSpeed must be > 0, got %f", maxSpeed);
    rclcpp::shutdown();
    return 1;
  }
  if (maxAccel <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "maxAccel must be > 0, got %f", maxAccel);
    rclcpp::shutdown();
    return 1;
  }
  if (lookAheadDis <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "lookAheadDis must be > 0, got %f", lookAheadDis);
    rclcpp::shutdown();
    return 1;
  }
  if (slowDwnDisThre <= 0) {
    RCLCPP_FATAL(nh->get_logger(), "slowDwnDisThre must be > 0, got %f", slowDwnDisThre);
    rclcpp::shutdown();
    return 1;
  }

  auto subOdom = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odomHandler);

  auto subPath = nh->create_subscription<nav_msgs::msg::Path>("/path", 5, pathHandler);

  auto subJoystick = nh->create_subscription<sensor_msgs::msg::Joy>("/joy", 5, joystickHandler);

  auto subSpeed = nh->create_subscription<std_msgs::msg::Float32>("/speed", 5, speedHandler);

  auto subStop = nh->create_subscription<std_msgs::msg::Int8>("/stop", 5, stopHandler);

  auto subSlowDown = nh->create_subscription<std_msgs::msg::Int8>("/slow_down", 5, slowDownHandler);

  auto subSpeedConfig = nh->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/speed_config", rclcpp::QoS(1).transient_local(), speedConfigHandler);

  auto pubSpeed = nh->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 5);
  geometry_msgs::msg::Twist cmd_vel;

  if (autonomyMode) {
    joySpeed = clampedAutonomySpeed();
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    if (pathInit.load()) {
      // Snapshot pose state for thread-safety
      float vehicleX, vehicleY, vehicleYaw;
      float vehicleXRec, vehicleYRec, vehicleYawRec;
      double odomTime, slowInitTime, stopInitTime;
      {
        std::lock_guard<std::mutex> lock(poseMtx);
        vehicleX = ::vehicleX;
        vehicleY = ::vehicleY;
        vehicleYaw = ::vehicleYaw;
        vehicleXRec = ::vehicleXRec;
        vehicleYRec = ::vehicleYRec;
        vehicleYawRec = ::vehicleYawRec;
        odomTime = ::odomTime;
        slowInitTime = ::slowInitTime;
        stopInitTime = ::stopInitTime;
      }

      // Staleness watchdog: zero cmd_vel if odometry data is stale
      if (lastOdomReceiveTime.load() > 0 &&
          nh->now().seconds() - lastOdomReceiveTime.load() > 0.5) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        pubSpeed->publish(cmd_vel);
        RCLCPP_WARN_THROTTLE(nh->get_logger(), *nh->get_clock(), 2000,
            "Odometry data stale, zeroing cmd_vel");
        status = rclcpp::ok();
        rate.sleep();
        continue;
      }

      float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
                        + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
      float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
                        + cos(vehicleYawRec) * (vehicleY - vehicleYRec);

      size_t pathSize = path.poses.size();
      if (pathSize == 0) {
        status = rclcpp::ok();
        rate.sleep();
        continue;
      }
      float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
      float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
      float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);

      float disX, disY, dis;
      while (static_cast<size_t>(pathPointID) < pathSize - 1) {
        disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
        disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
        dis = sqrt(disX * disX + disY * disY);
        if (dis < lookAheadDis) {
          pathPointID++;
        } else {
          break;
        }
      }

      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      float pathDir = atan2(disY, disX);

      float dirDiff = vehicleYaw - vehicleYawRec - pathDir;
      if (dirDiff > M_PI) dirDiff -= 2 * M_PI;
      else if (dirDiff < -M_PI) dirDiff += 2 * M_PI;
      if (dirDiff > M_PI) dirDiff -= 2 * M_PI;
      else if (dirDiff < -M_PI) dirDiff += 2 * M_PI;

      if (!twoWayDrive) {
        const float turnLockEnter = 170.0f * M_PI / 180.0f;
        const float turnLockExit = 135.0f * M_PI / 180.0f;
        if (fabs(dirDiff) > turnLockEnter) {
          if (oneWayTurnSign == 0) {
            oneWayTurnSign = dirDiff >= 0.0f ? 1 : -1;
          }
          dirDiff = oneWayTurnSign > 0 ? fabs(dirDiff) : -fabs(dirDiff);
        } else if (fabs(dirDiff) < turnLockExit) {
          oneWayTurnSign = 0;
        }
      }

      if (twoWayDrive && !omniDirDrive) {
        double time = nh->now().seconds();
        if (fabs(dirDiff) > M_PI / 2 && navFwd && time - switchTime > switchTimeThre) {
          navFwd = false;
          switchTime = time;
        } else if (fabs(dirDiff) < M_PI / 2 && !navFwd && time - switchTime > switchTimeThre) {
          navFwd = true;
          switchTime = time;
        }
      }

      float joySpeed2 = maxSpeed * joySpeed;
      if (!navFwd) {
        dirDiff += M_PI;
        if (dirDiff > M_PI) dirDiff -= 2 * M_PI;
        joySpeed2 *= -1;
      }

      if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0) vehicleYawRate = -stopYawRateGain * dirDiff;
      else vehicleYawRate = -yawRateGain * dirDiff;

      if (vehicleYawRate > maxYawRate * M_PI / 180.0) vehicleYawRate = maxYawRate * M_PI / 180.0;
      else if (vehicleYawRate < -maxYawRate * M_PI / 180.0) vehicleYawRate = -maxYawRate * M_PI / 180.0;

      if (joySpeed2 == 0 && !autonomyMode) {
        vehicleYawRate = maxYawRate * joyYaw * M_PI / 180.0;
      } else if (pathSize <= 1 || (dis < stopDisThre && noRotAtGoal)) {
        vehicleYawRate = 0;
      }

      if (pathSize <= 1) {
        joySpeed2 = 0;
      } else if (endDis / slowDwnDisThre < joySpeed) {
        joySpeed2 *= endDis / slowDwnDisThre;
      }

      float joySpeed3 = joySpeed2;
      if ((odomTime < slowInitTime + slowTime1 && slowInitTime > 0) || slowDown == 1) joySpeed3 *= slowRate1;
      else if ((odomTime < slowInitTime + slowTime1 + slowTime2 && slowInitTime > 0) || slowDown == 2) joySpeed3 *= slowRate2;
      else if (slowDown == 3) joySpeed3 *= slowRate3;

      // In omnidirectional drive the robot translates toward the lookahead point
      // regardless of heading error; rotation toward that direction is handled
      // independently by vehicleYawRate above. Otherwise the robot must be
      // (near-)aligned with the path before it is allowed to translate.
      bool moveAllowed;
      if (omniDirDrive) {
        moveAllowed = dis > stopDisThre;
      } else {
        moveAllowed = (fabs(dirDiff) < dirDiffThre ||
                       (dis < omniDirGoalThre && fabs(dirDiff) < omniDirDiffThre)) && dis > stopDisThre;
      }
      if (moveAllowed) {
        if (vehicleSpeed < joySpeed3) vehicleSpeed += maxAccel / 100.0;
        else if (vehicleSpeed > joySpeed3) vehicleSpeed -= maxAccel / 100.0;
      } else {
        if (vehicleSpeed > 0) vehicleSpeed -= maxAccel / 100.0;
        else if (vehicleSpeed < 0) vehicleSpeed += maxAccel / 100.0;
      }

      if (odomTime < stopInitTime + stopTime && stopInitTime > 0) {
        vehicleSpeed = 0;
        vehicleYawRate = 0;
      }

      if (safetyStop >= 1) vehicleSpeed = 0;
      if (safetyStop >= 2) vehicleYawRate = 0;

      pubSkipCount--;
      if (pubSkipCount < 0) {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = vehicleYawRate;

        if (fabs(vehicleSpeed) > maxAccel / 100.0) {
          if (omniDirDrive || omniDirGoalThre > 0) {
            // Decompose the path-frame translation speed into the current body
            // frame. dirDiff is the heading error, so this is the world-frame
            // velocity toward the lookahead point expressed in body axes.
            cmd_vel.linear.x = cos(dirDiff) * vehicleSpeed;
            cmd_vel.linear.y = -sin(dirDiff) * vehicleSpeed;
          } else {
            cmd_vel.linear.x = vehicleSpeed;
          }
        }

        if (manualMode) {
          cmd_vel.linear.x = maxSpeed * joyManualFwd;
          if (omniDirGoalThre > 0) cmd_vel.linear.y = maxSpeed / 2.0 * joyManualLeft;
          cmd_vel.angular.z = maxYawRate * M_PI / 180.0 * joyManualYaw;
        }

        pubSpeed->publish(cmd_vel);
        pubSkipCount = pubSkipNum;
      }
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}
