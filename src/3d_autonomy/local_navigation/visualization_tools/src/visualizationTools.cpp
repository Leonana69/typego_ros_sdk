#include <cmath>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <iomanip>

#include <fstream>
#include <filesystem>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include "visualization_tools/srv/save_explored_areas.hpp"
#include "visualization_tools/srv/save_map.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/polygon_stamped.h>
#include <geometry_msgs/msg/point_stamped.h>

#include "tf2/transform_datatypes.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <pcl/io/ply_io.h>
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


string metricFile;
string trajFile;
string pcdFile;
string mapFile;
string exploredAreaFile;
double overallMapVoxelSize = 0.5;
double exploredAreaVoxelSize = 0.05;
double exploredVolumeVoxelSize = 0.5;
// Default voxel size for /save_map snapshots. 0.1 m matches FAR planner's
// indoor.yaml voxel_dim, giving a planning-ready cloud out of the box.
double mapVoxelSize = 0.1;
double transInterval = 0.2;
double yawInterval = 10.0;
int overallMapDisplayInterval = 2;
int overallMapDisplayCount = 0;
int exploredAreaDisplayInterval = 1;
int exploredAreaDisplayCount = 0;
bool saveMetric = false; 
bool saveTraj = false;
bool savePcd = false;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr overallMapCloudDwz(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredAreaCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr exploredVolumeCloud2(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr trajectory(new pcl::PointCloud<pcl::PointXYZI>());

const int systemDelay = 5;
int systemDelayCount = 0;
bool systemDelayInited = false;
double systemTime = 0;
double systemInitTime = 0;
bool systemInited = false;

float vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;
float exploredVolume = 0, travelingDis = 0, runtime = 0, timeDuration = 0;

pcl::VoxelGrid<pcl::PointXYZ> overallMapDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredAreaDwzFilter;
pcl::VoxelGrid<pcl::PointXYZI> exploredVolumeDwzFilter;

sensor_msgs::msg::PointCloud2 overallMap2;

shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubExploredAreaPtr;

shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pubTrajectoryPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubExploredVolumePtr;

// Publisher that triggers FAR planner's graph_decoder to write the live
// visibility graph (covers the freespace_vgraph subset implicitly) when
// /save_map is invoked with save_vgraph=true. Set in main().
shared_ptr<rclcpp::Publisher<std_msgs::msg::String>> pubSaveFileDirPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTravelingDisPtr;

shared_ptr<rclcpp::Publisher<std_msgs::msg::Float32>> pubTimeDurationPtr;

// IMU gravity offsets received from imu_preintegration_node
double gravityRollOffsetDeg = 0.0;
double gravityPitchOffsetDeg = 0.0;
bool gravityOffsetsReceived = false;

FILE *metricFilePtr = NULL;
FILE *trajFilePtr = NULL;
FILE *pcdFilePtr = NULL;

void gravityOffsetsHandler(const std_msgs::msg::Float64MultiArray::ConstSharedPtr msg)
{
  if (msg->data.size() >= 2) {
    gravityRollOffsetDeg = msg->data[0];
    gravityPitchOffsetDeg = msg->data[1];
    gravityOffsetsReceived = true;
  }
}

string getTimeString()
{
  time_t logTime = time(0);
  tm *ltm = localtime(&logTime);
  return to_string(1900 + ltm->tm_year) + "-" + to_string(1 + ltm->tm_mon) + "-" +
         to_string(ltm->tm_mday) + "-" + to_string(ltm->tm_hour) + "-" +
         to_string(ltm->tm_min) + "-" + to_string(ltm->tm_sec);
}

// Writes a snapshot of the accumulated /registered_scan cloud to disk as
// binary PCD, plus a _gravity.txt companion file when offsets are known.
// If voxelSize > 0, the snapshot is re-filtered at that leaf size first
// so callers get a deterministic, planner-ready resolution regardless of
// the accumulator's running state.
//
// On success: returns true and writes the resolved path back via outPath.
// outNumPoints reports how many points landed in the saved file.
// outMessage is set in both success and failure cases.
bool writeMapSnapshot(const string &basePathWithExt,
                      double voxelSize,
                      string &outPath,
                      uint32_t &outNumPoints,
                      string &outMessage)
{
  outPath = basePathWithExt;
  outNumPoints = 0;

  if (exploredAreaCloud->empty()) {
    outMessage = "registered_scan accumulator is empty; nothing to save.";
    return false;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr snapshot(new pcl::PointCloud<pcl::PointXYZI>());
  *snapshot = *exploredAreaCloud;

  if (voxelSize > 0.0) {
    pcl::VoxelGrid<pcl::PointXYZI> filter;
    filter.setLeafSize(voxelSize, voxelSize, voxelSize);
    filter.setInputCloud(snapshot);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
    filter.filter(*filtered);
    snapshot = filtered;
  }

  if (pcl::io::savePCDFileBinary(outPath, *snapshot) == -1) {
    outMessage = "Failed to save map to " + outPath;
    return false;
  }
  outNumPoints = static_cast<uint32_t>(snapshot->points.size());

  string suffix;
  if (gravityOffsetsReceived) {
    string gravityFilePath = outPath;
    size_t pcdPos = gravityFilePath.rfind(".pcd");
    if (pcdPos != string::npos) {
      gravityFilePath.replace(pcdPos, 4, "_gravity.txt");
    } else {
      gravityFilePath += "_gravity.txt";
    }
    std::ofstream ofs(gravityFilePath);
    if (ofs.is_open()) {
      ofs << std::fixed << std::setprecision(6)
          << gravityRollOffsetDeg << " " << gravityPitchOffsetDeg << std::endl;
    }
    suffix = " (with gravity offsets)";
  } else {
    suffix = " (WARNING: no gravity offsets received)";
  }

  outMessage = "Saved " + std::to_string(outNumPoints) + " points to "
             + outPath + suffix;
  return true;
}

void saveExploredAreasHandler(
  const std::shared_ptr<visualization_tools::srv::SaveExploredAreas::Request> request,
  std::shared_ptr<visualization_tools::srv::SaveExploredAreas::Response> response)
{
  string basePath = request->file_path.empty()
                  ? exploredAreaFile + "_" + getTimeString() + ".pcd"
                  : request->file_path + ".pcd";
  string outPath;
  uint32_t numPoints = 0;
  // Preserve historical behaviour: no extra re-filtering, snapshot is
  // whatever the accumulator currently holds.
  response->success = writeMapSnapshot(basePath, 0.0, outPath, numPoints, response->message);
}

// Backend-agnostic map snapshot suitable for direct use by FAR planner
// (or any consumer that loads PCD). The source cloud (/registered_scan)
// is published in the map frame by both ARISE and lightning-lm, so the
// saved file is already in the planning frame. When request.save_vgraph
// is true (default), also publishes the sibling .vgh path on
// /save_file_dir so FAR planner's graph_decoder writes its visibility
// graph (which includes the freespace_vgraph subset).
void saveMapHandler(
  const std::shared_ptr<visualization_tools::srv::SaveMap::Request> request,
  std::shared_ptr<visualization_tools::srv::SaveMap::Response> response)
{
  string basePath = request->file_path.empty()
                  ? exploredAreaFile + "_" + getTimeString() + ".pcd"
                  : request->file_path + ".pcd";
  double voxel = request->voxel_size > 0.0 ? request->voxel_size : mapVoxelSize;

  string outPath;
  uint32_t numPoints = 0;
  response->success = writeMapSnapshot(basePath, voxel, outPath, numPoints, response->message);
  response->num_points = numPoints;
  response->vgraph_path = "";

  // Trigger the FAR planner vgraph dump alongside the PCD. graph_decoder
  // is a pure subscriber here, so only report a vgraph path when something
  // is actually listening on the save trigger.
  if (response->success && request->save_vgraph && pubSaveFileDirPtr) {
    if (pubSaveFileDirPtr->get_subscription_count() == 0) {
      response->message += "; skipped vgraph (no /save_file_dir subscribers)";
    } else {
      string vgraphPath = outPath;
      size_t pcdPos = vgraphPath.rfind(".pcd");
      if (pcdPos != string::npos) {
        vgraphPath.replace(pcdPos, 4, ".vgh");
      } else {
        vgraphPath += ".vgh";
      }
      std_msgs::msg::String pathMsg;
      pathMsg.data = vgraphPath;
      pubSaveFileDirPtr->publish(pathMsg);
      response->vgraph_path = vgraphPath;
      response->message += "; requested vgraph -> " + vgraphPath;
    }
  }
}

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom)
{
  systemTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w)).getRPY(roll, pitch, yaw);

  float dYaw = fabs(yaw - vehicleYaw);
  if (dYaw > M_PI) dYaw = 2 * M_PI  - dYaw;

  float dx = odom->pose.pose.position.x - vehicleX;
  float dy = odom->pose.pose.position.y - vehicleY;
  float dz = odom->pose.pose.position.z - vehicleZ;
  float dis = sqrt(dx * dx + dy * dy + dz * dz);

  if (!systemDelayInited) {
    vehicleYaw = yaw;
    vehicleX = odom->pose.pose.position.x;
    vehicleY = odom->pose.pose.position.y;
    vehicleZ = odom->pose.pose.position.z;
    return;
  }

  if (systemInited) {
    timeDuration = systemTime - systemInitTime;
    
    std_msgs::msg::Float32 timeDurationMsg;
    timeDurationMsg.data = timeDuration;
    pubTimeDurationPtr->publish(timeDurationMsg);
  }

  if (dis < transInterval && dYaw < yawInterval) {
    return;
  }

  if (!systemInited) {
    dis = 0;
    systemInitTime = systemTime;
    systemInited = true;
  }

  travelingDis += dis;

  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x;
  vehicleY = odom->pose.pose.position.y;
  vehicleZ = odom->pose.pose.position.z;

  if (saveTraj) {
    fprintf(trajFilePtr, "%f %f %f %f %f %f %f\n", vehicleX, vehicleY, vehicleZ, roll, pitch, yaw, timeDuration);
    fflush(trajFilePtr);
  }

  pcl::PointXYZI point;
  point.x = vehicleX;
  point.y = vehicleY;
  point.z = vehicleZ;
  point.intensity = travelingDis;
  trajectory->push_back(point);

  sensor_msgs::msg::PointCloud2 trajectory2;
  pcl::toROSMsg(*trajectory, trajectory2);
  trajectory2.header.stamp = odom->header.stamp;
  trajectory2.header.frame_id = "map";
  pubTrajectoryPtr->publish(trajectory2);
}

void laserCloudHandler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr laserCloudIn)
{
  if (!systemDelayInited) {
    systemDelayCount++;
    if (systemDelayCount > systemDelay) {
      systemDelayInited = true;
    }
  }

  if (!systemInited) {
    return;
  }

  laserCloud->clear();
  pcl::fromROSMsg(*laserCloudIn, *laserCloud);

  if (savePcd) {
    float timeDuration2 = rclcpp::Time(laserCloudIn->header.stamp).seconds() - systemInitTime;
    size_t laserCloudSize = laserCloud->points.size();
    for (size_t i = 0; i < laserCloudSize; i++) {
      fprintf(pcdFilePtr, "%f %f %f %f %f\n", laserCloud->points[i].x, laserCloud->points[i].y, laserCloud->points[i].z, laserCloud->points[i].intensity, timeDuration2);
    }
    fflush(pcdFilePtr);
  }

  *exploredVolumeCloud += *laserCloud;

  exploredVolumeCloud2->clear();
  exploredVolumeDwzFilter.setInputCloud(exploredVolumeCloud);
  exploredVolumeDwzFilter.filter(*exploredVolumeCloud2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr tempCloud = exploredVolumeCloud;
  exploredVolumeCloud = exploredVolumeCloud2;
  exploredVolumeCloud2 = tempCloud;

  exploredVolume = exploredVolumeVoxelSize * exploredVolumeVoxelSize * 
                   exploredVolumeVoxelSize * exploredVolumeCloud->points.size();

  *exploredAreaCloud += *laserCloud;

  exploredAreaDisplayCount++;
  if (exploredAreaDisplayCount >= 10 * exploredAreaDisplayInterval) {
    exploredAreaCloud2->clear();
    exploredAreaDwzFilter.setInputCloud(exploredAreaCloud);
    exploredAreaDwzFilter.filter(*exploredAreaCloud2);

    tempCloud = exploredAreaCloud;
    exploredAreaCloud = exploredAreaCloud2;
    exploredAreaCloud2 = tempCloud;

    sensor_msgs::msg::PointCloud2 exploredArea2;
    pcl::toROSMsg(*exploredAreaCloud, exploredArea2);
    exploredArea2.header.stamp = laserCloudIn->header.stamp;
    exploredArea2.header.frame_id = "map";
    pubExploredAreaPtr->publish(exploredArea2);

    exploredAreaDisplayCount = 0;
  }

  if (saveMetric) {
    fprintf(metricFilePtr, "%f %f %f %f\n", exploredVolume, travelingDis, runtime, timeDuration);
    fflush(metricFilePtr);
  }

  std_msgs::msg::Float32 exploredVolumeMsg;
  exploredVolumeMsg.data = exploredVolume;
  pubExploredVolumePtr->publish(exploredVolumeMsg);

  std_msgs::msg::Float32 travelingDisMsg;
  travelingDisMsg.data = travelingDis;
  pubTravelingDisPtr->publish(travelingDisMsg);
}

void runtimeHandler(const std_msgs::msg::Float32::ConstSharedPtr runtimeIn)
{
  runtime = runtimeIn->data;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("visualizationTools");

  nh->declare_parameter<std::string>("metricFile", metricFile);
  nh->declare_parameter<std::string>("trajFile", trajFile);
  nh->declare_parameter<std::string>("pcdFile", pcdFile);
  nh->declare_parameter<std::string>("mapFile", mapFile);
  nh->declare_parameter<std::string>("exploredAreaFile", exploredAreaFile);
  nh->declare_parameter<double>("overallMapVoxelSize", overallMapVoxelSize);
  nh->declare_parameter<double>("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->declare_parameter<double>("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->declare_parameter<double>("mapVoxelSize", mapVoxelSize);
  nh->declare_parameter<double>("transInterval", transInterval);
  nh->declare_parameter<double>("yawInterval", yawInterval);
  nh->declare_parameter<int>("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->declare_parameter<int>("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  nh->declare_parameter<bool>("saveMetric", saveMetric);
  nh->declare_parameter<bool>("saveTraj", saveTraj);
  nh->declare_parameter<bool>("savePcd", savePcd);

  nh->get_parameter("metricFile", metricFile);
  nh->get_parameter("trajFile", trajFile);
  nh->get_parameter("pcdFile", pcdFile);
  nh->get_parameter("mapFile", mapFile);
  nh->get_parameter("exploredAreaFile", exploredAreaFile);
  nh->get_parameter("overallMapVoxelSize", overallMapVoxelSize);
  nh->get_parameter("exploredAreaVoxelSize", exploredAreaVoxelSize);
  nh->get_parameter("exploredVolumeVoxelSize", exploredVolumeVoxelSize);
  nh->get_parameter("mapVoxelSize", mapVoxelSize);
  nh->get_parameter("transInterval", transInterval);
  nh->get_parameter("yawInterval", yawInterval);
  nh->get_parameter("overallMapDisplayInterval", overallMapDisplayInterval);
  nh->get_parameter("exploredAreaDisplayInterval", exploredAreaDisplayInterval);
  nh->get_parameter("saveMetric", saveMetric);
  nh->get_parameter("saveTraj", saveTraj);
  nh->get_parameter("savePcd", savePcd);

  // Paths arrive fully resolved from visualization_tools.launch, which uses
  // $(find-pkg-prefix vehicle_simulator). They are used verbatim.

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>("/state_estimation", 5, odometryHandler);

  auto subLaserCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/registered_scan", rclcpp::SensorDataQoS(), laserCloudHandler);

  auto subRuntime = nh->create_subscription<std_msgs::msg::Float32>("/runtime", 5, runtimeHandler);

  auto gravity_qos = rclcpp::QoS(1).transient_local();
  auto subGravityOffsets = nh->create_subscription<std_msgs::msg::Float64MultiArray>(
      "/gravity_offsets", gravity_qos, gravityOffsetsHandler);

  auto pubOverallMap = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/overall_map", 5);

  pubExploredAreaPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/explored_areas", 5);

  pubTrajectoryPtr = nh->create_publisher<sensor_msgs::msg::PointCloud2>("/trajectory", 5);
  
  pubExploredVolumePtr = nh->create_publisher<std_msgs::msg::Float32>("/explored_volume", 5);

  pubTravelingDisPtr = nh->create_publisher<std_msgs::msg::Float32>("/traveling_distance", 5);

  pubTimeDurationPtr = nh->create_publisher<std_msgs::msg::Float32>("/time_duration", 5);

  auto saveExploredAreasService =
    nh->create_service<visualization_tools::srv::SaveExploredAreas>("/save_explored_areas", saveExploredAreasHandler);
  (void)saveExploredAreasService;

  // Publisher to FAR planner's graph_decoder save trigger. QoS(5) matches
  // graph_decoder's subscription so messages aren't dropped on connect.
  pubSaveFileDirPtr =
    nh->create_publisher<std_msgs::msg::String>("/save_file_dir", rclcpp::QoS(5));

  // Backend-agnostic planning-grade map saver (ARISE + lightning-lm).
  auto saveMapService =
    nh->create_service<visualization_tools::srv::SaveMap>("/save_map", saveMapHandler);
  (void)saveMapService;

  overallMapDwzFilter.setLeafSize(overallMapVoxelSize, overallMapVoxelSize, overallMapVoxelSize);
  exploredAreaDwzFilter.setLeafSize(exploredAreaVoxelSize, exploredAreaVoxelSize, exploredAreaVoxelSize);
  exploredVolumeDwzFilter.setLeafSize(exploredVolumeVoxelSize, exploredVolumeVoxelSize, exploredVolumeVoxelSize);

  size_t overallMapCloudDwzSize = 0;
  if (std::filesystem::exists(mapFile)) {
    pcl::PLYReader ply_reader;
    if (ply_reader.read(mapFile, *overallMapCloud) == -1) {
      RCLCPP_WARN(nh->get_logger(), "Failed to read map PLY file: %s", mapFile.c_str());
    } else {
      overallMapCloudDwz->clear();
      overallMapDwzFilter.setInputCloud(overallMapCloud);
      overallMapDwzFilter.filter(*overallMapCloudDwz);
      overallMapCloud->clear();
      overallMapCloudDwzSize = overallMapCloudDwz->points.size();
      pcl::toROSMsg(*overallMapCloudDwz, overallMap2);
    }
  } else {
    RCLCPP_INFO(nh->get_logger(), "No map PLY file at %s, skipping /overall_map from visualization_tools.", mapFile.c_str());
  }

  string timeString = getTimeString();

  metricFile += "_" + timeString + ".txt";
  trajFile += "_" + timeString + ".txt";
  pcdFile += "_" + timeString + ".txt";
  if (saveMetric) {
    metricFilePtr = fopen(metricFile.c_str(), "w");
    if (!metricFilePtr) {
      RCLCPP_WARN(nh->get_logger(), "Failed to open metric file: %s", metricFile.c_str());
      saveMetric = false;
    }
  }
  if (saveTraj) {
    trajFilePtr = fopen(trajFile.c_str(), "w");
    if (!trajFilePtr) {
      RCLCPP_WARN(nh->get_logger(), "Failed to open trajectory file: %s", trajFile.c_str());
      saveTraj = false;
    }
  }
  if (savePcd) {
    pcdFilePtr = fopen(pcdFile.c_str(), "w");
    if (!pcdFilePtr) {
      RCLCPP_WARN(nh->get_logger(), "Failed to open pcd file: %s", pcdFile.c_str());
      savePcd = false;
    }
  }

  rclcpp::Rate rate(100);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);
    overallMapDisplayCount++;
    if (overallMapDisplayCount >= 100 * overallMapDisplayInterval) {
      if (overallMapCloudDwzSize > 0) {
        overallMap2.header.stamp = rclcpp::Time(static_cast<uint64_t>(systemTime * 1e9));
        overallMap2.header.frame_id = "map";
        pubOverallMap->publish(overallMap2);
      }

      overallMapDisplayCount = 0;
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  if (saveMetric) fclose(metricFilePtr);
  if (saveTraj) fclose(trajFilePtr);
  if (savePcd) fclose(pcdFilePtr);

  RCLCPP_INFO(nh->get_logger(), "Exploration metrics and vehicle trajectory are saved in 'src/vehicle_simulator/log'.");

  return 0;
}
