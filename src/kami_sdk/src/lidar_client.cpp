#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

#include "typego_sdk/namespace_utils.hpp"

class LidarClientNode : public rclcpp::Node {
public:
    LidarClientNode() : Node("lidar_client", typego_sdk::get_namespace_from_env()) {
        // Subscribe to /livox/lidar topic
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&LidarClientNode::pointcloud_callback, this, std::placeholders::_1));
        
        // Publish LaserScan
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        init_lidar_link();

        RCLCPP_INFO(this->get_logger(), 
                    "Lidar Client initialized: subscribing to /livox/lidar, publishing to scan");
    }

private:
    void init_lidar_link() {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "lidar_link";
        t.transform.translation.x = 0.20;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(t);
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // Convert PointCloud2 to LaserScan
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = msg->header.stamp;
        scan_msg.header.frame_id = "lidar_link";
        
        // LaserScan parameters
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0f;  // 1 degree resolution
        scan_msg.scan_time = 0.1f;  // 10 Hz
        scan_msg.range_min = 0.1f;
        scan_msg.range_max = 20.0f;
        scan_msg.time_increment = scan_msg.scan_time / 360.0f;  // Time between measurements

        int num_beams = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) /
                                         scan_msg.angle_increment);
        scan_msg.ranges.assign(num_beams, std::numeric_limits<float>::infinity());
        scan_msg.intensities.assign(num_beams, 0.0f);

        // Find x, y, z field offsets
        int x_offset = -1, y_offset = -1, z_offset = -1;
        for (const auto& field : msg->fields) {
            if (field.name == "x") x_offset = field.offset;
            else if (field.name == "y") y_offset = field.offset;
            else if (field.name == "z") z_offset = field.offset;
        }

        if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
            RCLCPP_WARN(this->get_logger(), "PointCloud2 missing x, y, or z fields");
            return;
        }

        // Process each point
        const uint8_t* data_ptr = msg->data.data();
        for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t* point_ptr = data_ptr + i * msg->point_step;
            
            float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(point_ptr + z_offset);

            // Filter: only use points near z=0 for 2D lidar (within 0.1m)
            if (std::abs(z) > 0.2f) continue;

            // Calculate angle and range
            float angle = std::atan2(y, x);
            float range = std::hypot(x, y);

            // Check if range is within limits
            if (range < scan_msg.range_min || range > scan_msg.range_max) continue;

            // Find the corresponding beam index
            int idx = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
            
            // Keep the minimum range for each beam (closest point)
            if (idx >= 0 && idx < num_beams && range < scan_msg.ranges[idx]) {
                scan_msg.ranges[idx] = range;
            }
        }

        // Publish LaserScan
        laserscan_publisher_->publish(scan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClientNode>());
    rclcpp::shutdown();
    return 0;
}
