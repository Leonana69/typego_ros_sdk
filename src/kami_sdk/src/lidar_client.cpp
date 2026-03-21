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
#include "typego_interface/msg/custom_msg.hpp"
#include "typego_interface/msg/custom_point.hpp"

class LidarClientNode : public rclcpp::Node {
public:
    LidarClientNode() : Node("lidar_client", typego_sdk::get_namespace_from_env()) {
        // Declare parameters
        this->declare_parameter<std::string>("xfer_format", "PointCloud2");  // "PointCloud2", "CustomMsg", or "None"
        this->declare_parameter<bool>("publish_scan", true);

        std::string xfer_format = this->get_parameter("xfer_format").as_string();
        publish_scan_ = this->get_parameter("publish_scan").as_bool();

        // Set transfer format
        if (xfer_format == "CustomMsg") {
            use_custom_msg_ = true;
            RCLCPP_INFO(this->get_logger(), "Using Livox CustomMsg format");
        } else if (xfer_format == "PointCloud2") {
            use_custom_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "Using PointCloud2 format");
        } else {
            use_custom_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "Point cloud format: None - only laser scan will be published");
        }

        // Subscribe to /livox/lidar topic
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/livox/lidar", 10,
            std::bind(&LidarClientNode::pointcloud_callback, this, std::placeholders::_1));

        // Create CustomMsg publisher if needed
        if (use_custom_msg_) {
            custom_publisher_ = this->create_publisher<typego_interface::msg::CustomMsg>("livox/lidar_custom", 10);
        }

        if (publish_scan_) {
            laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        }

        init_lidar_link_tf();

        RCLCPP_INFO(this->get_logger(),
                    "Lidar Client initialized: subscribing to /livox/lidar, publishing to scan");
    }

private:
    void init_lidar_link_tf() {
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "lidar_link";
        t.transform.translation.x = 0.20;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;

        geometry_msgs::msg::TransformStamped t2;
        t2.header.stamp = now();
        t2.header.frame_id = "lidar_link";
        t2.child_frame_id = "livox_frame";
        t2.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform({t, t2});
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
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

        // Publish CustomMsg if enabled
        if (use_custom_msg_) {
            publish_custom_msg(msg, x_offset, y_offset, z_offset);
        }

        if (publish_scan_) {
            publish_laser_scan(msg, x_offset, y_offset, z_offset);
        }
    }

    void publish_custom_msg(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                            int x_offset, int y_offset, int z_offset) {
        uint32_t num_points = msg->width * msg->height;

        typego_interface::msg::CustomMsg custom_msg;
        custom_msg.header.stamp = msg->header.stamp;
        custom_msg.header.frame_id = "lidar_link";
        custom_msg.timebase = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
                              static_cast<uint64_t>(msg->header.stamp.nanosec);
        custom_msg.point_num = num_points;
        custom_msg.lidar_id = 0;

        custom_msg.points.resize(num_points);

        const uint8_t* data_ptr = msg->data.data();
        const uint32_t point_step = msg->point_step;
        for (uint32_t i = 0; i < num_points; ++i) {
            const uint8_t* point_ptr = data_ptr + i * point_step;

            auto& pt = custom_msg.points[i];
            pt.x = *reinterpret_cast<const float*>(point_ptr + x_offset);
            pt.y = *reinterpret_cast<const float*>(point_ptr + y_offset);
            pt.z = *reinterpret_cast<const float*>(point_ptr + z_offset);

            // Approximate reflectivity from distance
            float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            pt.reflectivity = static_cast<uint8_t>(std::min(dist * 12.75f, 255.0f));
            pt.tag = 0;
            pt.line = 0;
            pt.offset_time = 0;
        }

        custom_publisher_->publish(std::move(custom_msg));
    }

    void publish_laser_scan(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                            int x_offset, int y_offset, int z_offset) {
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = msg->header.stamp;
        scan_msg.header.frame_id = "lidar_link";
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0f;  // 1 degree resolution
        scan_msg.scan_time = 0.1f;
        scan_msg.time_increment = scan_msg.scan_time / (360.0f);
        scan_msg.range_min = 0.1f;
        scan_msg.range_max = 20.0f;

        int num_beams = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) /
                                         scan_msg.angle_increment);
        scan_msg.ranges.assign(num_beams, std::numeric_limits<float>::infinity());

        // Filter points: only use points near z=0 (ground plane)
        constexpr float z_threshold = 0.1f;

        const uint8_t* data_ptr = msg->data.data();
        for (uint32_t i = 0; i < msg->width * msg->height; ++i) {
            const uint8_t* point_ptr = data_ptr + i * msg->point_step;

            float x = *reinterpret_cast<const float*>(point_ptr + x_offset);
            float y = *reinterpret_cast<const float*>(point_ptr + y_offset);
            float z = *reinterpret_cast<const float*>(point_ptr + z_offset);

            // Filter out points far from ground plane
            if (std::abs(z) > z_threshold) continue;

            float angle = std::atan2(y, x);
            float range = std::hypot(x, y);

            // Skip if out of range
            if (range < scan_msg.range_min || range > scan_msg.range_max) continue;

            int idx = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
            if (idx >= 0 && idx < num_beams && range < scan_msg.ranges[idx]) {
                scan_msg.ranges[idx] = range;
            }
        }

        laserscan_publisher_->publish(scan_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
    rclcpp::Publisher<typego_interface::msg::CustomMsg>::SharedPtr custom_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;

    bool use_custom_msg_{false};
    bool publish_scan_{true};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarClientNode>());
    rclcpp::shutdown();
    return 0;
}
