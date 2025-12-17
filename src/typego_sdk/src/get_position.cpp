#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <cmath>
#include <fstream>
#include <filesystem>
#include <thread>
#include <chrono>
#include <nlohmann/json.hpp>

#include "typego_sdk/namespace_utils.hpp"

int main(int argc, char** argv) {
    // Check for command-line argument
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <target_directory>" << std::endl;
        return 1;
    }

    std::string target_dir = argv[1];
    
    // Initialize ROS 2
    rclcpp::init(argc, argv);
    
    // Create a node
    auto node = std::make_shared<rclcpp::Node>("get_position", typego_sdk::get_namespace_from_env());
    
    // Create TF buffer and listener
    tf2_ros::Buffer tf_buffer(node->get_clock());
    tf2_ros::TransformListener tf_listener(tf_buffer);
    
    RCLCPP_INFO(node->get_logger(), "Waiting for transform from map to base_link...");
    
    // Wait for transform with retries
    geometry_msgs::msg::TransformStamped transform;
    bool transform_found = false;
    const int max_retries = 50;  // 5 seconds total (50 * 100ms)
    
    for (int i = 0; i < max_retries; ++i) {
        try {
            transform = tf_buffer.lookupTransform(
                "map",           // target frame
                "base_link",     // source frame
                tf2::TimePointZero,  // use latest available transform
                std::chrono::milliseconds(100)  // timeout
            );
            transform_found = true;
            break;
        } catch (tf2::TransformException &ex) {
            if (i == max_retries - 1) {
                RCLCPP_ERROR(node->get_logger(), "Could not transform map to base_link: %s", ex.what());
                rclcpp::shutdown();
                return 1;
            }
            // Wait a bit before retrying
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    
    if (!transform_found) {
        RCLCPP_ERROR(node->get_logger(), "Failed to get transform after %d retries", max_retries);
        rclcpp::shutdown();
        return 1;
    }
    
    // Extract position
    double x = transform.transform.translation.x;
    double y = transform.transform.translation.y;
    
    // Extract orientation (quaternion)
    double qx = transform.transform.rotation.x;
    double qy = transform.transform.rotation.y;
    double qz = transform.transform.rotation.z;
    double qw = transform.transform.rotation.w;
    
    // Convert quaternion to yaw (z-axis rotation)
    double yaw = std::atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));
    
    RCLCPP_INFO(node->get_logger(), 
                "Position: x=%.3f, y=%.3f, yaw=%.3f",
                x, y, yaw);
    
    // Create JSON object with only x, y, and yaw
    nlohmann::json pose_json;
    pose_json["x"] = x;
    pose_json["y"] = y;
    pose_json["yaw"] = yaw;
    
    // Ensure target directory exists
    std::filesystem::path dir_path(target_dir);
    if (!std::filesystem::exists(dir_path)) {
        try {
            std::filesystem::create_directories(dir_path);
            RCLCPP_INFO(node->get_logger(), "Created directory: %s", target_dir.c_str());
        } catch (const std::filesystem::filesystem_error& e) {
            RCLCPP_ERROR(node->get_logger(), "Failed to create directory %s: %s", target_dir.c_str(), e.what());
            rclcpp::shutdown();
            return 1;
        }
    }
    
    // Write JSON to file
    std::filesystem::path file_path = dir_path / "init_pose.json";
    std::ofstream file(file_path);
    if (!file.is_open()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to open file for writing: %s", file_path.c_str());
        rclcpp::shutdown();
        return 1;
    }
    
    file << pose_json.dump(4);  // Pretty print with 4-space indent
    file.close();
    
    RCLCPP_INFO(node->get_logger(), "Position saved to: %s", file_path.c_str());
    
    rclcpp::shutdown();
    return 0;
}
