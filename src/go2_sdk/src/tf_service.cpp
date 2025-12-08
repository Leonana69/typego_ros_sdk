#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <cmath>
#include <arpa/inet.h>
#include <fcntl.h>

#include "typego_sdk/namespace_utils.hpp"

class TFServiceNode : public rclcpp::Node {
public:
    TFServiceNode() : Node("tf_service", typego_sdk::get_namespace_from_env()) {
        // Get namespace
        std::string ns = this->get_namespace();
        
        // Create TF broadcasters - they will publish to namespaced topics
        // (e.g., /robot2/tf and /robot2/tf_static)
        // IMPORTANT: In ROS 2 Humble, broadcasters won't use the node namespace.
        // You need to remap the topics when launching the node.
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        RCLCPP_INFO(this->get_logger(),
            "TF broadcasters initialized. Will publish to %s/tf and %s/tf_static",
            ns.c_str(), ns.c_str());
        
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 10);

        const char* go2_ip = std::getenv("ROBOT_IP");
        std::string go2_ip_ = go2_ip ? std::string(go2_ip) : "192.168.0.253";
        const uint16_t go2_livox_port = 8889;

        // UDP setup
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Non-blocking
        int flags = fcntl(socket_, F_GETFL, 0);
        fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

        // Bind socket
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(go2_livox_port);
        addr.sin_addr.s_addr = INADDR_ANY;
        bind(socket_, (sockaddr*)&addr, sizeof(addr));

        // Send init packet
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(go2_livox_port);
        inet_pton(AF_INET, go2_ip_.c_str(), &server_addr.sin_addr);
        uint8_t init_packet[1] = {0};
        sendto(socket_, init_packet, sizeof(init_packet), 0, 
               (sockaddr*)&server_addr, sizeof(server_addr));

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&TFServiceNode::poll_socket, this));

        init_static_tf();

        RCLCPP_INFO(this->get_logger(), 
                    "Go2 TF Service initialized in namespace: %s, publishing to %s/tf", 
                    ns.c_str(), ns.c_str());
    }

private:
    void init_static_tf() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = "base_link";
        t.child_frame_id = "imu_link";
        t.transform.translation.x = 0.0;
        t.transform.translation.y = 0.0;
        t.transform.translation.z = 0.0;
        t.transform.rotation.w = 1.0;

        static_tf_broadcaster_->sendTransform(t);
    }

    void poll_socket() {
        std::vector<uint8_t> buffer(2048);
        ssize_t rlen = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (rlen < static_cast<ssize_t>(13 * sizeof(float))) return;

        const float* data = reinterpret_cast<float*>(buffer.data());

        // === Publish TF ===
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = data[0];
        tf_msg.transform.translation.y = data[1];
        tf_msg.transform.translation.z = data[2];
        tf_msg.transform.rotation.w = data[3];
        tf_msg.transform.rotation.x = data[4];
        tf_msg.transform.rotation.y = data[5];
        tf_msg.transform.rotation.z = data[6];
        tf_broadcaster_->sendTransform(tf_msg);

        // === Publish IMU ===
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = tf_msg.header.stamp;
        imu_msg.header.frame_id = "imu_link";

        // Orientation (quaternion)
        imu_msg.orientation.w = data[3];
        imu_msg.orientation.x = data[4];
        imu_msg.orientation.y = data[5];
        imu_msg.orientation.z = data[6];

        // Angular velocity (gyro, rad/s)
        imu_msg.angular_velocity.x = data[10];
        imu_msg.angular_velocity.y = data[11];
        imu_msg.angular_velocity.z = data[12];

        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = data[7];
        imu_msg.linear_acceleration.y = data[8];
        imu_msg.linear_acceleration.z = data[9];

        // Covariances (tunable)
        imu_msg.orientation_covariance = {0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025};
        imu_msg.angular_velocity_covariance = {0.0001, 0, 0, 0, 0.0001, 0, 0, 0, 0.0001};
        imu_msg.linear_acceleration_covariance = {0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01};

        imu_pub_->publish(imu_msg);
    }

    int socket_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFServiceNode>());
    rclcpp::shutdown();
    return 0;
}