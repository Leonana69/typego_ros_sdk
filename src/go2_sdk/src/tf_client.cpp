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
#include <chrono>

#include "typego_sdk/namespace_utils.hpp"
#include "typego_sdk/config_utils.hpp"

class TFClientNode : public rclcpp::Node {
public:
    TFClientNode() : Node("tf_client", typego_sdk::get_namespace_from_env()) {
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

        robot_ip_ = typego_sdk::resolve_robot_ip(this);
        robot_port_ = 8889;

        // Initialize connection
        connect_to_robot();

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&TFClientNode::poll_socket, this));

        init_static_tf();

        RCLCPP_INFO(this->get_logger(),
                    "Go2 TF Client initialized in namespace: %s, publishing to %s/tf", 
                    ns.c_str(), ns.c_str());
    }

    ~TFClientNode() {
        if (socket_ >= 0) {
            close(socket_);
        }
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

    void connect_to_robot() {
        // Close existing socket if any
        if (socket_ >= 0) {
            close(socket_);
        }

        // Create UDP socket
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
            return;
        }

        // Set non-blocking
        int flags = fcntl(socket_, F_GETFL, 0);
        fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

        // Bind socket
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(robot_port_);
        addr.sin_addr.s_addr = INADDR_ANY;
        
        if (bind(socket_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind socket");
            close(socket_);
            socket_ = -1;
            return;
        }

        // Send init packet
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(robot_port_);
        inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr);

        // connect() on a UDP socket pins the peer: the kernel drops datagrams
        // from any other source, preventing cross-talk when another robot on
        // the network is still streaming to this host.
        if (::connect(socket_, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to connect UDP socket to %s:%d: %s",
                         robot_ip_.c_str(), robot_port_, strerror(errno));
            close(socket_);
            socket_ = -1;
            return;
        }

        uint8_t init_packet[1] = {0};

        ssize_t sent = sendto(socket_, init_packet, sizeof(init_packet), 0,
                              (sockaddr*)&server_addr, sizeof(server_addr));
        
        if (sent < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send init packet to %s:%d", 
                         robot_ip_.c_str(), robot_port_);
        } else {
            RCLCPP_INFO(this->get_logger(), "Connected to robot at %s:%d", 
                        robot_ip_.c_str(), robot_port_);
            last_data_time_ = std::chrono::steady_clock::now();
            connection_active_ = true;
        }
    }

    void attempt_reconnect() {
        auto now = std::chrono::steady_clock::now();
        
        // Check if we should attempt reconnection (5 seconds since last attempt)
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_reconnect_attempt_).count() < 5) {
            return;
        }

        RCLCPP_WARN(this->get_logger(), "Attempting to reconnect to robot...");
        last_reconnect_attempt_ = now;
        
        connect_to_robot();
    }

    void poll_socket() {
        // Check for timeout if connection is active
        if (connection_active_) {
            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_data_time_).count();
            
            if (elapsed >= 5) {
                RCLCPP_WARN(this->get_logger(), "No data received for %ld seconds, connection lost", elapsed);
                connection_active_ = false;
                last_reconnect_attempt_ = std::chrono::steady_clock::now() - std::chrono::seconds(5); // Allow immediate reconnect
            }
        }

        // Try to reconnect if not connected
        if (!connection_active_) {
            attempt_reconnect();
            return;
        }

        // Poll for data
        std::vector<uint8_t> buffer(2048);
        ssize_t rlen = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (rlen < static_cast<ssize_t>(13 * sizeof(float))) return;

        // Data received successfully, update timestamp
        last_data_time_ = std::chrono::steady_clock::now();

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
    
    // Connection management
    std::string robot_ip_;
    uint16_t robot_port_;
    std::chrono::steady_clock::time_point last_data_time_;
    std::chrono::steady_clock::time_point last_reconnect_attempt_;
    bool connection_active_ = false;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFClientNode>());
    rclcpp::shutdown();
    return 0;
}
