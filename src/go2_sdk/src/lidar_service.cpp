#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
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

#include "tf2_ros/static_transform_broadcaster.h"
#include "go2_sdk/namespace_utils.hpp"

class LidarServiceNode : public rclcpp::Node {
public:
    LidarServiceNode() : Node("lidar_service", go2_sdk::get_namespace_from_env()) {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox_points", 10);
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        init_lidar_link();

        const char* go2_ip = std::getenv("ROBOT_IP");
        std::string go2_ip_ = go2_ip ? std::string(go2_ip) : "192.168.0.253";
        const uint16_t go2_livox_port = 8888;

        // UDP setup with non-blocking socket and initial dummy packet
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Set socket to non-blocking
        int flags = fcntl(socket_, F_GETFL, 0);
        fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

        // Bind socket
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(go2_livox_port);
        addr.sin_addr.s_addr = INADDR_ANY;
        bind(socket_, (sockaddr*)&addr, sizeof(addr));

        // Send a dummy packet to notify server of this client's address
        sockaddr_in server_addr{};
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(go2_livox_port);
        inet_pton(AF_INET, go2_ip_.c_str(), &server_addr.sin_addr);

        uint8_t init_packet[1] = {0};
        sendto(socket_, init_packet, sizeof(init_packet), 0, 
            (sockaddr*)&server_addr, sizeof(server_addr));

        // Poll socket at ~1kHz
        recv_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1),
            std::bind(&LidarServiceNode::poll_socket, this));

        // Aggregate publish at 10Hz
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarServiceNode::publish_aggregated_cloud, this));

        // Prepare PointCloud2 fields once
        for (auto&& [name, offset] : {std::pair{"x",0}, {"y",4}, {"z",8}}) {
            sensor_msgs::msg::PointField f;
            f.name = name;
            f.offset = offset;
            f.datatype = sensor_msgs::msg::PointField::FLOAT32;
            f.count = 1;
            fields_.push_back(f);
        }
    }

    ~LidarServiceNode() {
        close(socket_);
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

    void poll_socket() {
        std::vector<uint8_t> buffer(2048);
        ssize_t rlen = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (rlen <= 2) return;

        constexpr float scale = 1.0f / 1000.0f;
        const int16_t* data = reinterpret_cast<const int16_t*>(buffer.data() + 2);
        size_t num_pts = (rlen - 2) / sizeof(int16_t);
        if (num_pts % 3 != 0) return;

        std::lock_guard<std::mutex> lock(buffer_mutex_);
        for (size_t i = 0; i < num_pts; i++)
            point_buffer_.emplace_back(data[i] * scale);
    }

    void publish_aggregated_cloud() {
        std::vector<float> local_buffer;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            if (point_buffer_.empty()) return;
            local_buffer.swap(point_buffer_);
        }

        size_t num_points = local_buffer.size() / 3;
        auto timestamp = this->now();

        
        cloud_msg_.header.stamp = timestamp;
        cloud_msg_.header.frame_id = "lidar_link";
        cloud_msg_.height = 1;
        cloud_msg_.width = num_points;
        cloud_msg_.is_dense = true;
        cloud_msg_.is_bigendian = false;
        cloud_msg_.point_step = 12;
        cloud_msg_.row_step = 12 * num_points;
        cloud_msg_.fields = fields_;

        cloud_msg_.data.resize(local_buffer.size() * sizeof(float));
        memcpy(cloud_msg_.data.data(), local_buffer.data(), cloud_msg_.data.size());
        publisher_->publish(cloud_msg_);

        // LaserScan projection
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = "lidar_link";
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0f;
        scan_msg.scan_time = 0.1f;
        scan_msg.range_min = 0.1f;
        scan_msg.range_max = 20.0f;

        int num_beams = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) /
                                         scan_msg.angle_increment);
        scan_msg.ranges.assign(num_beams, std::numeric_limits<float>::infinity());
        for (size_t i = 0; i < num_points; ++i) {
            float x = local_buffer[3 * i];
            float y = local_buffer[3 * i + 1];
            float z = local_buffer[3 * i + 2];
            if (std::abs(z) > 0.1f) continue;

            float angle = std::atan2(y, x);
            float range = std::hypot(x, y);
            int idx = static_cast<int>((angle - scan_msg.angle_min) / scan_msg.angle_increment);
            if (idx >= 0 && idx < num_beams && range < scan_msg.ranges[idx])
                scan_msg.ranges[idx] = range;
        }
        laserscan_publisher_->publish(scan_msg);
    }

    int socket_;
    rclcpp::TimerBase::SharedPtr recv_timer_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::mutex buffer_mutex_;
    std::vector<float> point_buffer_;
    sensor_msgs::msg::PointCloud2 cloud_msg_;

    std::vector<sensor_msgs::msg::PointField> fields_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarServiceNode>());
    rclcpp::shutdown();
    return 0;
}