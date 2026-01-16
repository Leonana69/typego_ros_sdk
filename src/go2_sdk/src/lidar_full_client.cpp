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
#include <map>
#include <limits>
#include <cstring>

#include "tf2_ros/static_transform_broadcaster.h"
#include "typego_sdk/namespace_utils.hpp"

// Optimized point structure (xyz only, int16 in millimeters)
#pragma pack(push, 1)
struct OptimizedPoint {
    int16_t x;  // mm
    int16_t y;  // mm
    int16_t z;  // mm
};
#pragma pack(pop)

// Packet header structure matching the server
#pragma pack(push, 1)
struct PointCloudPacketHeader {
    uint16_t sequence_id;
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
    uint32_t handle;
    uint8_t dev_type;
    uint8_t data_type;
    uint16_t point_count;
    // OptimizedPoint[] follows immediately after header
};
#pragma pack(pop)

constexpr size_t OPTIMIZED_POINT_SIZE = sizeof(OptimizedPoint);  // 6 bytes

class LidarFullClientNode : public rclcpp::Node {
public:
    LidarFullClientNode() : Node("lidar_full_client", typego_sdk::get_namespace_from_env()) {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox_points", 10);
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);

        init_lidar_link();

        const char* go2_ip = std::getenv("ROBOT_IP");
        std::string go2_ip_ = go2_ip ? std::string(go2_ip) : "192.168.0.243";
        RCLCPP_INFO(this->get_logger(), "Go2 IP: %s", go2_ip_.c_str());
        const uint16_t go2_livox_port = 8888;

        // UDP setup with non-blocking socket and initial dummy packet
        socket_ = socket(AF_INET, SOCK_DGRAM, 0);

        // Set socket to non-blocking
        int flags = fcntl(socket_, F_GETFL, 0);
        fcntl(socket_, F_SETFL, flags | O_NONBLOCK);

        // Bind socket to any available port (let OS choose)
        // Don't bind to server's port - we'll receive on whatever port OS assigns
        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = 0;  // Let OS choose an available port
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(socket_, (sockaddr*)&addr, sizeof(addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind UDP socket: %s", strerror(errno));
            return;
        }
        
        // Get the actual port we're bound to (for logging)
        socklen_t addr_len = sizeof(addr);
        getsockname(socket_, (sockaddr*)&addr, &addr_len);
        RCLCPP_INFO(this->get_logger(), "UDP client bound to port %d", ntohs(addr.sin_port));

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
            std::bind(&LidarFullClientNode::poll_socket, this));

        // Publish aggregated cloud at 10Hz (fallback if no scan boundary detected)
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LidarFullClientNode::publish_aggregated_cloud, this));
        
        // Scan boundary detection: if no packets arrive for 200ms, consider scan complete
        // This helps separate different lidar scans and prevent z-axis drift
        scan_timeout_ms_ = 200;

        // Prepare PointCloud2 fields
        sensor_msgs::msg::PointField f;
        f.name = "x";
        f.offset = 0;
        f.datatype = sensor_msgs::msg::PointField::FLOAT32;
        f.count = 1;
        fields_.push_back(f);
        f.name = "y";
        f.offset = 4;
        fields_.push_back(f);
        f.name = "z";
        f.offset = 8;
        fields_.push_back(f);
    }

    ~LidarFullClientNode() {
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
        constexpr size_t MAX_UDP_PAYLOAD = 1472;
        std::vector<uint8_t> buffer(MAX_UDP_PAYLOAD);
        
        ssize_t rlen = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (rlen < static_cast<ssize_t>(sizeof(PointCloudPacketHeader))) {
            return;
        }

        // Parse packet header
        PointCloudPacketHeader* header = reinterpret_cast<PointCloudPacketHeader*>(buffer.data());
        uint16_t seq_id = ntohs(header->sequence_id);
        uint16_t point_count = ntohs(header->point_count);
        uint32_t timestamp_sec = ntohl(header->timestamp_sec);
        uint32_t timestamp_nsec = ntohl(header->timestamp_nsec);

        size_t header_size = sizeof(PointCloudPacketHeader);
        size_t expected_data_size = point_count * OPTIMIZED_POINT_SIZE;
        size_t expected_packet_size = header_size + expected_data_size;
        
        // Verify packet integrity
        if (point_count == 0) {
            // Empty packet
            return;
        }
        
        if (rlen < static_cast<ssize_t>(expected_packet_size)) {
            packets_dropped_++;
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Incomplete packet: received %zd bytes, expected %zu (points: %u, dropped: %lu)", 
                                rlen, expected_packet_size, point_count, packets_dropped_);
            return;
        }
        
        total_packets_received_++;
        total_points_received_ += point_count;

        // Extract optimized points from raw buffer
        // Points are int16_t values in millimeters, need to convert to meters (floats)
        const OptimizedPoint* opt_points = reinterpret_cast<const OptimizedPoint*>(buffer.data() + header_size);
        
        bool scan_boundary_detected = false;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            
            // Detect scan boundaries:
            // 1. Sequence ID wrapped around (went backwards significantly)
            // 2. Large gap in sequence IDs (more than 1000, indicating new scan)
            // 3. Time gap since last packet (handled by timeout mechanism)
            uint16_t seq_diff = (seq_id > current_sequence_id_) ? 
                                (seq_id - current_sequence_id_) : 
                                (65535 - current_sequence_id_ + seq_id + 1);
            
            if (current_sequence_id_ != 0 && seq_diff > 1000) {
                // Large gap indicates new scan - publish current buffer
                scan_boundary_detected = true;
            }
            
            // Update sequence tracking
            current_sequence_id_ = seq_id;
            last_packet_time_ = this->now();
            
            // Add points to buffer (full point cloud - no filtering)
            // Convert from int16_t (mm) to float (meters) by dividing by 1000.0f
            constexpr float MM_TO_M = 1.0f / 1000.0f;
            for (uint16_t i = 0; i < point_count; ++i) {
                point_buffer_.push_back(opt_points[i].x * MM_TO_M);  // x in meters
                point_buffer_.push_back(opt_points[i].y * MM_TO_M);  // y in meters
                point_buffer_.push_back(opt_points[i].z * MM_TO_M);  // z in meters
            }
        }
        
        // If scan boundary detected, publish immediately
        if (scan_boundary_detected) {
            publish_aggregated_cloud();
        }
    }

    void publish_aggregated_cloud() {
        std::vector<float> local_buffer;
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            if (point_buffer_.empty()) return;
            
            // Check if enough time has passed since last packet (scan timeout)
            // This helps detect when a scan is complete and prevent mixing scans
            rclcpp::Time now = this->now();
            bool scan_complete = false;
            if (last_packet_time_.nanoseconds() > 0) {
                auto time_since_last = (now - last_packet_time_).nanoseconds() / 1000000;  // ms
                scan_complete = (time_since_last > scan_timeout_ms_);
            }
            
            // Always publish when timer fires (10Hz), but prefer publishing when scan is complete
            // This ensures we don't accumulate too many points while still allowing scan completion detection
            if (!scan_complete && point_buffer_.size() < 500) {
                // Very few points and scan still active, wait for more to avoid fragmenting scans
                return;
            }
            
            local_buffer.swap(point_buffer_);
        }

        size_t num_points = local_buffer.size() / 3;
        if (num_points == 0) return;

        // Log statistics periodically
        static uint64_t publish_count = 0;
        if (++publish_count % 100 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "Published cloud: %zu points (total received: %lu packets, %lu points, dropped: %lu)",
                       num_points, total_packets_received_, total_points_received_, packets_dropped_);
        }

        // Use ROS time for stable timestamps (more reliable than packet timestamps)
        rclcpp::Time timestamp = this->now();

        // Publish full point cloud
        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = timestamp;
        cloud_msg.header.frame_id = "lidar_link";
        cloud_msg.height = 1;
        cloud_msg.width = num_points;
        cloud_msg.is_dense = true;
        cloud_msg.is_bigendian = false;
        cloud_msg.point_step = 12;  // 3 floats * 4 bytes
        cloud_msg.row_step = 12 * num_points;
        cloud_msg.fields = fields_;
        cloud_msg.data.resize(local_buffer.size() * sizeof(float));
        memcpy(cloud_msg.data.data(), local_buffer.data(), cloud_msg.data.size());
        publisher_->publish(cloud_msg);

        // Publish filtered laser scan (2D projection)
        sensor_msgs::msg::LaserScan scan_msg;
        scan_msg.header.stamp = timestamp;
        scan_msg.header.frame_id = "lidar_link";
        scan_msg.angle_min = -M_PI;
        scan_msg.angle_max = M_PI;
        scan_msg.angle_increment = M_PI / 180.0f;  // 1 degree resolution
        scan_msg.scan_time = 0.1f;
        scan_msg.time_increment = scan_msg.scan_time / (360.0f);  // Approximate
        scan_msg.range_min = 0.1f;
        scan_msg.range_max = 20.0f;

        int num_beams = static_cast<int>((scan_msg.angle_max - scan_msg.angle_min) /
                                         scan_msg.angle_increment);
        scan_msg.ranges.assign(num_beams, std::numeric_limits<float>::infinity());

        // Filter points: only use points near z=0 (ground plane)
        constexpr float z_threshold = 0.1f;
        for (size_t i = 0; i < num_points; ++i) {
            float x = local_buffer[3 * i];
            float y = local_buffer[3 * i + 1];
            float z = local_buffer[3 * i + 2];
            
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

    int socket_;
    rclcpp::TimerBase::SharedPtr recv_timer_;
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::mutex buffer_mutex_;
    std::vector<float> point_buffer_;
    std::vector<sensor_msgs::msg::PointField> fields_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Packet sequence tracking
    uint16_t current_sequence_id_{0};
    rclcpp::Time last_packet_time_{0, 0};
    int32_t scan_timeout_ms_{50};  // Timeout in ms to detect scan completion
    
    // Statistics for debugging
    uint64_t total_packets_received_{0};
    uint64_t total_points_received_{0};
    uint64_t packets_dropped_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFullClientNode>());
    rclcpp::shutdown();
    return 0;
}
