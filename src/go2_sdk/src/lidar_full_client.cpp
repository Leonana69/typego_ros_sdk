#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include <arpa/inet.h>
#include <fcntl.h>
#include <map>
#include <limits>
#include <cstring>

#include "tf2_ros/static_transform_broadcaster.h"
#include "typego_sdk/namespace_utils.hpp"
#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

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

// IMU data structure (packed for network transmission)
#pragma pack(push, 1)
struct ImuData {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
};
#pragma pack(pop)

// Packet header structure for IMU data transmission
#pragma pack(push, 1)
struct ImuPacketHeader {
    uint16_t sequence_id;
    uint32_t timestamp_sec;
    uint32_t timestamp_nsec;
    uint32_t handle;
    uint8_t dev_type;
    uint8_t data_type;  // Will contain IMU data type from Livox SDK
    uint16_t imu_count;  // Number of IMU samples in this packet
    // ImuData[] follows immediately after header
};
#pragma pack(pop)

constexpr size_t OPTIMIZED_POINT_SIZE = sizeof(OptimizedPoint);  // 6 bytes
constexpr size_t IMU_DATA_SIZE = sizeof(ImuData);  // 24 bytes
constexpr size_t IMU_PACKET_HEADER_SIZE = sizeof(ImuPacketHeader);  // 18 bytes
constexpr size_t IMU_PACKET_SIZE_SINGLE = IMU_PACKET_HEADER_SIZE + IMU_DATA_SIZE;  // 42 bytes for 1 IMU sample

class LidarFullClientNode : public rclcpp::Node {
public:
    LidarFullClientNode() : Node("lidar_full_client", typego_sdk::get_namespace_from_env()) {
        // Declare parameters
        this->declare_parameter<double>("publish_rate", 10.0);  // Hz
        this->declare_parameter<int>("scan_timeout_ms", 200);   // milliseconds
        this->declare_parameter<std::string>("xfer_format", "PointCloud2");  // "PointCloud2" or "CustomMsg"
        
        // Get parameters
        double publish_rate = this->get_parameter("publish_rate").as_double();
        scan_timeout_ms_ = this->get_parameter("scan_timeout_ms").as_int();
        std::string xfer_format = this->get_parameter("xfer_format").as_string();
        
        // Validate parameters
        if (publish_rate <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Invalid publish_rate %f, using default 10.0 Hz", publish_rate);
            publish_rate = 10.0;
        }
        if (scan_timeout_ms_ <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid scan_timeout_ms %d, using default 200 ms", scan_timeout_ms_);
            scan_timeout_ms_ = 200;
        }
        
        // Set transfer format
        if (xfer_format == "CustomMsg") {
            use_custom_msg_ = true;
            RCLCPP_INFO(this->get_logger(), "Using Livox CustomMsg format");
        } else {
            use_custom_msg_ = false;
            RCLCPP_INFO(this->get_logger(), "Using PointCloud2 format");
        }
        
        int publish_period_ms = static_cast<int>(1000.0 / publish_rate);
        RCLCPP_INFO(this->get_logger(), "Publish rate: %.1f Hz (period: %d ms), Scan timeout: %d ms", 
                   publish_rate, publish_period_ms, scan_timeout_ms_);
        
        // Create publishers based on format
        if (use_custom_msg_) {
            custom_publisher_ = this->create_publisher<livox_ros_driver2::msg::CustomMsg>("livox/lidar", 10);
        } else {
            publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("livox/lidar", 10);
        }
        laserscan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("livox/imu", 10);

        init_lidar_link_tf();

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

        // Start dedicated receive thread for high-frequency UDP reception (2000+ Hz)
        // This is more efficient than timer-based polling and won't miss packets
        recv_thread_running_ = true;
        recv_thread_ = std::thread(&LidarFullClientNode::receive_thread_func, this);

        // Publish aggregated cloud at configured rate (fallback if no scan boundary detected)
        pub_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_period_ms),
            std::bind(&LidarFullClientNode::publish_aggregated_cloud, this));

        // Prepare PointCloud2 fields (x, y, z, intensity)
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
        f.name = "intensity";
        f.offset = 12;
        fields_.push_back(f);
    }

    ~LidarFullClientNode() {
        // Stop receive thread
        recv_thread_running_ = false;
        if (recv_thread_.joinable()) {
            recv_thread_.join();
        }
        close(socket_);
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

        static_tf_broadcaster_->sendTransform(t);
    }

    void receive_thread_func() {
        // Dedicated thread for high-frequency UDP packet reception
        // Runs in a tight loop to handle 2000+ Hz packet rates without missing packets
        constexpr size_t MAX_UDP_PAYLOAD = 1472;
        std::vector<uint8_t> buffer(MAX_UDP_PAYLOAD);
        
        while (recv_thread_running_) {
            ssize_t rlen = recvfrom(socket_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
            if (rlen < static_cast<ssize_t>(IMU_PACKET_HEADER_SIZE)) {
                // No data available (non-blocking socket) or error
                // Small sleep to prevent CPU spinning when no packets arrive
                std::this_thread::sleep_for(std::chrono::microseconds(10));
                continue;
            }
            
            // Process the received packet (check if it's IMU or point cloud)
            if (rlen == static_cast<ssize_t>(IMU_PACKET_SIZE_SINGLE)) {
                // Likely an IMU packet (single IMU sample)
                process_imu_packet(buffer.data(), rlen);
            } else {
                // Likely a point cloud packet
                process_packet(buffer.data(), rlen);
            }
        }
    }
    
    void process_imu_packet(const uint8_t* buffer_data, ssize_t rlen) {
        if (rlen < static_cast<ssize_t>(IMU_PACKET_HEADER_SIZE)) {
            return;
        }

        // Parse IMU packet header
        const ImuPacketHeader* header = reinterpret_cast<const ImuPacketHeader*>(buffer_data);
        uint16_t imu_count = ntohs(header->imu_count);
        
        // Verify packet integrity
        if (imu_count == 0) {
            return;
        }
        
        size_t expected_packet_size = IMU_PACKET_HEADER_SIZE + (imu_count * IMU_DATA_SIZE);
        if (rlen < static_cast<ssize_t>(expected_packet_size)) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                "Incomplete IMU packet: received %zd bytes, expected %zu (imu_count: %u)", 
                                rlen, expected_packet_size, imu_count);
            return;
        }
        
        // Extract IMU data (only use first sample as user specified)
        const ImuData* imu_data = reinterpret_cast<const ImuData*>(buffer_data + IMU_PACKET_HEADER_SIZE);
        
        // Create and publish IMU message
        sensor_msgs::msg::Imu imu_msg;
        imu_msg.header.stamp = this->now();
        imu_msg.header.frame_id = "lidar_link";
        
        // Linear acceleration (m/s²)
        imu_msg.linear_acceleration.x = imu_data[0].acc_x;
        imu_msg.linear_acceleration.y = imu_data[0].acc_y;
        imu_msg.linear_acceleration.z = imu_data[0].acc_z;
        
        // Angular velocity (rad/s)
        imu_msg.angular_velocity.x = imu_data[0].gyro_x;
        imu_msg.angular_velocity.y = imu_data[0].gyro_y;
        imu_msg.angular_velocity.z = imu_data[0].gyro_z;
        
        // Orientation is not provided, so we'll leave it uninitialized (ROS will treat as unknown)
        // Set covariance matrices to indicate unknown orientation
        imu_msg.orientation_covariance[0] = -1.0;  // -1 indicates unknown
        
        // Set reasonable covariance values for acceleration and angular velocity
        imu_msg.linear_acceleration_covariance[0] = 0.01;
        imu_msg.linear_acceleration_covariance[4] = 0.01;
        imu_msg.linear_acceleration_covariance[8] = 0.01;
        
        imu_msg.angular_velocity_covariance[0] = 0.0001;
        imu_msg.angular_velocity_covariance[4] = 0.0001;
        imu_msg.angular_velocity_covariance[8] = 0.0001;
        
        imu_publisher_->publish(imu_msg);
    }
    
    void process_packet(const uint8_t* buffer_data, ssize_t rlen) {
        if (rlen < static_cast<ssize_t>(sizeof(PointCloudPacketHeader))) {
            return;
        }

        // Parse packet header
        const PointCloudPacketHeader* header = reinterpret_cast<const PointCloudPacketHeader*>(buffer_data);
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
        const OptimizedPoint* opt_points = reinterpret_cast<const OptimizedPoint*>(buffer_data + header_size);
        
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
            
            // Store packet timestamp for CustomMsg
            if (custom_points_.empty()) {
                // First packet in this scan, store base timestamp
                packet_base_time_ns_ = static_cast<uint64_t>(timestamp_sec) * 1000000000ULL + 
                                       static_cast<uint64_t>(timestamp_nsec);
            }
            
            // Add points to buffer (full point cloud - no filtering)
            // Convert from int16_t (mm) to float (meters) by dividing by 1000.0f
            constexpr float MM_TO_M = 1.0f / 1000.0f;
            
            if (use_custom_msg_) {
                // Store points with additional info for CustomMsg format
                for (uint16_t i = 0; i < point_count; ++i) {
                    livox_ros_driver2::msg::CustomPoint pt;
                    pt.x = opt_points[i].x * MM_TO_M;
                    pt.y = opt_points[i].y * MM_TO_M;
                    pt.z = opt_points[i].z * MM_TO_M;
                    // Calculate distance for reflectivity approximation
                    float dist = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
                    pt.reflectivity = static_cast<uint8_t>(std::min(dist * 12.75f, 255.0f));  // Approx
                    pt.tag = 0;  // Not available in our optimized format
                    pt.line = 0;  // Not available in our optimized format
                    // Offset time in nanoseconds from base time
                    uint64_t current_time_ns = static_cast<uint64_t>(timestamp_sec) * 1000000000ULL + 
                                               static_cast<uint64_t>(timestamp_nsec);
                    pt.offset_time = static_cast<uint32_t>(current_time_ns - packet_base_time_ns_);
                    custom_points_.push_back(pt);
                }
            } else {
                // Store points for PointCloud2 format
                for (uint16_t i = 0; i < point_count; ++i) {
                    point_buffer_.push_back(opt_points[i].x * MM_TO_M);  // x in meters
                    point_buffer_.push_back(opt_points[i].y * MM_TO_M);  // y in meters
                    point_buffer_.push_back(opt_points[i].z * MM_TO_M);  // z in meters
                }
            }
        }
        
        // If scan boundary detected, publish immediately
        if (scan_boundary_detected) {
            publish_aggregated_cloud();
        }
    }

    void publish_aggregated_cloud() {
        std::vector<float> local_buffer;
        std::vector<livox_ros_driver2::msg::CustomPoint> local_custom_points;
        uint64_t local_base_time_ns = 0;
        
        {
            std::lock_guard<std::mutex> lock(buffer_mutex_);
            
            // Check which buffer to use
            size_t buffer_size = use_custom_msg_ ? custom_points_.size() : point_buffer_.size();
            if (buffer_size == 0) return;
            
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
            size_t min_points = use_custom_msg_ ? 500 : 1500;  // 500 points for custom, 1500 floats (500 points) for PC2
            if (!scan_complete && buffer_size < min_points) {
                // Very few points and scan still active, wait for more to avoid fragmenting scans
                return;
            }
            
            if (use_custom_msg_) {
                local_custom_points.swap(custom_points_);
                local_base_time_ns = packet_base_time_ns_;
                packet_base_time_ns_ = 0;  // Reset for next scan
            } else {
                local_buffer.swap(point_buffer_);
            }
        }

        // Use ROS time for stable timestamps (more reliable than packet timestamps)
        rclcpp::Time timestamp = this->now();

        size_t num_points = 0;
        
        if (use_custom_msg_) {
            // Publish CustomMsg format
            num_points = local_custom_points.size();
            if (num_points == 0) return;
            
            livox_ros_driver2::msg::CustomMsg custom_msg;
            custom_msg.header.stamp = timestamp;
            custom_msg.header.frame_id = "lidar_link";
            custom_msg.timebase = local_base_time_ns;
            custom_msg.point_num = static_cast<uint32_t>(num_points);
            custom_msg.lidar_id = 0;  // Not available in our format
            custom_msg.points = std::move(local_custom_points);
            
            custom_publisher_->publish(custom_msg);
            
            // Log statistics periodically
            static uint64_t publish_count = 0;
            if (++publish_count % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Published CustomMsg: %zu points (total received: %lu packets, %lu points, dropped: %lu)",
                           num_points, total_packets_received_, total_points_received_, packets_dropped_);
            }
            
            // Publish laser scan from custom points
            publish_laser_scan_from_custom(custom_msg.points, timestamp);
            
        } else {
            // Publish PointCloud2 format
            num_points = local_buffer.size() / 3;
            if (num_points == 0) return;

            // Log statistics periodically
            static uint64_t publish_count = 0;
            if (++publish_count % 100 == 0) {
                RCLCPP_INFO(this->get_logger(), 
                           "Published PointCloud2: %zu points (total received: %lu packets, %lu points, dropped: %lu)",
                           num_points, total_packets_received_, total_points_received_, packets_dropped_);
            }

            // Publish full point cloud with intensity (for color visualization in RViz2)
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_msg.header.stamp = timestamp;
            cloud_msg.header.frame_id = "lidar_link";
            cloud_msg.height = 1;
            cloud_msg.width = num_points;
            cloud_msg.is_dense = true;
            cloud_msg.is_bigendian = false;
            cloud_msg.point_step = 16;  // 4 floats * 4 bytes (x, y, z, intensity)
            cloud_msg.row_step = 16 * num_points;
            cloud_msg.fields = fields_;
            
            // Create point cloud data with intensity computed from distance
            cloud_msg.data.resize(num_points * cloud_msg.point_step);
            for (size_t i = 0; i < num_points; ++i) {
                float x = local_buffer[3 * i];
                float y = local_buffer[3 * i + 1];
                float z = local_buffer[3 * i + 2];
                
                // Compute intensity from distance (normalized to 0-1 range, max at 20m)
                float distance = std::sqrt(x * x + y * y + z * z);
                float intensity = std::min(distance / 20.0f, 1.0f);  // Normalize to 0-1, max at 20m
                
                // Write point data (x, y, z, intensity)
                float* point_data = reinterpret_cast<float*>(cloud_msg.data.data() + i * cloud_msg.point_step);
                point_data[0] = x;
                point_data[1] = y;
                point_data[2] = z;
                point_data[3] = intensity;
            }
            
            publisher_->publish(cloud_msg);

            // Publish laser scan from buffer
            publish_laser_scan_from_buffer(local_buffer, timestamp);
        }
    }
    
    void publish_laser_scan_from_buffer(const std::vector<float>& buffer, const rclcpp::Time& timestamp) {
        size_t num_points = buffer.size() / 3;
        
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
            float x = buffer[3 * i];
            float y = buffer[3 * i + 1];
            float z = buffer[3 * i + 2];
            
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
    
    void publish_laser_scan_from_custom(const std::vector<livox_ros_driver2::msg::CustomPoint>& points, 
                                        const rclcpp::Time& timestamp) {
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
        for (const auto& point : points) {
            // Filter out points far from ground plane
            if (std::abs(point.z) > z_threshold) continue;

            float angle = std::atan2(point.y, point.x);
            float range = std::hypot(point.x, point.y);
            
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
    std::thread recv_thread_;
    std::atomic<bool> recv_thread_running_{false};
    rclcpp::TimerBase::SharedPtr pub_timer_;
    std::mutex buffer_mutex_;
    std::vector<float> point_buffer_;
    std::vector<livox_ros_driver2::msg::CustomPoint> custom_points_;
    std::vector<sensor_msgs::msg::PointField> fields_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr custom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laserscan_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    
    // Transfer format flag
    bool use_custom_msg_{false};
    
    // Packet sequence tracking
    uint16_t current_sequence_id_{0};
    rclcpp::Time last_packet_time_{0, 0};
    int32_t scan_timeout_ms_{50};  // Timeout in ms to detect scan completion
    uint64_t packet_base_time_ns_{0};  // Base timestamp for CustomMsg
    
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
