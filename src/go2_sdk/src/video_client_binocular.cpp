#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <atomic>
#include <thread>
#include <cstring>

#include "typego_sdk/namespace_utils.hpp"

class BinocularGStreamerStream {
public:
    BinocularGStreamerStream(rclcpp::Node* node,
                             const std::string& iface,
                             const std::string& host,
                             int port,
                             const std::string& frame_prefix)
        : node_(node), frame_prefix_(frame_prefix)
    {
        // Create publishers for single camera and binocular modes
        single_camera_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("camera/color/image_raw", 10);
        left_camera_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("camera/color/image_left_raw", 10);
        right_camera_publisher_ = node_->create_publisher<sensor_msgs::msg::Image>("camera/color/image_right_raw", 10);
        
        // Build GStreamer pipeline matching the provided command
        std::string pipeline_str =
            "udpsrc address=" + host + " port=" + std::to_string(port) +
            " multicast-iface=" + iface + " buffer-size=5000000 "
            "! application/x-rtp,encoding-name=H264,payload=96 "
            "! rtph264depay "
            "! h264parse "
            "! queue max-size-buffers=2 leaky=downstream "
            "! avdec_h264 "
            "! queue max-size-buffers=2 leaky=downstream "
            "! videoconvert "
            "! video/x-raw,format=BGR "
            "! appsink name=appsink sync=false max-buffers=1 drop=true";

        GError* error = nullptr;
        pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);
        if (!pipeline_ || error) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create GStreamer pipeline on port %d: %s",
                         port, error ? error->message : "unknown error");
            if (error) g_clear_error(&error);
            return;
        }

        appsink_ = gst_bin_get_by_name(GST_BIN(pipeline_), "appsink");
        gst_element_set_state(pipeline_, GST_STATE_PLAYING);

        thread_ = std::thread([this]() { run(); });

        RCLCPP_INFO(node_->get_logger(), "Started Binocular GStreamer stream on host %s port %d with frame prefix: %s", 
                    host.c_str(), port, frame_prefix_.empty() ? "<none>" : frame_prefix_.c_str());
    }

    ~BinocularGStreamerStream() {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (appsink_) gst_object_unref(appsink_);
        if (pipeline_) gst_object_unref(pipeline_);
    }

private:
    void run() {
        while (rclcpp::ok() && running_) {
            GstSample* sample = gst_app_sink_try_pull_sample(GST_APP_SINK(appsink_), 10 * GST_MSECOND);
            if (!sample) continue;

            GstBuffer* buffer = gst_sample_get_buffer(sample);
            GstCaps* caps = gst_sample_get_caps(sample);
            GstStructure* s = gst_caps_get_structure(caps, 0);

            int width = 0, height = 0;
            gst_structure_get_int(s, "width", &width);
            gst_structure_get_int(s, "height", &height);

            GstMapInfo map;
            if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
                gst_sample_unref(sample);
                continue;
            }

            auto stamp = node_->get_clock()->now();
            bool is_binocular = (width > 2 * height);

            if (is_binocular) {
                // Split the image into left and right halves
                int half_width = width / 2;
                
                // Create left image message
                sensor_msgs::msg::Image left_msg;
                left_msg.header.stamp = stamp;
                left_msg.header.frame_id = frame_prefix_ + "color_left_frame";
                left_msg.height = height;
                left_msg.width = half_width;
                left_msg.encoding = "bgr8";
                left_msg.is_bigendian = false;
                left_msg.step = half_width * 3;
                left_msg.data.resize(half_width * height * 3);

                // Create right image message
                sensor_msgs::msg::Image right_msg;
                right_msg.header.stamp = stamp;
                right_msg.header.frame_id = frame_prefix_ + "color_right_frame";
                right_msg.height = height;
                right_msg.width = half_width;
                right_msg.encoding = "bgr8";
                right_msg.is_bigendian = false;
                right_msg.step = half_width * 3;
                right_msg.data.resize(half_width * height * 3);

                // Copy left half (first half_width columns)
                for (int y = 0; y < height; ++y) {
                    const uint8_t* src_row = map.data + y * width * 3;
                    uint8_t* dst_left_row = left_msg.data.data() + y * half_width * 3;
                    uint8_t* dst_right_row = right_msg.data.data() + y * half_width * 3;
                    
                    // Copy left half
                    std::memcpy(dst_left_row, src_row, half_width * 3);
                    // Copy right half
                    std::memcpy(dst_right_row, src_row + half_width * 3, half_width * 3);
                }

                left_camera_publisher_->publish(left_msg);
                right_camera_publisher_->publish(right_msg);
            } else {
                // Single camera image - publish as is
                sensor_msgs::msg::Image msg;
                msg.header.stamp = stamp;
                msg.header.frame_id = frame_prefix_ + "color_frame";
                msg.height = height;
                msg.width = width;
                msg.encoding = "bgr8";
                msg.is_bigendian = false;
                msg.step = width * 3;
                msg.data.assign(map.data, map.data + map.size);

                single_camera_publisher_->publish(msg);
            }

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);
        }
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr single_camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_camera_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_camera_publisher_;
    GstElement* pipeline_{nullptr};
    GstElement* appsink_{nullptr};
    std::thread thread_;
    std::atomic<bool> running_{true};
    std::string frame_prefix_;
};

class VideoClientNode : public rclcpp::Node {
public:
    VideoClientNode() : Node("video_client", typego_sdk::get_namespace_from_env()) {
        gst_init(nullptr, nullptr);
        
        // Get frame prefix from namespace
        std::string ns = this->get_namespace();
        frame_prefix_ = typego_sdk::get_frame_prefix_from_namespace(ns);

        RCLCPP_INFO(this->get_logger(), "Binocular GStreamer node initializing in namespace: %s with frame prefix: %s", 
                    ns.c_str(), frame_prefix_.empty() ? "<none>" : frame_prefix_.c_str());

        const char* iface = std::getenv("GSTREAMER_INTERFACE");
        std::string interface_str = iface ? iface : "wlan0";

        const char* rgb_port_env = std::getenv("GSTREAMER_RGB_PORT");
        int rgb_port = rgb_port_env ? std::atoi(rgb_port_env) : 1722;

        const char* ros_domain_id = std::getenv("ROS_DOMAIN_ID");
        std::string host_str = ros_domain_id ? "230.1.1." + std::string(ros_domain_id) : "230.1.1.1";

        color_stream_ = std::make_unique<BinocularGStreamerStream>(this,
                        interface_str, host_str, rgb_port, frame_prefix_);

        RCLCPP_INFO(this->get_logger(), "Binocular GStreamer node initialized.");
    }

private:
    std::unique_ptr<BinocularGStreamerStream> color_stream_;
    std::string frame_prefix_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoClientNode>());
    rclcpp::shutdown();
    return 0;
}
