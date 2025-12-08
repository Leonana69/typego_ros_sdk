#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <atomic>
#include <thread>
#include <cstring>

#include "typego_sdk/namespace_utils.hpp"

class GStreamerStream {
public:
    GStreamerStream(rclcpp::Node* node,
                    const std::string& topic_name,
                    const std::string& encoding,
                    const std::string& iface,
                    const std::string& host,
                    int port,
                    const std::string& frame_prefix,
                    bool is_depth = false)
        : node_(node), encoding_(encoding), is_depth_(is_depth), frame_prefix_(frame_prefix)
    {
        publisher_ = node_->create_publisher<sensor_msgs::msg::Image>(topic_name, 10);

        // For depth, we receive BGR (containing packed depth), for color we receive BGR
        std::string format = (encoding == "bgr8" || is_depth) ? "BGR" : "GRAY8";
        
        std::string pipeline_str =
            "udpsrc address=" + host + " port=" + std::to_string(port) +
            " multicast-iface=" + iface + " "
            "! application/x-rtp, media=video, encoding-name=H264 "
            "! rtph264depay "
            "! h264parse "
            "! avdec_h264 "
            "! videoconvert "
            "! video/x-raw,format=" + format + " "
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

        RCLCPP_INFO(node_->get_logger(), "Started GStreamer stream on host %s port %d with frame prefix: %s", 
                    host.c_str(), port, frame_prefix_.empty() ? "<none>" : frame_prefix_.c_str());
    }

    ~GStreamerStream() {
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

            sensor_msgs::msg::Image msg;
            msg.header.stamp = node_->get_clock()->now();
            msg.header.frame_id = is_depth_ ? (frame_prefix_ + "depth_frame") : (frame_prefix_ + "color_frame");
            msg.height = height;
            msg.width = width;

            if (is_depth_) {
                // 8-bit grayscale
                msg.encoding = "8UC1";
                msg.is_bigendian = false;
                msg.step = width;

                // Allocate space for 8-bit depth data
                msg.data.resize(width * height);
                uint8_t* depth_ptr = reinterpret_cast<uint8_t*>(msg.data.data());

                // Convert BGR to grayscale (take any channel since they're all the same)
                // Then scale back to millimeters: grayscale_value = depth_in_mm / 25
                for (int i = 0; i < width * height; ++i) {
                    uint8_t gray_value = map.data[i * 3];  // Take B channel (all BGR channels are same)
                    depth_ptr[i] = static_cast<uint8_t>(gray_value);  // Convert back to mm
                }
            } else {
                // Regular color image
                msg.encoding = encoding_;
                msg.is_bigendian = false;
                msg.step = width * 3;
                msg.data.assign(map.data, map.data + map.size);
            }

            gst_buffer_unmap(buffer, &map);
            gst_sample_unref(sample);

            publisher_->publish(msg);
        }
    }

    rclcpp::Node* node_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    GstElement* pipeline_{nullptr};
    GstElement* appsink_{nullptr};
    std::thread thread_;
    std::atomic<bool> running_{true};
    std::string encoding_;
    bool is_depth_;
    std::string frame_prefix_;
};

class VideoServiceNode : public rclcpp::Node {
public:
    VideoServiceNode() : Node("video_service", typego_sdk::get_namespace_from_env()) {
        gst_init(nullptr, nullptr);
        
        // Get frame prefix from namespace
        std::string ns = this->get_namespace();
        frame_prefix_ = typego_sdk::get_frame_prefix_from_namespace(ns);

        RCLCPP_INFO(this->get_logger(), "Dual GStreamer node initializing in namespace: %s with frame prefix: %s", 
                    ns.c_str(), frame_prefix_.empty() ? "<none>" : frame_prefix_.c_str());

        const char* iface = std::getenv("GSTREAMER_INTERFACE");
        std::string interface_str = iface ? iface : "wlp38s0";

        const char* rgb_port_env = std::getenv("GSTREAMER_RGB_PORT");
        int rgb_port = rgb_port_env ? std::atoi(rgb_port_env) : 1722;
        const char* depth_port_env = std::getenv("GSTREAMER_DEPTH_PORT");
        int depth_port = depth_port_env ? std::atoi(depth_port_env) : 1723;

        const char* robot_id = std::getenv("ROBOT_ID");
        std::string host_str = robot_id ? "230.1.1." + std::string(robot_id) : "230.1.1.1";

        color_stream_ = std::make_unique<GStreamerStream>(this,
                        "camera/color/image_raw", "bgr8", interface_str, host_str, rgb_port, frame_prefix_, false);
        depth_stream_ = std::make_unique<GStreamerStream>(this,
                        "camera/depth/image_raw", "16UC1", interface_str, host_str, depth_port, frame_prefix_, true);

        RCLCPP_INFO(this->get_logger(), "Dual GStreamer node initialized.");
    }

private:
    std::unique_ptr<GStreamerStream> color_stream_;
    std::unique_ptr<GStreamerStream> depth_stream_;
    std::string frame_prefix_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoServiceNode>());
    rclcpp::shutdown();
    return 0;
}