#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <algorithm>

#include "typego_sdk/namespace_utils.hpp"

// Helper function for HTTP requests
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

class CmdVelControllerNode : public rclcpp::Node {
public:
    CmdVelControllerNode() : Node("cmd_vel_controller", typego_sdk::get_namespace_from_env()) {
        // Get robot URL from environment variable or use default
        const char* robot_ip = std::getenv("ROBOT_IP");
        std::string robot_ip_ = robot_ip ? std::string(robot_ip) : "192.168.0.243";
        robot_url_ = "http://" + robot_ip_ + ":18080";
        RCLCPP_INFO(this->get_logger(), "Go2 IP: %s", robot_ip_.c_str());

        // Accept cmd_vel parameter (default: true)
        this->declare_parameter("accept_cmd_vel", true);
        accept_cmd_vel_ = this->get_parameter("accept_cmd_vel").as_bool();

        // Subscribe to cmd_vel topic
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelControllerNode::cmd_vel_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CmdVel Controller node initialized");
    }

private:
    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
        const double max_speed_ = 0.8;
        const double max_angular_speed_ = 1.0;
        const double min_angular_speed_ = 0.2;
        if (!accept_cmd_vel_) {
            return;
        }

        // Ignore zero velocity commands
        if (msg->linear.x == 0.0 && msg->linear.y == 0.0 && msg->angular.z == 0.0) {
            return;
        }

        // Limit the velocity to the maximum speed
        msg->linear.x  = std::clamp(msg->linear.x, -max_speed_, max_speed_);
        msg->linear.y  = std::clamp(msg->linear.y, -max_speed_, max_speed_);
        msg->angular.z = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        if (msg->angular.z > 0.0 && msg->angular.z < min_angular_speed_) {
            msg->angular.z = min_angular_speed_;
        } else if (msg->angular.z < 0.0 && msg->angular.z > -min_angular_speed_) {
            msg->angular.z = -min_angular_speed_;
        }

        // Prepare JSON control command
        nlohmann::json control = {
            {"command", "nav"},
            {"vx", msg->linear.x},
            {"vy", msg->linear.y},
            {"vyaw", msg->angular.z}
        };

        // Send request asynchronously to avoid blocking
        std::thread([this, control]() {
            send_control_request(control);
        }).detach();
    }

    void send_control_request(const nlohmann::json& control) {
        CURL *curl = curl_easy_init();
        if (curl) {
            std::string url = robot_url_ + "/control";
            std::string json_str = control.dump();

            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_str.c_str());

            // Set headers
            struct curl_slist *headers = nullptr;
            headers = curl_slist_append(headers, "Content-Type: application/json");
            curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

            // Set timeout
            curl_easy_setopt(curl, CURLOPT_TIMEOUT_MS, 500L);

            // Response handling (optional, can be discarded)
            std::string response_string;
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

            // Perform the request
            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                RCLCPP_WARN(this->get_logger(), "Failed to send control command: %s", curl_easy_strerror(res));
            }

            // Cleanup
            curl_slist_free_all(headers);
            curl_easy_cleanup(curl);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::string robot_url_;
    bool accept_cmd_vel_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelControllerNode>());
    rclcpp::shutdown();
    return 0;
}
