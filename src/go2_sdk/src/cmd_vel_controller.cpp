#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <thread>
#include <algorithm>

#include "typego_sdk/namespace_utils.hpp"
#include "typego_sdk/config_utils.hpp"

// Helper function for HTTP requests
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

class CmdVelControllerNode : public rclcpp::Node {
public:
    CmdVelControllerNode() : Node("cmd_vel_controller", typego_sdk::get_namespace_from_env()) {
        std::string robot_ip_ = typego_sdk::resolve_robot_ip(this);
        robot_url_ = "http://" + robot_ip_ + ":18080";
        RCLCPP_INFO(this->get_logger(), "Go2 IP: %s", robot_ip_.c_str());

        // Declare parameters
        this->declare_parameter("accept_cmd_vel", true);
        this->declare_parameter<std::string>("cmd_vel_type", "Twist");  // "Twist" or "TwistStamped"
        
        accept_cmd_vel_ = this->get_parameter("accept_cmd_vel").as_bool();
        std::string cmd_vel_type = this->get_parameter("cmd_vel_type").as_string();

        // Subscribe based on message type
        if (cmd_vel_type == "TwistStamped") {
            use_twist_stamped_ = true;
            cmd_vel_stamped_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
                "cmd_vel", 10, std::bind(&CmdVelControllerNode::cmd_vel_stamped_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "CmdVel Controller initialized with TwistStamped support");
        } else {
            use_twist_stamped_ = false;
            cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "cmd_vel", 10, std::bind(&CmdVelControllerNode::cmd_vel_callback, this, std::placeholders::_1));
            RCLCPP_INFO(this->get_logger(), "CmdVel Controller initialized with Twist support");
        }
    }

private:
    void cmd_vel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
        process_twist_command(msg->linear.x, msg->linear.y, msg->angular.z);
    }

    void cmd_vel_stamped_callback(geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        process_twist_command(msg->twist.linear.x, msg->twist.linear.y, msg->twist.angular.z);
    }

    void process_twist_command(double linear_x, double linear_y, double angular_z) {
        const double max_speed_ = 0.8;
        const double max_angular_speed_ = 1.0;
        const double min_angular_speed_ = 0.2;
        
        if (!accept_cmd_vel_) {
            return;
        }

        // Ignore zero velocity commands
        if (linear_x == 0.0 && linear_y == 0.0 && angular_z == 0.0) {
            return;
        }

        // Limit the velocity to the maximum speed
        linear_x  = std::clamp(linear_x, -max_speed_, max_speed_);
        linear_y  = std::clamp(linear_y, -max_speed_, max_speed_);
        angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);

        if (angular_z > 0.0 && angular_z < min_angular_speed_) {
            angular_z = min_angular_speed_;
        } else if (angular_z < 0.0 && angular_z > -min_angular_speed_) {
            angular_z = -min_angular_speed_;
        }

        // Prepare JSON control command
        nlohmann::json control = {
            {"command", "nav"},
            {"vx", linear_x},
            {"vy", linear_y},
            {"vyaw", angular_z}
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
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_vel_stamped_sub_;
    std::string robot_url_;
    bool accept_cmd_vel_;
    bool use_twist_stamped_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelControllerNode>());
    rclcpp::shutdown();
    return 0;
}
