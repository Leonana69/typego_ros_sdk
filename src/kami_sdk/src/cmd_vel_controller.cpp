#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nlohmann/json.hpp>
#include <thread>
#include <algorithm>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>

#include "typego_sdk/namespace_utils.hpp"

class CmdVelControllerNode : public rclcpp::Node {
public:
    CmdVelControllerNode() : Node("cmd_vel_controller", typego_sdk::get_namespace_from_env()) {
        // Get robot IP from environment variable or use default
        const char* robot_ip = std::getenv("ROBOT_IP");
        robot_ip_ = robot_ip ? std::string(robot_ip) : "192.168.168.168";
        ctrl_port_ = 8484;
        
        RCLCPP_INFO(this->get_logger(), "Kami IP: %s, Port: %d", robot_ip_.c_str(), ctrl_port_);

        // Create UDP socket
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
        } else {
            // Set socket timeout
            struct timeval tv;
            tv.tv_sec = 2;
            tv.tv_usec = 0;
            setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
        }

        // Accept cmd_vel parameter (default: true)
        this->declare_parameter("accept_cmd_vel", true);
        accept_cmd_vel_ = this->get_parameter("accept_cmd_vel").as_bool();

        // Subscribe to cmd_vel topic
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&CmdVelControllerNode::cmd_vel_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "CmdVel Controller node initialized");
    }

    ~CmdVelControllerNode() {
        if (sock_ >= 0) {
            close(sock_);
        }
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
        double vx = std::clamp(msg->linear.x, -max_speed_, max_speed_);
        double vy = std::clamp(msg->linear.y, -max_speed_, max_speed_);
        double vyaw = std::clamp(msg->angular.z, -max_angular_speed_, max_angular_speed_);

        if (vyaw > 0.0 && vyaw < min_angular_speed_) {
            vyaw = min_angular_speed_;
        } else if (vyaw < 0.0 && vyaw > -min_angular_speed_) {
            vyaw = -min_angular_speed_;
        }

        // Prepare JSON control command according to rotate.py format
        // joystick: [vy, -vx, vyaw, 0]
        nlohmann::json control = {
            {"type", "remote"},
            {"joystick", {vy, -vx, vyaw, 0}},
            {"button", {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}}
        };

        // Send request asynchronously to avoid blocking
        std::thread([this, control]() {
            send_control_request(control);
        }).detach();
    }

    void send_control_request(const nlohmann::json& control) {
        if (sock_ < 0) {
            return;
        }

        try {
            // Convert JSON to string and encode to bytes
            std::string json_str = control.dump();
            std::vector<uint8_t> data(json_str.begin(), json_str.end());

            // Setup destination address
            struct sockaddr_in server_addr;
            std::memset(&server_addr, 0, sizeof(server_addr));
            server_addr.sin_family = AF_INET;
            server_addr.sin_port = htons(ctrl_port_);
            if (inet_pton(AF_INET, robot_ip_.c_str(), &server_addr.sin_addr) <= 0) {
                RCLCPP_WARN(this->get_logger(), "Invalid address: %s", robot_ip_.c_str());
                return;
            }

            // Send UDP packet
            ssize_t sent = sendto(sock_, data.data(), data.size(), 0,
                                 (struct sockaddr*)&server_addr, sizeof(server_addr));
            if (sent < 0) {
                RCLCPP_WARN(this->get_logger(), "Failed to send control command: %s", strerror(errno));
            }
        } catch (const std::exception& e) {
            RCLCPP_WARN(this->get_logger(), "Exception in send_control_request: %s", e.what());
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    std::string robot_ip_;
    int ctrl_port_;
    int sock_;
    bool accept_cmd_vel_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelControllerNode>());
    rclcpp::shutdown();
    return 0;
}
