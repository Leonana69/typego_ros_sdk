// Shared helpers for reading identity/network config from the ROS
// parameter API with a limited env-var fallback for unported callers.
//
// Precedence for every field: declared ROS parameter > environment
// variable > fatal error. The hardcoded IP fallbacks that used to live
// in tf_client/lidar_client/cmd_vel_controller are intentionally gone —
// the single source of truth is robot.yaml, rendered to env vars by
// ``typego-config env`` and/or passed as parameters by the launch files.
#ifndef TYPEGO_SDK__CONFIG_UTILS_HPP_
#define TYPEGO_SDK__CONFIG_UTILS_HPP_

#include <cstdlib>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace typego_sdk {

inline std::string resolve_string(
    rclcpp::Node * node,
    const std::string & param_name,
    const char * env_name,
    const std::string & hint)
{
    if (!node->has_parameter(param_name)) {
        node->declare_parameter<std::string>(param_name, "");
    }
    auto value = node->get_parameter(param_name).as_string();
    if (!value.empty()) {
        return value;
    }
    const char * env_val = std::getenv(env_name);
    if (env_val && env_val[0] != '\0') {
        return std::string(env_val);
    }
    throw std::runtime_error(
        std::string("typego_sdk: ") + param_name +
        " not set (nor $" + env_name + "). " + hint);
}

inline int resolve_int(
    rclcpp::Node * node,
    const std::string & param_name,
    const char * env_name,
    int sentinel,
    const std::string & hint)
{
    if (!node->has_parameter(param_name)) {
        node->declare_parameter<int>(param_name, sentinel);
    }
    int value = node->get_parameter(param_name).as_int();
    if (value != sentinel) {
        return value;
    }
    const char * env_val = std::getenv(env_name);
    if (env_val && env_val[0] != '\0') {
        try {
            return std::stoi(env_val);
        } catch (const std::exception &) {
            // fall through to throw below
        }
    }
    throw std::runtime_error(
        std::string("typego_sdk: ") + param_name +
        " not set (nor $" + env_name + "). " + hint);
}

inline std::string resolve_robot_ip(rclcpp::Node * node)
{
    return resolve_string(
        node, "robot_ip", "ROBOT_IP",
        "Set network.robot_ip in robot.yaml or pass -p robot_ip:=<addr>.");
}

inline std::string resolve_edge_service_ip(rclcpp::Node * node)
{
    return resolve_string(
        node, "edge_service_ip", "EDGE_SERVICE_IP",
        "Set network.edge_service.ip in robot.yaml.");
}

inline int resolve_edge_service_port(rclcpp::Node * node)
{
    return resolve_int(
        node, "edge_service_port", "EDGE_SERVICE_PORT", 0,
        "Set network.edge_service.port in robot.yaml.");
}

}  // namespace typego_sdk

#endif  // TYPEGO_SDK__CONFIG_UTILS_HPP_
