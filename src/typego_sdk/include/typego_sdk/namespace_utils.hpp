#ifndef TYPEGO_SDK__NAMESPACE_UTILS_HPP_
#define TYPEGO_SDK__NAMESPACE_UTILS_HPP_

#include <string>
#include <cstring>

namespace typego_sdk {

/**
 * @brief Get ROS namespace from ROBOT_ID environment variable
 * @return Namespace string (e.g., "/robot1") or "/" if ROBOT_ID is not set
 */
inline std::string get_namespace_from_env() {
    const char* robot_id = std::getenv("ROBOT_ID");
    if (robot_id && std::strlen(robot_id) > 0) {
        return "/robot" + std::string(robot_id);
    }
    return "/";
}

/**
 * @brief Get frame prefix from namespace (e.g., "robot1/" from "/robot1")
 * @param ns Namespace string
 * @return Frame prefix with trailing slash, or empty string for root namespace
 */
inline std::string get_frame_prefix_from_namespace(const std::string& ns) {
    if (ns == "/") {
        return "";
    }
    // Remove leading "/" and add trailing "/"
    return ns.substr(1) + "/";
}

}  // namespace typego_sdk

#endif  // TYPEGO_SDK__NAMESPACE_UTILS_HPP_