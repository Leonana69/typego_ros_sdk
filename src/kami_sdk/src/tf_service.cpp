#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "typego_sdk/namespace_utils.hpp"

class TFServiceNode : public rclcpp::Node {
public:
    TFServiceNode() : Node("tf_service", typego_sdk::get_namespace_from_env()) {
        // Get namespace
        std::string ns = this->get_namespace();
        
        // Create TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // Subscribe to /odom/mc_odom topic
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/mc_odom", 10,
            std::bind(&TFServiceNode::odom_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(),
            "TF broadcaster initialized. Will publish to /tf");

        RCLCPP_INFO(this->get_logger(), 
                    "Kami TF Service initialized in namespace: %s, subscribing to /odom/mc_odom, publishing to /tf", 
                    ns.c_str());
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        // Create transform from odometry message
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = msg->header.stamp;
        tf_msg.header.frame_id = msg->header.frame_id;  // "odom"
        tf_msg.child_frame_id = msg->child_frame_id;    // "base_link"
        
        // Copy pose position
        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;
        
        // Copy pose orientation
        tf_msg.transform.rotation = msg->pose.pose.orientation;
        
        // Publish transform
        tf_broadcaster_->sendTransform(tf_msg);
    }
    
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFServiceNode>());
    rclcpp::shutdown();
    return 0;
}