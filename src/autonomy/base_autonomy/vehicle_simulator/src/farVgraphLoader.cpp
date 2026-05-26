#include <chrono>
#include <fstream>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("far_vgraph_loader");

  node->declare_parameter<std::string>("vgraph_file", "");
  node->declare_parameter<double>("wait_subscriber_timeout", 15.0);
  node->declare_parameter<int>("publish_count", 5);
  node->declare_parameter<double>("publish_interval", 0.5);

  const std::string vgraph_file = node->get_parameter("vgraph_file").as_string();
  const double wait_timeout = node->get_parameter("wait_subscriber_timeout").as_double();
  const int publish_count = node->get_parameter("publish_count").as_int();
  const double publish_interval = node->get_parameter("publish_interval").as_double();

  if (vgraph_file.empty()) {
    RCLCPP_INFO(node->get_logger(), "No FAR vgraph file configured; skipping restore.");
    rclcpp::shutdown();
    return 0;
  }

  std::ifstream graph_file(vgraph_file);
  if (!graph_file.good()) {
    RCLCPP_WARN(node->get_logger(), "FAR vgraph file does not exist or is unreadable: %s",
                vgraph_file.c_str());
    rclcpp::shutdown();
    return 0;
  }
  graph_file.close();

  auto pub = node->create_publisher<std_msgs::msg::String>("/read_file_dir", rclcpp::QoS(5));
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::duration<double>(wait_timeout);
  rclcpp::Rate wait_rate(10.0);
  while (rclcpp::ok() && pub->get_subscription_count() == 0 &&
         std::chrono::steady_clock::now() < deadline) {
    rclcpp::spin_some(node);
    wait_rate.sleep();
  }

  if (pub->get_subscription_count() == 0) {
    RCLCPP_WARN(node->get_logger(),
                "No /read_file_dir subscribers found; publishing FAR vgraph restore anyway.");
  }

  std_msgs::msg::String msg;
  msg.data = vgraph_file;
  const int count = publish_count > 0 ? publish_count : 1;
  const auto interval = std::chrono::duration<double>(publish_interval > 0.0 ? publish_interval : 0.5);
  for (int i = 0; rclcpp::ok() && i < count; ++i) {
    pub->publish(msg);
    RCLCPP_INFO(node->get_logger(), "Requested FAR vgraph restore from %s", vgraph_file.c_str());
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(interval);
  }

  rclcpp::shutdown();
  return 0;
}
