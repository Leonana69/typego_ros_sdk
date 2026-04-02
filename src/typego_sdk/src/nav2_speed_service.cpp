#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/msg/parameter.hpp>
#include "typego_interface/srv/set_speed.hpp"

using SetSpeed = typego_interface::srv::SetSpeed;

class Nav2SpeedService : public rclcpp::Node
{
public:
  Nav2SpeedService()
  : Node("nav2_speed_service")
  {
    declare_parameter<double>("max_autonomy_speed", 1.5);
    declare_parameter<double>("max_reverse_speed", 0.35);
    declare_parameter<double>("max_angular_speed", 2.5);

    max_autonomy_speed_ = get_parameter("max_autonomy_speed").as_double();
    max_reverse_speed_ = get_parameter("max_reverse_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();

    // Use a separate callback group for parameter clients so their responses
    // can be processed while the service callback is running.
    param_cb_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    controller_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "controller_server", rmw_qos_profile_parameters, param_cb_group_);
    smoother_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "velocity_smoother", rmw_qos_profile_parameters, param_cb_group_);

    service_ = create_service<SetSpeed>(
      "typego/set_speed",
      std::bind(&Nav2SpeedService::handleSetSpeed, this,
                std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(get_logger(),
      "Nav2 speed service ready (max_autonomy=%.2f, reverse=%.2f, angular=%.2f)",
      max_autonomy_speed_, max_reverse_speed_, max_angular_speed_);
  }

private:
  void handleSetSpeed(
    const std::shared_ptr<SetSpeed::Request> request,
    std::shared_ptr<SetSpeed::Response> response)
  {
    double new_max = request->max_autonomy_speed;
    double new_reverse = request->max_reverse_speed;
    double new_angular = request->max_angular_speed;

    bool change_max = (new_max >= 0);
    bool change_reverse = (new_reverse >= 0);
    bool change_angular = (new_angular >= 0);

    if (!change_max && !change_reverse && !change_angular) {
      response->success = true;
      response->message = "No changes requested. Returning current values.";
      fillCurrentValues(response);
      return;
    }

    if (!change_max) new_max = max_autonomy_speed_;
    if (!change_reverse) new_reverse = max_reverse_speed_;
    if (!change_angular) new_angular = max_angular_speed_;

    // Validate
    if (new_max <= 0) {
      response->success = false;
      response->message = "max_autonomy_speed must be > 0, got " + std::to_string(new_max);
      fillCurrentValues(response);
      return;
    }
    if (new_reverse <= 0) {
      response->success = false;
      response->message = "max_reverse_speed must be > 0, got " + std::to_string(new_reverse);
      fillCurrentValues(response);
      return;
    }
    if (new_angular <= 0) {
      response->success = false;
      response->message = "max_angular_speed must be > 0, got " + std::to_string(new_angular);
      fillCurrentValues(response);
      return;
    }

    // Build parameter updates for MPPI controller
    std::vector<rclcpp::Parameter> controller_params;
    controller_params.emplace_back("FollowPath.vx_max", new_max);
    controller_params.emplace_back("FollowPath.vx_min", -new_reverse);
    controller_params.emplace_back("FollowPath.wz_max", new_angular);

    // Build parameter updates for velocity smoother
    std::vector<rclcpp::Parameter> smoother_params;
    smoother_params.emplace_back("max_velocity",
      std::vector<double>{new_max, 0.0, new_angular});
    smoother_params.emplace_back("min_velocity",
      std::vector<double>{-new_reverse, 0.0, -new_angular});

    // Apply — use wait_for() on futures instead of spin_until_future_complete
    // to avoid deadlocking the single-threaded executor.
    bool controller_ok = setParametersAsync(controller_param_client_, controller_params,
                                             "controller_server");
    bool smoother_ok = setParametersAsync(smoother_param_client_, smoother_params,
                                           "velocity_smoother");

    if (!controller_ok && !smoother_ok) {
      response->success = false;
      response->message = "Failed to update both controller_server and velocity_smoother. "
                          "Are the Nav2 nodes running?";
      fillCurrentValues(response);
      return;
    }

    // Update stored values
    max_autonomy_speed_ = new_max;
    max_reverse_speed_ = new_reverse;
    max_angular_speed_ = new_angular;

    std::string msg = "Speed updated:";
    if (!controller_ok) msg += " [WARN: controller_server unreachable]";
    if (!smoother_ok) msg += " [WARN: velocity_smoother unreachable]";
    msg += " max_autonomy=" + std::to_string(max_autonomy_speed_) +
           ", reverse=" + std::to_string(max_reverse_speed_) +
           ", angular=" + std::to_string(max_angular_speed_);

    RCLCPP_INFO(get_logger(), "%s", msg.c_str());

    response->success = true;
    response->message = msg;
    fillCurrentValues(response);
  }

  bool setParametersAsync(
    std::shared_ptr<rclcpp::AsyncParametersClient> client,
    const std::vector<rclcpp::Parameter> & params,
    const std::string & node_name)
  {
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "%s parameter service not available", node_name.c_str());
      return false;
    }

    auto future = client->set_parameters(params);

    // Wait on the future directly — safe because the param client uses its
    // own callback group which the multi-threaded executor can service.
    auto status = future.wait_for(std::chrono::seconds(5));
    if (status != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Timeout setting parameters on %s", node_name.c_str());
      return false;
    }

    auto results = future.get();
    for (size_t i = 0; i < results.size(); ++i) {
      if (!results[i].successful) {
        RCLCPP_WARN(get_logger(), "Failed to set param %zu on %s: %s",
                    i, node_name.c_str(), results[i].reason.c_str());
        return false;
      }
    }
    return true;
  }

  void fillCurrentValues(std::shared_ptr<SetSpeed::Response> response)
  {
    response->current_max_autonomy_speed = max_autonomy_speed_;
    response->current_max_reverse_speed = max_reverse_speed_;
    response->current_max_angular_speed = max_angular_speed_;
  }

  double max_autonomy_speed_;
  double max_reverse_speed_;
  double max_angular_speed_;

  rclcpp::CallbackGroup::SharedPtr param_cb_group_;
  std::shared_ptr<rclcpp::AsyncParametersClient> controller_param_client_;
  std::shared_ptr<rclcpp::AsyncParametersClient> smoother_param_client_;
  rclcpp::Service<SetSpeed>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  // Multi-threaded executor so the param client callback group can process
  // responses while the service callback is blocked waiting for them.
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<Nav2SpeedService>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
