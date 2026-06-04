#include <rclcpp/rclcpp.hpp>
#include <map>
#include <string>
#include <vector>
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
    // Acceleration limits — defaults mirror the velocity_smoother config
    // (max_accel/max_decel = [2.5, _, 4.5]). Stored as positive magnitudes.
    declare_parameter<double>("max_accel", 2.5);
    declare_parameter<double>("max_decel", 2.5);
    declare_parameter<double>("max_angular_accel", 4.5);

    max_autonomy_speed_ = get_parameter("max_autonomy_speed").as_double();
    max_reverse_speed_ = get_parameter("max_reverse_speed").as_double();
    max_angular_speed_ = get_parameter("max_angular_speed").as_double();
    max_accel_ = get_parameter("max_accel").as_double();
    max_decel_ = get_parameter("max_decel").as_double();
    max_angular_accel_ = get_parameter("max_angular_accel").as_double();

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
      "Nav2 speed service ready (max_autonomy=%.2f, reverse=%.2f, angular=%.2f, "
      "accel=%.2f, decel=%.2f, ang_accel=%.2f)",
      max_autonomy_speed_, max_reverse_speed_, max_angular_speed_,
      max_accel_, max_decel_, max_angular_accel_);
  }

private:
  void handleSetSpeed(
    const std::shared_ptr<SetSpeed::Request> request,
    std::shared_ptr<SetSpeed::Response> response)
  {
    double new_max = request->max_autonomy_speed;
    double new_reverse = request->max_reverse_speed;
    double new_angular = request->max_angular_speed;
    double new_accel = request->max_accel;
    double new_decel = request->max_decel;
    double new_angular_accel = request->max_angular_accel;

    bool change_max = (new_max >= 0);
    bool change_reverse = (new_reverse >= 0);
    bool change_angular = (new_angular >= 0);
    bool change_accel = (new_accel >= 0);
    bool change_decel = (new_decel >= 0);
    bool change_angular_accel = (new_angular_accel >= 0);

    if (!change_max && !change_reverse && !change_angular &&
        !change_accel && !change_decel && !change_angular_accel) {
      response->success = true;
      response->message = "No changes requested. Returning current values.";
      fillCurrentValues(response);
      return;
    }

    // Validate only the fields actually being changed; the change flag already
    // guarantees the value is >= 0, so this just rejects an explicit 0.
    // Unchanged fields keep their current values and are never re-validated,
    // re-written, or echoed from a possibly-stale cache.
    if (change_max && new_max <= 0) {
      response->success = false;
      response->message = "max_autonomy_speed must be > 0, got " + std::to_string(new_max);
      fillCurrentValues(response);
      return;
    }
    if (change_reverse && new_reverse <= 0) {
      response->success = false;
      response->message = "max_reverse_speed must be > 0, got " + std::to_string(new_reverse);
      fillCurrentValues(response);
      return;
    }
    if (change_angular && new_angular <= 0) {
      response->success = false;
      response->message = "max_angular_speed must be > 0, got " + std::to_string(new_angular);
      fillCurrentValues(response);
      return;
    }
    if (change_accel && new_accel <= 0) {
      response->success = false;
      response->message = "max_accel must be > 0, got " + std::to_string(new_accel);
      fillCurrentValues(response);
      return;
    }
    if (change_decel && new_decel <= 0) {
      response->success = false;
      response->message = "max_decel must be > 0, got " + std::to_string(new_decel);
      fillCurrentValues(response);
      return;
    }
    if (change_angular_accel && new_angular_accel <= 0) {
      response->success = false;
      response->message = "max_angular_accel must be > 0, got " + std::to_string(new_angular_accel);
      fillCurrentValues(response);
      return;
    }

    // ---- MPPI controller: write ONLY the speed scalars that changed. Although
    // nav2_params.yaml lists FollowPath.ax_max/ax_min/az_max, the installed MPPI
    // (Humble v1.1.x) never declares them — ControlConstraints is only
    // {vx_max, vx_min, vy, wz}, and Nav2 lifecycle nodes don't auto-declare YAML
    // overrides — so those keys are inert and setting them would be rejected as
    // undeclared params. Acceleration therefore lives solely in the
    // velocity_smoother, and an accel-only request never touches the controller. ----
    std::vector<rclcpp::Parameter> controller_params;
    if (change_max)     controller_params.emplace_back("FollowPath.vx_max", new_max);
    if (change_reverse) controller_params.emplace_back("FollowPath.vx_min", -new_reverse);
    if (change_angular) controller_params.emplace_back("FollowPath.wz_max", new_angular);

    // ---- velocity_smoother: its [x, y, theta] vectors enforce both the speed
    // limits and the ONLY acceleration limits in the stack. Rewrite a vector
    // only when one of its elements changed, and read the live value first so
    // the untouched elements (the y axis, and the linear/angular partner) keep
    // their actual current values rather than being clobbered by a possibly
    // stale cache — this honours "omitted field = no change" even under a
    // custom nav2_params_file. ----
    const bool need_max_vel = change_max || change_angular;
    const bool need_min_vel = change_reverse || change_angular;
    const bool need_max_acc = change_accel || change_angular_accel;
    const bool need_max_dec = change_decel || change_angular_accel;

    std::vector<std::string> smoother_names;
    if (need_max_vel) smoother_names.push_back("max_velocity");
    if (need_min_vel) smoother_names.push_back("min_velocity");
    if (need_max_acc) smoother_names.push_back("max_accel");
    if (need_max_dec) smoother_names.push_back("max_decel");

    const bool controller_needed = !controller_params.empty();
    const bool smoother_needed = !smoother_names.empty();

    std::vector<rclcpp::Parameter> smoother_params;
    std::map<std::string, std::vector<double>> cur;  // live smoother values, kept for rollback
    bool smoother_built = true;
    if (smoother_needed) {
      if (!getArrayParams(smoother_param_client_, smoother_names, cur, "velocity_smoother")) {
        smoother_built = false;
      } else {
        if (need_max_vel) {
          std::vector<double> v = cur["max_velocity"];
          if (v.size() < 3) smoother_built = false;
          else {
            if (change_max)     v[0] = new_max;
            if (change_angular) v[2] = new_angular;
            smoother_params.emplace_back("max_velocity", v);
          }
        }
        if (need_min_vel) {
          std::vector<double> v = cur["min_velocity"];
          if (v.size() < 3) smoother_built = false;
          else {
            if (change_reverse) v[0] = -new_reverse;
            if (change_angular) v[2] = -new_angular;
            smoother_params.emplace_back("min_velocity", v);
          }
        }
        if (need_max_acc) {
          std::vector<double> v = cur["max_accel"];
          if (v.size() < 3) smoother_built = false;
          else {
            if (change_accel)         v[0] = new_accel;
            if (change_angular_accel) v[2] = new_angular_accel;
            smoother_params.emplace_back("max_accel", v);
          }
        }
        if (need_max_dec) {
          std::vector<double> v = cur["max_decel"];
          if (v.size() < 3) smoother_built = false;
          else {
            if (change_decel)         v[0] = -new_decel;
            if (change_angular_accel) v[2] = -new_angular_accel;
            smoother_params.emplace_back("max_decel", v);
          }
        }
      }
    }

    // ---- Apply atomically. The smoother carries both the speed limits and the
    // ONLY acceleration limits, so apply it FIRST: if it fails, nothing has been
    // written anywhere. Only then update the controller; if THAT fails, roll the
    // smoother back to the values we just read so the call stays all-or-nothing
    // (and never falsely reports "no values changed" while a live node was
    // mutated). An accel-only request never needs the controller, so it succeeds
    // or fails purely on the smoother. ----
    if (smoother_needed &&
        !(smoother_built &&
          setParametersAsync(smoother_param_client_, smoother_params, "velocity_smoother"))) {
      response->success = false;
      response->message =
        "Update failed (velocity_smoother not updated); no values changed. "
        "Is the Nav2 velocity_smoother running?";
      fillCurrentValues(response);
      return;
    }

    if (controller_needed &&
        !setParametersAsync(controller_param_client_, controller_params, "controller_server")) {
      // Controller write failed after the smoother already applied — restore the
      // smoother to its previous values so no partial change sticks.
      std::vector<rclcpp::Parameter> restore;
      for (const auto & name : smoother_names) restore.emplace_back(name, cur[name]);
      const bool rolled_back =
        setParametersAsync(smoother_param_client_, restore, "velocity_smoother");
      response->success = false;
      response->message =
        std::string("Update failed (controller_server not updated); no values changed.") +
        (rolled_back ? " velocity_smoother rolled back."
                     : " WARNING: velocity_smoother rollback FAILED — limits may be inconsistent.") +
        " Is the Nav2 controller_server running?";
      fillCurrentValues(response);
      return;
    }

    // Commit only the fields that changed (all required subsystems succeeded).
    if (change_max)           max_autonomy_speed_ = new_max;
    if (change_reverse)       max_reverse_speed_ = new_reverse;
    if (change_angular)       max_angular_speed_ = new_angular;
    if (change_accel)         max_accel_ = new_accel;
    if (change_decel)         max_decel_ = new_decel;
    if (change_angular_accel) max_angular_accel_ = new_angular_accel;

    std::string msg = "Updated:";
    if (change_max)           msg += " max_autonomy=" + std::to_string(max_autonomy_speed_);
    if (change_reverse)       msg += " reverse=" + std::to_string(max_reverse_speed_);
    if (change_angular)       msg += " angular=" + std::to_string(max_angular_speed_);
    if (change_accel)         msg += " accel=" + std::to_string(max_accel_);
    if (change_decel)         msg += " decel=" + std::to_string(max_decel_);
    if (change_angular_accel) msg += " ang_accel=" + std::to_string(max_angular_accel_);

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

  // Read current double-array parameters from a node so we can preserve the
  // vector elements we are not changing. Returns false if the node is
  // unreachable, the read times out, the count is wrong, or a value is not a
  // double array.
  bool getArrayParams(
    std::shared_ptr<rclcpp::AsyncParametersClient> client,
    const std::vector<std::string> & names,
    std::map<std::string, std::vector<double>> & out,
    const std::string & node_name)
  {
    if (names.empty()) return true;
    if (!client->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_WARN(get_logger(), "%s parameter service not available", node_name.c_str());
      return false;
    }
    auto future = client->get_parameters(names);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_WARN(get_logger(), "Timeout reading parameters from %s", node_name.c_str());
      return false;
    }
    auto values = future.get();
    if (values.size() != names.size()) {
      RCLCPP_WARN(get_logger(), "Unexpected parameter count from %s", node_name.c_str());
      return false;
    }
    for (size_t i = 0; i < names.size(); ++i) {
      if (values[i].get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        RCLCPP_WARN(get_logger(), "%s.%s is not a double array",
                    node_name.c_str(), names[i].c_str());
        return false;
      }
      out[names[i]] = values[i].as_double_array();
    }
    return true;
  }

  void fillCurrentValues(std::shared_ptr<SetSpeed::Response> response)
  {
    response->current_max_autonomy_speed = max_autonomy_speed_;
    response->current_max_reverse_speed = max_reverse_speed_;
    response->current_max_angular_speed = max_angular_speed_;
    response->current_max_accel = max_accel_;
    response->current_max_decel = max_decel_;
    response->current_max_angular_accel = max_angular_accel_;
  }

  double max_autonomy_speed_;
  double max_reverse_speed_;
  double max_angular_speed_;
  double max_accel_;
  double max_decel_;
  double max_angular_accel_;

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
