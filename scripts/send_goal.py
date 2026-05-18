import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import math
import time

# action_msgs/GoalStatus codes -> human-readable label
_STATUS_LABEL = {
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
}


class GoalClient(Node):
    def __init__(self, robot_namespace=''):
        super().__init__('goal_sender')

        # Build namespaced action name
        if robot_namespace:
            action_name = f'/{robot_namespace}/navigate_to_pose'
        else:
            action_name = '/navigate_to_pose'

        self.client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f'Goal client initialized for action: {action_name}')

    def send_goal(self, x, y, yaw_deg):
        """Send a goal and block until navigation finishes.

        Returns True only on STATUS_SUCCEEDED; False if the goal was
        rejected, aborted (no path / stuck), or canceled.
        """
        goal_msg = NavigateToPose.Goal()
        yaw = math.radians(yaw_deg)
        goal_msg.pose.header.frame_id = 'map'  # Use namespaced frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info('Waiting for action server...')
        self.client.wait_for_server()

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw_deg}°')
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        self.goal_handle = send_future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by the action server!')
            return False
        self.get_logger().info('Goal accepted! Robot is navigating...')

        # Block until the server reports a terminal result.
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        wrapped = result_future.result()

        status = wrapped.status
        result = wrapped.result
        status_label = _STATUS_LABEL.get(status, f'UNKNOWN({status})')

        # NavigateToPose result carries error_code; error_msg exists on
        # newer distros only, so fetch it defensively.
        error_code = getattr(result, 'error_code', 0)
        error_msg = getattr(result, 'error_msg', '')

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation SUCCEEDED.')
            return True

        detail = f' (error_code={error_code}'
        detail += f', "{error_msg}")' if error_msg else ')'
        self.get_logger().error(
            f'Navigation failed: status={status_label}{detail}')
        return False

    def cancel_goal(self):
        if hasattr(self, 'goal_handle'):
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('Goal canceled')


# Main execution
rclpy.init()

# For robot2
node = GoalClient(robot_namespace='')
succeeded = node.send_goal(1.5, 0.0, 0)  # Set your (x, y, yaw)

# For single robot (no namespace)
# node = GoalClient(robot_namespace='')
# succeeded = node.send_goal(-0.0, -3.0, 0)

rclpy.shutdown()
raise SystemExit(0 if succeeded else 1)
