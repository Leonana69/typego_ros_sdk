import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time
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
        goal_msg = NavigateToPose.Goal()
        yaw = math.radians(yaw_deg)
        goal_msg.pose.header.frame_id = 'map'  # Use namespaced frame
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(f'Waiting for action server...')
        self.client.wait_for_server()
        
        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw_deg}°')
        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
        else:
            self.get_logger().info('Goal accepted! Robot is navigating...')

    def cancel_goal(self):
        if hasattr(self, 'goal_handle'):
            self.goal_handle.cancel_goal_async()
            self.get_logger().info('Goal canceled')

# Main execution
rclpy.init()

# For robot2
node = GoalClient(robot_namespace='')
node.send_goal(1.5, 0.0, 0)  # Set your (x, y, yaw)

# For single robot (no namespace)
# node = GoalClient(robot_namespace='')
# node.send_goal(-0.0, -3.0, 0)

rclpy.shutdown()
