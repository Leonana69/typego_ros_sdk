#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib
matplotlib.use("Agg")  # headless backend
import matplotlib.pyplot as plt


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom/mc_odom',
            self.odom_callback,
            10
        )

        # 5 Hz timer (0.2 s)
        self.timer = self.create_timer(0.2, self.timer_callback)

        self.latest_msg = None
        self.xs = []
        self.ys = []

        self.get_logger().info("Subscribed to /odom/mc_odom (processing at 5 Hz)")

    def odom_callback(self, msg: Odometry):
        # Just cache the latest message
        self.latest_msg = msg

    def timer_callback(self):
        if self.latest_msg is None:
            return

        x = self.latest_msg.pose.pose.position.x
        y = self.latest_msg.pose.pose.position.y

        self.xs.append(x)
        self.ys.append(y)

        self.get_logger().info(f"Plot update @5Hz: x={x:.3f}, y={y:.3f}")

        self.save_plot()

    def save_plot(self):
        plt.figure(figsize=(5, 5))
        plt.plot(self.xs, self.ys, marker='o', markersize=3, linewidth=1)

        plt.xlim(0, 2)
        plt.ylim(0, 2)

        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.title("Odometry Trajectory (/odom/mc_odom)")
        plt.grid(True)
        plt.axis("equal")

        plt.tight_layout()
        plt.savefig("odom_trajectory.png")
        plt.close()


def main():
    rclpy.init()
    node = OdomPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

