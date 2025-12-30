#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import time
import threading
import math

import json, socket
KAMI_IP = "192.168.168.168"
CTRL_PORT = 8484

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# sock.connect(("192.168.168.168", 8484))  # sets default peer
# sock.bind(("0.0.0.0", 8484))
sock.settimeout(2.0)

def send_command(command):
    data = json.dumps(command).encode("utf-8")
    try:
        slen = sock.sendto(data, (KAMI_IP, CTRL_PORT))
        print(f"Send success: {slen}")
    except Exception as e:
        print("Failed!", e)

def nav(vx, vy, vyaw):
    cmd = {
        'type': 'remote',
        'joystick': [vy, -vx, vyaw, 0],
        'button': [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    }
    send_command(cmd)

class RotateTest(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom/mc_odom',
            self.odom_callback,
            10
        )

        self.latest_msg = None
        self.current_yaw = 0.0
        self.xs = []
        self.ys = []
        
        # For tracking rotation
        self.initial_yaw = None
        self.target_yaw = None
        self.rotating = False

        self.get_logger().info("Subscribed to /odom/mc_odom (processing at 5 Hz)")

    def odom_callback(self, msg: Odometry):
        # Just cache the latest message
        self.latest_msg = msg
        self.current_yaw = self.get_yaw_from_msg(self.latest_msg)

    def get_yaw_from_msg(self, msg):
        """Extract yaw angle from quaternion in Odometry message"""
        q = msg.pose.pose.orientation
        # Convert quaternion to yaw
        yaw = -math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                        1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return yaw

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def log_pos(self):
        for i in range(300):
            pos = self.latest_msg.pose.pose.position
            print(f'{pos.x}, {pos.y}')
            time.sleep(0.1)
    
    def rotate(self, deg: float) -> bool:
        """
        Rotates the robot by the specified angle in degrees.
        """
        print(f"-> Rotate by {deg} degrees")
        initial_yaw = self.current_yaw
        delta_rad = math.radians(deg)

        accumulated_angle = 0.0
        previous_yaw = initial_yaw

        while True:
            cycle_start_time = time.time()
            current_yaw = self.current_yaw
            yaw_diff = current_yaw - previous_yaw

            # Normalize yaw difference to the range [-pi, pi]
            if yaw_diff > math.pi:
                yaw_diff -= 2 * math.pi
            elif yaw_diff < -math.pi:
                yaw_diff += 2 * math.pi

            accumulated_angle += yaw_diff
            previous_yaw = current_yaw

            remaining_angle = delta_rad - accumulated_angle

            print(f"-> Remaining angle: {math.degrees(remaining_angle):.2f} degrees, accumulated: {math.degrees(accumulated_angle):.2f} degrees")
            if abs(remaining_angle) < 0.01 or delta_rad * remaining_angle < 0:
                # If the remaining angle is small enough or we have overshot the target
                break

            vyaw = 1.0
            # print_t(f"-> vyaw: {vyaw:.2f} rad/s")
            print(f"-> Rotate: vyaw={vyaw:.2f} rad/s")
            nav(0.0, 0.0, vyaw)

            time.sleep(max(0, 0.1 - (time.time() - cycle_start_time)))
        nav(0.0, 0.0, 0.0)
        return True

def main():
    rclpy.init()
    node = RotateTest()

    spt = threading.Thread(target=rclpy.spin, args=(node,))
    spt.start()
    
    # Wait a bit for first odometry message
    time.sleep(1.0)
    
    try:
        # Example: rotate 90 degrees
        # node.rotate(90)
        node.log_pos()
        # time.sleep(10.0)  # Pause between rotations if doing multiple
        
        # Or for full rotation:
        # node.rotate(360)
        
    except KeyboardInterrupt:
        pass
    finally:
        # Make sure to stop the robot
        nav(0.0, 0.0, 0.0)
        
    node.destroy_node()
    rclpy.shutdown()
    spt.join()


if __name__ == '__main__':
    main()