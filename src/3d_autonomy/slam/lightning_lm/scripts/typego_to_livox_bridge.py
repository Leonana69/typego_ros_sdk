#!/usr/bin/env python3
"""Bridge typego_interface/CustomMsg -> livox_ros_driver2/CustomMsg.

The go2/kami driver publishes Livox CustomMsg under the typego_interface
namespace; lightning-lm only subscribes to livox_ros_driver2 CustomMsg.
The two message definitions are field-identical (verified against
livox_ros_driver2 master), so this node does a straight per-field copy.

Run from launch with two args:
    --input  typego_interface CustomMsg topic   (default /livox/lidar_custom)
    --output livox_ros_driver2 CustomMsg topic  (default /lightning/livox_custom)
"""

import argparse
import sys

import rclpy
from rclpy.node import Node

from typego_interface.msg import CustomMsg as TypegoCustomMsg
from typego_interface.msg import CustomPoint as TypegoCustomPoint  # noqa: F401  (imported so rosidl typesupport loads)
from livox_ros_driver2.msg import CustomMsg as LivoxCustomMsg
from livox_ros_driver2.msg import CustomPoint as LivoxCustomPoint


class TypegoToLivoxBridge(Node):
    def __init__(self, input_topic: str, output_topic: str):
        super().__init__('typego_to_livox_bridge')
        self._pub = self.create_publisher(LivoxCustomMsg, output_topic, 10)
        self._sub = self.create_subscription(
            TypegoCustomMsg, input_topic, self._on_msg, 10)
        self.get_logger().info(
            f'bridging {input_topic} -> {output_topic} '
            f'(typego_interface -> livox_ros_driver2)')

    def _on_msg(self, src: TypegoCustomMsg) -> None:
        out = LivoxCustomMsg()
        out.header = src.header
        out.timebase = src.timebase
        out.point_num = src.point_num
        out.lidar_id = src.lidar_id
        out.rsvd = src.rsvd
        out.points = [
            self._copy_point(p) for p in src.points
        ]
        self._pub.publish(out)

    @staticmethod
    def _copy_point(p) -> LivoxCustomPoint:
        q = LivoxCustomPoint()
        q.offset_time = p.offset_time
        q.x = p.x
        q.y = p.y
        q.z = p.z
        q.reflectivity = p.reflectivity
        q.tag = p.tag
        q.line = p.line
        return q


def main(argv=None):
    argv = list(sys.argv if argv is None else argv)
    rclpy.init(args=argv)

    parser = argparse.ArgumentParser()
    parser.add_argument('--input', default='/livox/lidar_custom')
    parser.add_argument('--output', default='/lightning/livox_custom')
    # ros2 launch always appends --ros-args ...; argparse ignores it via parse_known_args.
    args, _ = parser.parse_known_args(argv[1:])

    node = TypegoToLivoxBridge(args.input, args.output)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
