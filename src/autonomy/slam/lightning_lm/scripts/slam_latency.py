#!/usr/bin/env python3
"""Measure end-to-end SLAM latency.

Subscribes to /state_estimation (or any nav_msgs/Odometry topic) and prints
rolling stats on:
  - publish rate (Hz)
  - "freshness" = (now - msg.header.stamp) — how stale each pose is when the
    planner gets it. This is what actually determines whether the local
    planner reacts in time to obstacles.

Usage:
    # Lightning backend running:
    python3 slam_latency.py
    # Arise backend running (same command — same topic):
    python3 slam_latency.py

    # Custom topic:
    python3 slam_latency.py --topic /integrated_to_init
"""

import argparse
import statistics
import sys
import time
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class LatencyProbe(Node):
    def __init__(self, topic: str, window: int):
        super().__init__('slam_latency_probe')
        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        # Try reliable first; many SLAM publishers default to reliable.
        self._sub = self.create_subscription(Odometry, topic, self._cb, qos)
        self._lags_ms = deque(maxlen=window)
        self._arrivals = deque(maxlen=window)
        self._t_report = time.monotonic()
        self.get_logger().info(f'measuring latency on {topic} (window={window})')

    def _cb(self, msg: Odometry):
        now = self.get_clock().now()
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        lag_ms = (now.nanoseconds - stamp_ns) / 1e6
        self._lags_ms.append(lag_ms)
        self._arrivals.append(time.monotonic())

        # Print a row every 2 s.
        if time.monotonic() - self._t_report >= 2.0:
            self._t_report = time.monotonic()
            self._report()

    def _report(self):
        n = len(self._lags_ms)
        if n < 2:
            return
        lags = sorted(self._lags_ms)
        p50 = lags[n // 2]
        p95 = lags[int(n * 0.95)]
        p99 = lags[int(n * 0.99)]
        # Rate from arrival deltas:
        deltas = [b - a for a, b in zip(self._arrivals, list(self._arrivals)[1:])]
        rate = 1.0 / statistics.mean(deltas) if deltas else 0.0
        print(
            f'n={n:4d}  rate={rate:5.1f} Hz  '
            f'lag p50={p50:6.1f} ms  p95={p95:6.1f} ms  p99={p99:6.1f} ms',
            flush=True,
        )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/state_estimation')
    parser.add_argument('--window', type=int, default=200)
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = LatencyProbe(args.topic, args.window)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
