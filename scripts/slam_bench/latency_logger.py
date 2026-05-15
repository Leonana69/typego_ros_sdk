#!/usr/bin/env python3
"""Per-message latency CSV for /state_estimation.

Same math as src/autonomy/slam/lightning_lm/scripts/slam_latency.py but writes
one row per received Odometry message instead of rolling stdout reports, so
summarize.py can compute p50/p95/p99 lag and mean rate offline.

CSV header: arrival_t_mono,header_stamp_s,lag_ms
"""

import argparse
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


class LatencyLogger(Node):
    def __init__(self, topic: str, out_path: str):
        super().__init__("latency_logger")
        self._fh = open(out_path, "w", buffering=1)
        self._fh.write("arrival_t_mono,header_stamp_s,lag_ms\n")

        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Odometry, topic, self._cb, qos)
        self.get_logger().info(f"logging latency: topic={topic} csv={out_path}")

    def _cb(self, msg: Odometry):
        # Skip uninitialized stamps (zero-stamped startup message).
        if msg.header.stamp.sec == 0:
            return
        arrival = time.monotonic()
        stamp_s = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        now_ns = self.get_clock().now().nanoseconds
        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        lag_ms = (now_ns - stamp_ns) / 1e6
        self._fh.write(f"{arrival:.6f},{stamp_s:.6f},{lag_ms:.3f}\n")

    def close(self):
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/state_estimation")
    p.add_argument("--out", required=True)
    args, _ = p.parse_known_args()

    rclpy.init()
    node = LatencyLogger(args.topic, args.out)

    # SIGINT/SIGTERM → set a flag; the spin_once loop exits within 0.2 s so the
    # CSV is flushed and closed cleanly.
    stop = {"v": False}

    def _stop(_s, _f):
        stop["v"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while not stop["v"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    sys.exit(0)


if __name__ == "__main__":
    main()
