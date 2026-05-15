#!/usr/bin/env python3
"""Log /state_estimation as a TUM trajectory and a final-vs-start drift summary.

TUM lines:  t tx ty tz qx qy qz qw  (t in seconds, from msg.header.stamp)

On SIGINT (sent by the orchestrator after the bag finishes), this writes
drift.json next to the TUM file:
    {
      "translation_m":  ||p_last - p_first||_2,
      "yaw_deg":        wrapped angle between first and last yaw,
      "path_length_m":  sum of consecutive position deltas,
      "n_poses":        number of received poses,
      "duration_s":     last_stamp - first_stamp
    }

For a closed loop (start == end physically), translation_m and yaw_deg are
the absolute drift.
"""

import argparse
import json
import math
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


class TrajectoryLogger(Node):
    def __init__(self, topic: str, tum_path: str, drift_path: str):
        super().__init__("trajectory_logger")
        self._tum_path = tum_path
        self._drift_path = drift_path
        self._fh = open(tum_path, "w", buffering=1)  # line-buffered
        self._fh.write("# timestamp tx ty tz qx qy qz qw\n")

        self._first = None  # (t, x, y, z, yaw)
        self._last = None
        self._prev_xyz = None
        self._path_len = 0.0
        self._n = 0

        qos = QoSProfile(
            depth=50,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(Odometry, topic, self._cb, qos)
        self.get_logger().info(
            f"logging trajectory: topic={topic} tum={tum_path} drift={drift_path}"
        )

    def _cb(self, msg: Odometry):
        # Skip uninitialized stamps — some backends emit a zero-stamped pose at
        # startup, which would otherwise dominate the duration computation.
        if msg.header.stamp.sec == 0:
            return
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._fh.write(
            f"{t:.6f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n"
        )
        yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        entry = (t, p.x, p.y, p.z, yaw)
        if self._first is None:
            self._first = entry
        self._last = entry
        if self._prev_xyz is not None:
            dx = p.x - self._prev_xyz[0]
            dy = p.y - self._prev_xyz[1]
            dz = p.z - self._prev_xyz[2]
            self._path_len += math.sqrt(dx * dx + dy * dy + dz * dz)
        self._prev_xyz = (p.x, p.y, p.z)
        self._n += 1

    def write_drift(self):
        if self._first is None or self._last is None or self._n < 2:
            drift = {
                "translation_m": None,
                "yaw_deg": None,
                "path_length_m": 0.0,
                "n_poses": self._n,
                "duration_s": 0.0,
                "note": "fewer than 2 poses received",
            }
        else:
            t0, x0, y0, z0, yaw0 = self._first
            t1, x1, y1, z1, yaw1 = self._last
            dx, dy, dz = x1 - x0, y1 - y0, z1 - z0
            drift = {
                "translation_m": math.sqrt(dx * dx + dy * dy + dz * dz),
                "yaw_deg": math.degrees(wrap_pi(yaw1 - yaw0)),
                "path_length_m": self._path_len,
                "n_poses": self._n,
                "duration_s": t1 - t0,
            }
        with open(self._drift_path, "w") as f:
            json.dump(drift, f, indent=2)
        self.get_logger().info(f"wrote {self._drift_path}: {drift}")

    def close(self):
        try:
            self._fh.flush()
            self._fh.close()
        except Exception:
            pass


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/state_estimation")
    p.add_argument("--out", required=True, help="TUM trajectory output path")
    p.add_argument("--drift-out", required=True, help="drift.json output path")
    args, _ = p.parse_known_args()

    rclpy.init()
    node = TrajectoryLogger(args.topic, args.out, args.drift_out)

    # SIGINT/SIGTERM → set a flag; the spin_once loop notices within 0.2 s and
    # exits so the finalizers (write_drift/close) always run. A handler that
    # called rclpy.shutdown() would not reliably break rclpy.spin().
    stop = {"v": False}

    def _stop(_signum, _frame):
        stop["v"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    try:
        while not stop["v"] and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.write_drift()
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
