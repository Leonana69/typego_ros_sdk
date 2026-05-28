#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import json
import os
import sys
import threading


# ── Configuration ─────────────────────────────────────────────────────────────
# Override MAP_NAME and PATROL_LIST via command-line args, or edit here.
MAP_NAME    = "full-map"          # e.g. "full-map"  →  Map-full-map/waypoints.json
PATROL_LIST = [1]       # waypoint indices to visit in order (repeats)
ROBOT_NS    = ""                  # robot namespace, empty for single robot
# ──────────────────────────────────────────────────────────────────────────────


def load_waypoints(map_name: str) -> dict:
    """Load waypoints.json for the given map name.
    Returns a dict: {index: (x, y)} using the id field.
    """
    script_dir = os.path.dirname(os.path.abspath(__file__))
    json_path = os.path.join(
        script_dir, "..", "src", "typego_sdk", "resource",
        f"Map-{map_name}", "waypoints.json"
    )
    json_path = os.path.normpath(json_path)

    if not os.path.isfile(json_path):
        print(f"[ERROR] Waypoints file not found: {json_path}")
        sys.exit(1)

    waypoints = {}
    with open(json_path) as f:
        doc = json.load(f)
    items = doc.get("waypoints", doc) if isinstance(doc, dict) else doc
    for item in items:
        waypoints[int(item["id"])] = (float(item["x"]), float(item["y"]))

    print(f"[INFO] Loaded {len(waypoints)} waypoints from: {json_path}")
    return waypoints


class PatrolClient(Node):
    def __init__(self, robot_namespace: str = ""):
        super().__init__("patrol_client")
        action_name = f"/{robot_namespace}/navigate_to_pose" if robot_namespace \
                      else "/navigate_to_pose"
        self.client = ActionClient(self, NavigateToPose, action_name)
        self.get_logger().info(f"Patrol client ready on action: {action_name}")
        self._goal_handle = None

    def send_goal_and_wait(self, x: float, y: float, yaw_deg: float = 0.0) -> bool:
        """Send a navigation goal and block until it succeeds or fails.
        Returns True on success, False otherwise.
        """
        goal_msg = NavigateToPose.Goal()
        yaw = math.radians(yaw_deg)
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info("Waiting for action server…")
        self.client.wait_for_server()

        self.get_logger().info(f"Sending goal → x={x:.3f}, y={y:.3f}, yaw={yaw_deg}°")
        send_future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_future)

        self._goal_handle = send_future.result()
        if not self._goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted — navigating…")
        result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        status = result_future.result().status
        # action_msgs/GoalStatus: SUCCEEDED = 4
        if status == 4:
            self.get_logger().info("Waypoint reached!")
            return True
        else:
            self.get_logger().warn(f"Navigation ended with status {status}")
            return False

    def cancel_current_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info("Cancelling current goal…")
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


def _wait_for_stop_key(stop_event: threading.Event):
    """Background thread: pressing Enter sets the stop flag."""
    print("\n[INFO] Press  ENTER  at any time to stop the patrol.\n")
    input()
    stop_event.set()
    print("[INFO] Stop requested — finishing current waypoint…")


def main():
    # ── Parse optional CLI args ────────────────────────────────────────────────
    map_name    = MAP_NAME
    patrol_list = PATROL_LIST
    robot_ns    = ROBOT_NS

    if len(sys.argv) > 1:
        map_name = sys.argv[1]
    if len(sys.argv) > 2:
        # Accept comma-separated indices: e.g.  "1,2,3,2"
        patrol_list = [int(i) for i in sys.argv[2].split(",")]
    if len(sys.argv) > 3:
        robot_ns = sys.argv[3]

    print(f"[INFO] Map      : {map_name}")
    print(f"[INFO] Patrol   : {patrol_list}")
    print(f"[INFO] Robot NS : '{robot_ns}' (empty = single robot)")

    waypoints = load_waypoints(map_name)

    # Validate patrol list against loaded waypoints
    for idx in patrol_list:
        if idx not in waypoints:
            print(f"[ERROR] Patrol index {idx} not found in waypoints. "
                  f"Available: {sorted(waypoints)}")
            sys.exit(1)

    # ── ROS init ───────────────────────────────────────────────────────────────
    rclpy.init()
    node = PatrolClient(robot_namespace=robot_ns)

    # Background thread to watch for Enter key
    stop_event = threading.Event()
    key_thread = threading.Thread(target=_wait_for_stop_key, args=(stop_event,), daemon=True)
    key_thread.start()

    # ── Patrol loop ────────────────────────────────────────────────────────────
    lap = 0
    try:
        while not stop_event.is_set():
            lap += 1
            print(f"\n[INFO] ── Patrol lap {lap} ──────────────────────────────")
            for wp_idx in patrol_list:
                if stop_event.is_set():
                    break
                x, y = waypoints[wp_idx]
                print(f"[INFO] Waypoint {wp_idx}  ({x:.3f}, {y:.3f})")
                node.send_goal_and_wait(x, y)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl-C received.")
    finally:
        node.cancel_current_goal()
        node.destroy_node()
        rclpy.shutdown()
        print("[INFO] Patrol stopped. Bye!")


if __name__ == "__main__":
    main()
