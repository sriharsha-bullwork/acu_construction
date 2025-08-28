#!/usr/bin/env python3
import json
import math
import time
from typing import Dict, Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations

# how often we publish status / path to the UI
PUB_HZ = 5.0

def q_from_yaw(yaw_rad):
    x, y, z, w = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
    return x, y, z, w

def make_pose(x: float, y: float, yaw_deg: float, frame="map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    qx, qy, qz, qw = q_from_yaw(math.radians(float(yaw_deg)))
    p.pose.orientation.x = qx
    p.pose.orientation.y = qy
    p.pose.orientation.z = qz
    p.pose.orientation.w = qw
    return p

class WebWaypointManager(Node):
    """
    Bidirectional bridge for a web UI:

    SUBSCRIBE (from dashboard):
      /web_nav/command : std_msgs/String
        Commands (space-separated):
          - add <name> <x> <y> <yaw_deg>
          - delete <name>
          - goto <name>
          - pause
          - resume
          - cancel
          - clear_local
          - clear_both
          - list   (forces republish of points)

    PUBLISH (to dashboard):
      /web_nav/points : std_msgs/String (JSON list of {name,x,y,yaw})
      /web_nav/status : std_msgs/String (JSON dict: state, active_goal, distance_remaining, last_error)
      /web_nav/path   : nav_msgs/Path    (current planned global path)
      (robot pose: just use /odom directly in the web page)
    """

    def __init__(self):
        super().__init__("web_waypoint_manager")

        # storage: named waypoints
        self.points: Dict[str, Dict] = {}   # name -> {"x":..,"y":..,"yaw":..}

        # nav2 helper
        self.nav = BasicNavigator()

        # state
        self.active_goal_name: Optional[str] = None
        self.active_goal_pose: Optional[PoseStamped] = None
        self.distance_remaining: Optional[float] = None
        self.last_error: Optional[str] = None
        self._last_path_stamp = 0.0

        # pubs / subs
        self.cmd_sub = self.create_subscription(String, "/web_nav/command", self._on_cmd, 10)
        self.points_pub = self.create_publisher(String, "/web_nav/points", 10)
        self.status_pub = self.create_publisher(String, "/web_nav/status", 10)
        self.path_pub = self.create_publisher(Path, "/web_nav/path", 10)

        self.odom_sub = self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.robot_xy = (0.0, 0.0)

        # timers
        self.create_timer(1.0 / PUB_HZ, self._tick)

        # try to set initial pose at (0,0,0) if you run map->odom static transform like your sim
        init = make_pose(0.0, 0.0, 0.0)
        self.nav.setInitialPose(init)

        self.get_logger().info("WebWaypointManager ready. Send commands on /web_nav/command")

    # ---------------- subscriptions ----------------

    def _on_odom(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _on_cmd(self, msg: String):
        """Parse and execute commands from the web dashboard."""
        txt = msg.data.strip()
        if not txt:
            return

        parts = txt.split()
        cmd = parts[0].lower()

        try:
            if cmd == "add" and len(parts) == 5:
                name = parts[1]
                x = float(parts[2]); y = float(parts[3]); yaw = float(parts[4])
                self.points[name] = {"x": x, "y": y, "yaw": yaw}
                self._publish_points()
                self.get_logger().info(f"Added point '{name}' = ({x}, {y}, {yaw}°)")

            elif cmd == "delete" and len(parts) == 2:
                name = parts[1]
                if name in self.points:
                    del self.points[name]
                    self._publish_points()
                    self.get_logger().info(f"Deleted point '{name}'")

            elif cmd == "list":
                self._publish_points()

            elif cmd == "goto" and len(parts) == 2:
                name = parts[1]
                self._goto_point(name)

            elif cmd == "pause":
                self.nav.cancelTask()
                # keep active_goal_* so resume can resend
                self.get_logger().info("Paused current goal (canceled action).")

            elif cmd == "resume":
                if self.active_goal_pose is not None:
                    self._send_goal(self.active_goal_pose, self.active_goal_name)
                    self.get_logger().info("Resumed current goal.")
                else:
                    self.get_logger().warn("No goal to resume.")

            elif cmd == "cancel":
                self.nav.cancelTask()
                self.active_goal_name = None
                self.active_goal_pose = None
                self.distance_remaining = None
                self.get_logger().info("Canceled current goal.")

            elif cmd == "clear_local":
                try:
                    self.nav.clearLocalCostmap()
                finally:
                    self.get_logger().info("Cleared local costmap.")

            elif cmd == "clear_both":
                try:
                    self.nav.clearCostmap()
                except Exception:
                    try: self.nav.clearLocalCostmap()
                    except Exception: pass
                    try: self.nav.clearGlobalCostmap()
                    except Exception: pass
                self.get_logger().info("Cleared both costmaps.")

            else:
                self.get_logger().warn(f"Unknown or malformed command: '{txt}'")

        except Exception as ex:
            self.last_error = str(ex)
            self.get_logger().error(f"Command '{txt}' failed: {ex}")

    # ---------------- navigation helpers ----------------

    def _goto_point(self, name: str):
        if name not in self.points:
            self.last_error = f"Point '{name}' not found"
            self.get_logger().error(self.last_error)
            return

        p = self.points[name]
        goal = make_pose(p["x"], p["y"], p["yaw"])
        self._send_goal(goal, name)

    def _send_goal(self, goal: PoseStamped, name: Optional[str]):
        goal.header.stamp = self.get_clock().now().to_msg()
        self.active_goal_name = name
        self.active_goal_pose = goal
        self.distance_remaining = None
        self.last_error = None

        # publish a planned path to draw (optional)
        try:
            # Using Simple Commander to fetch a path without executing
            path = self.nav.getPath(initial_pose=None, goal_pose=goal)
            if path is not None:
                self.path_pub.publish(path)
                self._last_path_stamp = time.time()
        except Exception:
            # If planner service not yet ready, ignore
            pass

        # now actually go
        self.nav.goToPose(goal)

    # ---------------- periodic publish ----------------

    def _tick(self):
        # publish points (periodic light keep-alive)
        # (frontend also requests "list" to refresh, but this keeps lag down)
        self._publish_points(throttle=True)

        # publish status
        st = {
            "state": "idle" if self.active_goal_pose is None else "navigating",
            "active_goal": self.active_goal_name,
            "distance_remaining": None,
            "last_error": self.last_error,
            "robot_xy": {"x": self.robot_xy[0], "y": self.robot_xy[1]},
        }

        if self.active_goal_pose is not None:
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    st["state"] = "succeeded"
                    self.active_goal_name = None
                    self.active_goal_pose = None
                    self.distance_remaining = None
                elif result == TaskResult.CANCELED:
                    st["state"] = "paused_or_canceled"
                else:
                    st["state"] = "failed"
                    self.last_error = "Task failed"
            else:
                fb = self.nav.getFeedback()
                if fb and fb.distance_remaining is not None:
                    self.distance_remaining = float(fb.distance_remaining)
                st["distance_remaining"] = self.distance_remaining

                # refresh the published path occasionally (in case replanning is off,
                # this just helps UI recover if it opened late)
                if (time.time() - self._last_path_stamp) > 1.0:
                    try:
                        path = self.nav.getPath(initial_pose=None,
                                                goal_pose=self.active_goal_pose)
                        if path is not None:
                            self.path_pub.publish(path)
                            self._last_path_stamp = time.time()
                    except Exception:
                        pass

        self.status_pub.publish(String(data=json.dumps(st)))

    def _publish_points(self, throttle: bool = False):
        # light throttle so we don’t spam on every tick
        if throttle:
            # publish at 1 Hz even if PUB_HZ is faster
            if not hasattr(self, "_last_points_pub"):
                self._last_points_pub = 0.0
            now = time.time()
            if (now - self._last_points_pub) < 1.0:
                return
            self._last_points_pub = now

        pts = [{"name": n, "x": d["x"], "y": d["y"], "yaw": d["yaw"]}
               for n, d in sorted(self.points.items())]
        self.points_pub.publish(String(data=json.dumps(pts)))

def main():
    rclpy.init()
    node = WebWaypointManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
