#!/usr/bin/env python3
import math, sys, threading, subprocess, shlex
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import tf_transformations

def q_from_yaw(yaw):
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def make_pose(x, y, yaw_deg, frame="map", stamp=None):
    p = PoseStamped()
    p.header.frame_id = frame
    if stamp is not None:
        p.header.stamp = stamp
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    p.pose.orientation = q_from_yaw(math.radians(yaw_deg))
    return p

def shell(cmd: str, timeout: float = 3.0):
    try:
        subprocess.run(shlex.split(cmd), check=True, timeout=timeout)
        return True
    except Exception as e:
        print(f"[shell] {cmd} -> {e}")
        return False

class Commander(Node):
    def __init__(self):
        super().__init__("waypoint_commander")
        self.nav = BasicNavigator()

        print("Waiting for Nav2 (bt_navigator) to activate...")
        # matches your installed commander
        self.nav._waitForNodeToActivate("bt_navigator")
        print("Nav2 is active.")

        self.paused = False
        self.cancel_requested = False
        self.quit_requested = False
        self.soft_reset_requested = False
        self.idx = 0
        self.waypoints = []

        # Input thread
        threading.Thread(target=self._input_loop, daemon=True).start()

    def _input_loop(self):
        print("\nControls: [p]=pause  [r]=resume  [c]=cancel+skip  [x]=clear local costmap  [z]=soft reset  [q]=quit\n")
        for line in sys.stdin:
            cmd = line.strip().lower()
            if cmd == 'p':
                self.paused = True
                self.nav.cancelTask()
                print("Paused current goal.")
            elif cmd == 'r':
                if self.paused:
                    self.paused = False
                    print("Resuming…")
                else:
                    print("Not paused.")
            elif cmd == 'c':
                self.cancel_requested = True
                self.nav.cancelTask()
                print("Canceled current goal (will skip to next).")
            elif cmd == 'x':
                self.clear_local_costmap()
            elif cmd == 'z':
                self.soft_reset_requested = True
                self.nav.cancelTask()
                print("Soft reset requested (cancel + clear local/global).")
            elif cmd == 'q':
                self.quit_requested = True
                self.nav.cancelTask()
                print("Quitting after current action…")
                break

    # --- Utilities that avoid client collisions ---

    def clear_local_costmap(self):
        if hasattr(self.nav, "clearLocalCostmap"):
            try:
                self.nav.clearLocalCostmap()
                print("Local costmap cleared (navigator).")
                return
            except Exception as e:
                print(f"navigator.clearLocalCostmap failed: {e}")
        # Fallback via CLI
        ok = shell("ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty {}")
        print("Local costmap cleared (CLI)." if ok else "Local costmap clear failed.")

    def clear_all_costmaps(self):
        if hasattr(self.nav, "clearAllCostmaps"):
            try:
                self.nav.clearAllCostmaps()
                print("All costmaps cleared (navigator).")
                return
            except Exception as e:
                print(f"navigator.clearAllCostmaps failed: {e}")
        # Fallback via CLI (local + global)
        shell("ros2 service call /local_costmap/clear_entirely_local_costmap std_srvs/srv/Empty {}")
        shell("ros2 service call /global_costmap/clear_entirely_global_costmap std_srvs/srv/Empty {}")
        print("All costmaps cleared (CLI).")

    def soft_reset_nav2(self):
        """
        Non-destructive reset: cancel task, clear costmaps.
        (Lifecycle flip is overkill in sim; relaunch if truly wedged.)
        """
        self.nav.cancelTask()
        self.clear_all_costmaps()
        # brief spin to process any transitions
        rclpy.spin_once(self, timeout_sec=0.1)

    # ---------------------------------------------

    def run(self):
        now = self.nav.get_clock().now().to_msg()
        init = make_pose(0.0, 0.0, 0.0, stamp=now)
        self.nav.setInitialPose(init)

        # Define route
        self.waypoints = [
            make_pose(5.0, 0.0,   0.0,  stamp=now),
            make_pose(15.0, 1.0,  90.0,  stamp=now),
            make_pose(0.0, 5.0, 180.0,  stamp=now),
            make_pose(0.0, 0.0, -90.0,  stamp=now),
        ]

        self.idx = 0
        while self.idx < len(self.waypoints) and not self.quit_requested:
            pose = self.waypoints[self.idx]
            print(f"Going to waypoint {self.idx+1}/{len(self.waypoints)}")
            self.nav.goToPose(pose)

            while not self.nav.isTaskComplete():
                fb = self.nav.getFeedback()
                # if fb and getattr(fb, "distance_remaining", None) is not None:
                    # print(f"Remaining: {fb.distance_remaining:.2f} m")
                rclpy.spin_once(self, timeout_sec=0.1)

                if self.soft_reset_requested:
                    self.soft_reset_requested = False
                    print("Soft resetting Nav2…")
                    self.soft_reset_nav2()
                    # Re-issue same waypoint after reset
                    print("Re-issuing current waypoint…")
                    self.nav.goToPose(pose)

                if self.paused:
                    print("Paused. Type 'r' to resume.")
                    while self.paused and not self.quit_requested:
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if self.quit_requested:
                        break
                    print("Resumed. Re-issuing current waypoint…")
                    self.nav.goToPose(pose)

                if self.cancel_requested:
                    self.cancel_requested = False
                    print("Skipping to next waypoint…")
                    break  # exit inner loop, advance idx

                if self.quit_requested:
                    break

            if self.quit_requested:
                break

            # If we canceled to skip, advance and continue
            if not self.paused:
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    print("Reached waypoint.")
                    self.idx += 1
                elif result == TaskResult.CANCELED:
                    print("Goal canceled.")
                    self.idx += 1  # treat as skip
                else:
                    print("Goal failed — clearing local costmap and retrying once.")
                    self.clear_local_costmap()
                    self.nav.goToPose(pose)
                    while not self.nav.isTaskComplete():
                        rclpy.spin_once(self, timeout_sec=0.1)
                    if self.nav.getResult() != TaskResult.SUCCEEDED:
                        print("Retry failed. Skipping.")
                        self.idx += 1

        print("Done. Shutting down.")

def main():
    rclpy.init()
    node = Commander()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
