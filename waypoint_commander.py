#!/usr/bin/env python3
import sys, math, select, termios, tty, time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from lifecycle_msgs.srv import GetState
from lifecycle_msgs.msg import State
import tf_transformations

# --------- Tunables ---------
BLOCK_TIMEOUT_SEC = 60.0   # max wait per blockage
PROGRESS_EPS_M   = 0.03    # required improvement in distance to count as progress
RETRY_PER_GOAL   = 1       # automatic retries per goal after a failure
# ----------------------------

def q_from_yaw(yaw_rad: float) -> Quaternion:
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw_rad)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def make_pose(x: float, y: float, yaw_deg: float, frame: str = "map") -> PoseStamped:
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    p.pose.orientation = q_from_yaw(math.radians(float(yaw_deg)))
    return p

class Commander(Node):
    def __init__(self, waypoints: List[PoseStamped]) -> None:
        super().__init__("waypoint_commander")
        self.nav = BasicNavigator()
        self.waypoints = waypoints
        self.idx = 0

        self.current_goal: Optional[PoseStamped] = None
        self.paused = False

        # retry budget per goal (after a FAIL/CANCEL)
        self.retry_used = 0

        # progress / blockage tracking (per-goal)
        self._last_dist: Optional[float] = None
        self._last_progress_ts: float = time.monotonic()
        self._forced_cancel_for_block = False  # mark when we cancel due to 60s block

    # ---------- Bringup ----------
    def wait_for_bt_navigator_active(self) -> None:
        svc_name = "/bt_navigator/get_state"
        cli = self.create_client(GetState, svc_name)

        print("Waiting for Nav2 (bt_navigator) to activate...")
        while rclpy.ok() and not cli.wait_for_service(timeout_sec=1.0):
            print("[wait] bt_navigator/get_state service not available, waiting...")

        req = GetState.Request()
        while rclpy.ok():
            future = cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if not future.done():
                time.sleep(0.5)
                continue
            try:
                state = future.result().current_state
            except Exception:
                time.sleep(0.5)
                continue

            if state.id == State.PRIMARY_STATE_ACTIVE:
                print("Nav2 is active.")
                return
            else:
                print(f"[wait] bt_navigator state = {state.label} ({state.id}); waiting...")
                time.sleep(1.0)

    def set_initial_pose(self, pose: PoseStamped) -> None:
        self.nav.setInitialPose(pose)
        time.sleep(0.5)  # settle

    # ---------- Helpers ----------
    def _reset_progress_tracker(self):
        self._last_dist = None
        self._last_progress_ts = time.monotonic()
        self._forced_cancel_for_block = False

    def _update_progress(self, dist_now: float):
        now = time.monotonic()
        if self._last_dist is None:
            self._last_dist = dist_now
            self._last_progress_ts = now
            return

        # Progress if distance decreased enough
        if (self._last_dist - dist_now) >= PROGRESS_EPS_M:
            self._last_progress_ts = now
            self._last_dist = dist_now
        else:
            # No significant progress; check blockage timeout
            if (now - self._last_progress_ts) > BLOCK_TIMEOUT_SEC:
                print(f"\nBlocked > {BLOCK_TIMEOUT_SEC:.0f}s — aborting current attempt.")
                self.cancel_current(silent=True)
                self._forced_cancel_for_block = True
                # After cancel, BasicNavigator will mark task complete -> result=CANCELED on next tick

    # ---------- Goal control ----------
    def send_goal(self, goal: PoseStamped):
        goal.header.stamp = self.get_clock().now().to_msg()
        self.current_goal = goal
        self._reset_progress_tracker()
        self.nav.goToPose(goal)
        print(f"Going to waypoint {self.idx+1}/{len(self.waypoints)}")

    def cancel_current(self, silent: bool = False):
        if self.current_goal is not None:
            try:
                self.nav.cancelTask()
            except Exception:
                pass
            if not silent:
                print("Canceled current task.")

    def clear_local(self):
        try:
            self.nav.clearLocalCostmap()
        finally:
            print("Local costmap cleared.")

    def clear_both(self):
        try:
            self.nav.clearCostmap()
        except Exception:
            try: self.nav.clearLocalCostmap()
            except Exception: pass
            try: self.nav.clearGlobalCostmap()
            except Exception: pass
        print("Both costmaps cleared.")

    # ---------- Keyboard ----------
    @staticmethod
    def _getch_nonblocking() -> Optional[str]:
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    # ---------- Main loop step ----------
    def step(self) -> bool:
        if self.current_goal and not self.paused:
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    print(f"Reached waypoint {self.idx+1}.")
                    self.idx += 1
                    self.retry_used = 0
                    self.current_goal = None
                    if self.idx < len(self.waypoints):
                        self.send_goal(self.waypoints[self.idx])
                    else:
                        print("Route complete ✅")
                        return False

                elif result in (TaskResult.CANCELED, TaskResult.FAILED):
                    # Distinguish a blockage-timeout cancel from user pause etc.
                    if self._forced_cancel_for_block and self.retry_used < RETRY_PER_GOAL:
                        print("Attempting automatic retry after blockage timeout...")
                        self.clear_local()
                        self.retry_used += 1
                        self.send_goal(self.waypoints[self.idx])
                    else:
                        if self._forced_cancel_for_block:
                            print("Retry budget exhausted for this goal. Skipping.")
                        else:
                            print("Goal failed/canceled. Skipping.")
                        self.idx += 1
                        self.retry_used = 0
                        self.current_goal = None
                        if self.idx < len(self.waypoints):
                            self.send_goal(self.waypoints[self.idx])
                        else:
                            print("Route complete (with skips) ⚠️")
                            return False
            else:
                fb = self.nav.getFeedback()
                if fb and fb.distance_remaining is not None:
                    # Show live progress line
                    print(f"Remaining: {fb.distance_remaining:.2f} m", end="\r")
                    # Update blockage timer (resets every time distance decreases by PROGRESS_EPS_M)
                    self._update_progress(float(fb.distance_remaining))

        # Keyboard controls
        ch = self._getch_nonblocking()
        if ch:
            if ch == "p":  # pause
                if self.current_goal and not self.paused:
                    print("\nPausing current goal...")
                    self.cancel_current(silent=True)
                    self.paused = True
                    print("Paused current goal.")
                else:
                    print("\nNothing to pause.")
            elif ch == "r":  # resume
                if self.paused and self.current_goal:
                    print("\nResuming current goal...")
                    self.paused = False
                    self.send_goal(self.current_goal)
                else:
                    print("\nNothing to resume.")
            elif ch == "c":  # cancel & skip
                print("\nCanceling and skipping to next...")
                self.cancel_current(silent=True)
                self.paused = False
                self.retry_used = 0
                self.current_goal = None
                self.idx += 1
                if self.idx < len(self.waypoints):
                    self.send_goal(self.waypoints[self.idx])
                else:
                    print("No more waypoints. Exiting.")
                    return False
            elif ch == "x":  # clear local
                print("\nClearing local costmap...")
                self.clear_local()
                if self.current_goal and not self.paused:
                    self.send_goal(self.current_goal)
            elif ch == "z":  # soft reset
                print("\nSoft reset: cancel, clear both, resend current goal...")
                if self.current_goal:
                    goal_copy = self.current_goal
                    self.cancel_current(silent=True)
                    self.clear_both()
                    self.paused = False
                    self.retry_used = 0
                    self.send_goal(goal_copy)
                else:
                    self.clear_both()
            elif ch == "q":
                print("\nQuitting...")
                self.cancel_current(silent=True)
                return False

        return True

def main():
    # Put terminal into raw mode
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    rclpy.init()
    node = Commander(
        waypoints=[
            make_pose(5.0,  0.0,   0.0),
            make_pose(15.0, 1.0,   0.0),
            make_pose(15.0,-1.0, 180.0),
            make_pose(5.0,  0.0, 180.0),
        ]
    )

    try:
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        node.wait_for_bt_navigator_active()

        print("\nControls: [p]=pause  [r]=resume  [c]=cancel+skip  [x]=clear local  [z]=soft reset  [q]=quit\n")

        node.set_initial_pose(make_pose(0.0, 0.0, 0.0))
        if node.waypoints:
            node.send_goal(node.waypoints[0])

        keep_running = True
        while rclpy.ok() and keep_running:
            executor.spin_once(timeout_sec=0.1)
            keep_running = node.step()

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
