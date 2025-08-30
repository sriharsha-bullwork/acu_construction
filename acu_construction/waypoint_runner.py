#!/usr/bin/env python3
import math, rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def make_pose(nav, x, y, yaw=0.0, frame='map'):
    qz, qw = math.sin(yaw/2.0), math.cos(yaw/2.0)
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = nav.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = qz, qw
    return p

def main():
    rclpy.init()
    try:
        node = Node("construction_waypoint_runner")
        # Parameters (can be overridden in launch)
        waypoints = node.declare_parameter(
            "waypoints",
            [[1.0, 0.0, 0.0], [1.0, 1.0, 1.5708], [0.0, 1.0, 3.1416]]
        ).value
        frame_id = node.declare_parameter("frame_id", "map").value
        speed    = float(node.declare_parameter("speed", 0.15).value)
        repeat   = bool(node.declare_parameter("repeat", False).value)

        nav = BasicNavigator()
        nav.waitUntilNav2Active()
        if speed > 0:
            nav.setSpeed(speed)

        def run_once():
            poses = [make_pose(nav, float(x), float(y), float(yaw), frame_id) for x, y, yaw in waypoints]
            nav.goThroughPoses(poses)
            while not nav.isTaskComplete():
                fb = nav.getFeedback()
                if fb and hasattr(fb, "number_of_poses_remaining"):
                    node.get_logger().info(f"Remaining: {fb.number_of_poses_remaining}")
            return nav.getResult()

        result = run_once()
        node.get_logger().info(f"Result: {result}")
        while repeat and result == TaskResult.SUCCEEDED:
            result = run_once()
            node.get_logger().info(f"Loop result: {result}")

    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
