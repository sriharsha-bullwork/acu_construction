#!/usr/bin/env python3


import math
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import tf_transformations




def q_from_yaw(yaw):
    q = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])




def make_pose(x, y, yaw_deg, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.pose.position.x = float(x)
    p.pose.position.y = float(y)
    p.pose.position.z = 0.0
    p.pose.orientation = q_from_yaw(math.radians(yaw_deg))
    return p




def main():
    rclpy.init()
    nav = BasicNavigator()


    # Wait for Nav2 to be active
    nav.waitUntilNav2Active()


    # Initial pose at origin (map frame). With fixed map->odom, this is adequate for sim.
    init = make_pose(0.0, 0.0, 0.0)
    nav.setInitialPose(init)


    # Define your route here (map frame). Add as many as you need.
    waypoints = [
    make_pose(1.0, 0.0, 0.0),
    make_pose(1.0, 1.0, 90.0),
    make_pose(0.0, 1.0, 180.0),
    make_pose(0.0, 0.0, -90.0),
    ]


    nav.navigateThroughPoses(waypoints)


    while not nav.isTaskComplete():
        fb = nav.getFeedback()
        if fb is not None and fb.distance_remaining is not None:
            print(f"Remaining: {fb.distance_remaining:.2f} m")
        rclpy.spin_once(nav, timeout_sec=0.1)


    result = nav.getResult()
    if result == TaskResult.SUCCEEDED:
        print("Route complete ✅")
    elif result == TaskResult.CANCELED:
        print("Task canceled ⚠️")
    else:
        print("Task failed ❌")


    rclpy.shutdown()




if __name__ == '__main__':
    main()