import sys
import time
import math
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from robot_localization.srv import FromLL
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


DEFAULT_WP_FILE = \
    "/workspaces/Simulation_ACU/src/acu_construction/logged_waypoint/demo_wp.yaml"


def yaw_to_quaternion(yaw_rad: float):
    cz = math.cos(0.5 * yaw_rad)
    sz = math.sin(0.5 * yaw_rad)
    return (0.0, 0.0, sz, cz)


def load_first_waypoint(yaml_path: str):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f) or {}
    if 'waypoints' not in data or not isinstance(data['waypoints'], list) or not data['waypoints']:
        raise ValueError("YAML must contain a non-empty 'waypoints' list")
    wp = data['waypoints'][0]
    for k in ('latitude', 'longitude', 'yaw'):
        if k not in wp:
            raise ValueError(f"Waypoint missing key: {k}")
    return float(wp['latitude']), float(wp['longitude']), float(wp['yaw'])


def ll_to_map_xy(node: Node, latitude: float, longitude: float, altitude: float = 0.0):
    client = node.create_client(FromLL, '/fromLL')
    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('Waiting for /fromLL service...')

    req = FromLL.Request()
    req.ll_point.latitude = latitude
    req.ll_point.longitude = longitude
    req.ll_point.altitude = altitude

    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is None:
        raise RuntimeError(f"/fromLL service call failed: {future.exception()}")
    resp = future.result()
    return resp.map_point.x, resp.map_point.y


def build_goal_pose(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def main(argv=None):
    argv = argv if argv is not None else sys.argv[1:]

    # Pick YAML path (argument or default)
    yaml_path = Path(argv[0]) if argv else Path(DEFAULT_WP_FILE)
    if not yaml_path.exists():
        print(f"ERROR: Waypoint file not found: {yaml_path}")
        return 2

    rclpy.init()

    # Temporary node for LL->map service call
    helper = Node('wp_ll_to_map_helper')
    try:
        lat, lon, yaw = load_first_waypoint(str(yaml_path))
        helper.get_logger().info(
            f"Loaded waypoint: lat={lat}, lon={lon}, yaw={yaw}")

        x, y = ll_to_map_xy(helper, lat, lon, altitude=0.0)
        helper.get_logger().info(f"Converted to map XY: x={x:.3f}, y={y:.3f}")
    except Exception as e:
        helper.get_logger().error(f"Failed preparing goal: {e}")
        helper.destroy_node()
        rclpy.shutdown()
        return 1

    helper.destroy_node()

    # Use Simple Commander to send goal
    navigator = BasicNavigator()
    # If no AMCL/localizer, skip waiting on it by passing a non-existent service
    # navigator.waitUntilNav2Active(amcl_service_name='none')

    goal = build_goal_pose(navigator, x, y, yaw)
    navigator.goToPose(goal)
    print(f"Goal sent to Nav2: x={x:.3f}, y={y:.3f}, yaw={yaw:.3f} rad")

    try:
        while not navigator.isTaskComplete():
            fb = navigator.getFeedback()
            # Minimal monitoring; sleep a bit to avoid busy loop
            time.sleep(0.2)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal reached successfully.')
        elif result == TaskResult.CANCELED:
            print('Goal canceled.')
        elif result == TaskResult.FAILED:
            err_code, err_msg = navigator.getTaskError()
            print(f'Goal failed: {err_code}: {err_msg}')
        else:
            print('Goal returned unknown status.')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
