import yaml
from pyproj import Proj
import math
import os
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped

def yaw_to_quaternion(yaw):
    """Converts a yaw angle (in radians) to a quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    # Returns (x, y, z, w)
    return (0.0, 0.0, sy, cy)

def convert_gps_to_local(latitude, longitude, projector, origin_utm):
    """Converts GPS coordinates to local map coordinates relative to an origin."""
    utm_x, utm_y = projector(longitude, latitude)
    map_x = utm_x - origin_utm[0]
    map_y = utm_y - origin_utm[1]
    return map_x, map_y

def create_goal_pose(navigator, x, y, yaw):
    """Creates a PoseStamped message for a Nav2 goal."""
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = 0.0

    q_x, q_y, q_z, q_w = yaw_to_quaternion(yaw)
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    
    return goal_pose

def main():
    """
    Reads GPS waypoints, converts them, and sends them as goals to Nav2.
    """
    # --- ROS2 Initialization ---
    rclpy.init()
    navigator = BasicNavigator()

    # --- File Loading ---
    try:
        script_dir = os.path.dirname(os.path.realpath(__file__))
    except NameError:
        script_dir = os.getcwd()

    waypoints_file_name = 'demo_wp.yaml' 
    waypoints_path = os.path.join(script_dir, waypoints_file_name)
    
    print(f"Loading waypoints from: {waypoints_path}")

    try:
        with open(waypoints_path, 'r') as file:
            waypoints_data = yaml.safe_load(file)
            if not waypoints_data or 'waypoints' not in waypoints_data:
                print("ERROR: YAML file is empty or missing 'waypoints' key.")
                rclpy.shutdown()
                return
            waypoints = waypoints_data['waypoints']
    except (FileNotFoundError, yaml.YAMLError) as e:
        print(f"ERROR: Could not load or parse waypoints file: {e}")
        rclpy.shutdown()
        return

    # --- Wait for Nav2 to be active ---
    print("Waiting for Nav2 to become active...")
    # Since you are not using AMCL, we tell the function to skip waiting for it.
    navigator.waitUntilNav2Active(amcl_service_name="none")
    print("Nav2 is active. Starting waypoint following.")

    # --- Coordinate Projection Setup ---
    projector = Proj(proj='utm', zone=43, ellps='WGS84', preserve_units=False)
    first_wp = waypoints[0]
    origin_utm = projector(first_wp['longitude'], first_wp['latitude'])
    print(f"Map origin set at UTM (x,y): {origin_utm}")
    print("-" * 30)

    # --- Process and Send Waypoints as Goals ---
    try:
        for i, wp in enumerate(waypoints):
            print(f"Processing waypoint #{i+1}/{len(waypoints)}...")
            
            local_x, local_y = convert_gps_to_local(
                wp['latitude'], 
                wp['longitude'], 
                projector, 
                origin_utm
            )
            
            goal_pose = create_goal_pose(navigator, local_x, local_y, wp['yaw'])
            
            print(f"  - Sending goal: (x={local_x:.2f}, y={local_y:.2f})")
            navigator.goToPose(goal_pose)

            # --- Wait for the goal to complete ---
            while not navigator.isTaskComplete():
                feedback = navigator.getFeedback()
                if feedback:
                    # Optional: Print feedback, e.g., distance remaining
                    pass

            result = navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                print("  - Goal succeeded!")
            elif result == TaskResult.CANCELED:
                print("  - Goal was canceled! Stopping.")
                break
            elif result == TaskResult.FAILED:
                print("  - Goal failed! Stopping.")
                break
            else:
                print("  - Goal has an invalid return status!")
            
            print("-" * 30)

    except KeyboardInterrupt:
        print("\nCaught KeyboardInterrupt. Canceling the current goal...")
        navigator.cancelTask()

    finally:
        print("All waypoints processed or task interrupted. Shutting down.")
        rclpy.shutdown()

if __name__ == '__main__':
    main()

