from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='construction',
            executable='waypoint_runner',
            name='construction_waypoint_runner',
            output='screen',
            parameters=[{
                # Edit in-place or override via CLI
                'speed': 0.15,
                'repeat': False,
                'frame_id': 'map',
                # Default waypoints (x,y,yaw in radians)
                'waypoints': [
                    [1.0, 0.0, 0.0],
                    [1.0, 1.0, 1.5708],
                    [0.0, 1.0, 3.1416]
                ],
            }],
        )
    ])
