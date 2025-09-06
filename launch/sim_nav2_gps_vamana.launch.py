from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # This launch assumes the robot and Gazebo are already up via agriculture.launch.py
    pkg_this = Path(get_package_share_directory('acu_construction'))

    # Robot Localization: dual EKF + navsat_transform, Vamana-specific params
    rl_params = str(pkg_this / 'vamana_sim' / 'vamana_robot_localization.yaml')

    ekf_local = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_odom', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, rl_params],
        remappings=[('odometry/filtered', 'odometry/local')],
    )

    ekf_map = Node(
        package='robot_localization', executable='ekf_node', name='ekf_filter_node_map', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, rl_params],
        remappings=[('odometry/filtered', 'odometry/global')],
    )

    navsat = Node(
        package='robot_localization', executable='navsat_transform_node', name='navsat_transform', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, rl_params],
        remappings=[
            ('odometry/filtered', 'odometry/local'),
            ('odometry', 'odometry/gps'),
            ('gps/fix', '/gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            ('imu', '/gps_imu/data'),
        ],
    )

    # Nav2 core nodes using Vamana-specific params and BTs
    nav2_params = str(pkg_this / 'vamana_sim' / 'vamana_nav2_params.yaml')
    bt_to_pose = str(pkg_this / 'vamana_sim' / 'vamana_navigate_to_pose_no_replan.xml')
    bt_through = str(pkg_this / 'vamana_sim' / 'vamana_navigate_through_poses_no_replan.xml')

    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[nav2_params]
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[nav2_params]
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[nav2_params]
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen', parameters=[nav2_params]
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time},
                                    {'default_nav_to_pose_bt_xml': bt_to_pose},
                                    {'default_nav_through_poses_bt_xml': bt_through},
                                    nav2_params]
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[nav2_params]
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen', parameters=[{'use_sim_time': True}, {'autostart': True}, {'bond_timeout': 0.0},
                                     {'node_names': ['controller_server', 'planner_server', 'smoother_server',
                                                     'behavior_server', 'bt_navigator', 'waypoint_follower']}]
    )

    # RViz: use Nav2's default layout from nav2_bringup
    pkg_nav2 = Path(get_package_share_directory('nav2_bringup'))
    rviz_cfg = str(pkg_nav2 / 'rviz' / 'nav2_default_view.rviz')
    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_local,
        navsat,
        ekf_map,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz2,
    ])
