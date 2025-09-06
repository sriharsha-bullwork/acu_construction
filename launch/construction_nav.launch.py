from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    # This launch intentionally excludes robot_localization and any TF publishers.
    # It assumes another container/simulator is already publishing the full TF tree
    # (map -> odom -> base_footprint/base_link and sensors) and robot state.

    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    use_static_map_to_odom = LaunchConfiguration('use_static_map_to_odom')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    cmd_vel_topic_arg = DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel')
    # Provide a safe identity map->odom TF so Nav2 can start even if the
    # simulator delays or omits publishing it. Set to false to rely on sim.
    use_static_map_to_odom_arg = DeclareLaunchArgument('use_static_map_to_odom', default_value='false')
    # Always run map_server by default for visualization and compatibility

    pkg_this = Path(get_package_share_directory('acu_construction'))

    # Nav2 params and BTs (no-replan trees)
    # Allow overriding params via CLI
    params_file = LaunchConfiguration('params_file')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=str(pkg_this / 'vamana_sim' / 'simple_nav2_params.yaml'),
        description='Full path to the Nav2 params file to use'
    )
    bt_to_pose = str(pkg_this / 'vamana_sim' / 'vamana_navigate_to_pose_no_replan.xml')
    bt_through = str(pkg_this / 'vamana_sim' / 'vamana_navigate_through_poses_no_replan.xml')

    # Provide an empty map via map_server for visualization if desired
    map_yaml = str(pkg_this / 'maps' / 'empty_world.yaml')
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server', output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml}]
    )

    # Optional identity transform is added later conditionally

    # Core Nav2 servers
    controller_server = Node(
        package='nav2_controller', executable='controller_server', name='controller_server',
        output='screen', parameters=[params_file],
        remappings=[('cmd_vel', cmd_vel_topic)]
    )
    planner_server = Node(
        package='nav2_planner', executable='planner_server', name='planner_server',
        output='screen', parameters=[params_file]
    )
    behavior_server = Node(
        package='nav2_behaviors', executable='behavior_server', name='behavior_server',
        output='screen', parameters=[params_file]
    )
    smoother_server = Node(
        package='nav2_smoother', executable='smoother_server', name='smoother_server',
        output='screen', parameters=[params_file]
    )
    bt_navigator = Node(
        package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
        output='screen', parameters=[{'use_sim_time': use_sim_time},
                                    {'default_nav_to_pose_bt_xml': bt_to_pose},
                                    {'default_nav_through_poses_bt_xml': bt_through},
                                    params_file]
    )
    waypoint_follower = Node(
        package='nav2_waypoint_follower', executable='waypoint_follower', name='waypoint_follower',
        output='screen', parameters=[params_file]
    )

    # Lifecycle management
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_localization',
        output='screen', parameters=[{'use_sim_time': True}, {'autostart': True}, {'bond_timeout': 0.0},
                                     {'node_names': ['map_server']}]
    )
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager', name='lifecycle_manager_navigation',
        output='screen', parameters=[{'use_sim_time': True}, {'autostart': True}, {'bond_timeout': 0.0},
                                     {'node_names': ['controller_server', 'planner_server', 'smoother_server',
                                                     'behavior_server', 'bt_navigator', 'waypoint_follower']}]
    )

    # RViz2 with Nav2 default view
    pkg_nav2 = Path(get_package_share_directory('nav2_bringup'))
    rviz_cfg = str(pkg_nav2 / 'rviz' / 'nav2_default_view.rviz')
    rviz2 = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_cfg]
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(cmd_vel_topic_arg)
    ld.add_action(use_static_map_to_odom_arg)
    ld.add_action(params_file_arg)
    # Conditionally add static map->odom based on argument value
    from launch.conditions import IfCondition
    static_map_to_odom_cond = Node(
        package='tf2_ros', executable='static_transform_publisher', name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'], output='screen',
        condition=IfCondition(use_static_map_to_odom)
    )
    ld.add_action(static_map_to_odom_cond)
    # Start map_server and its lifecycle manager
    ld.add_action(map_server)
    ld.add_action(controller_server)
    ld.add_action(planner_server)
    ld.add_action(behavior_server)
    ld.add_action(smoother_server)
    ld.add_action(bt_navigator)
    ld.add_action(waypoint_follower)
    ld.add_action(lifecycle_manager_localization)
    ld.add_action(lifecycle_manager_navigation)
    ld.add_action(rviz2)
    return ld
