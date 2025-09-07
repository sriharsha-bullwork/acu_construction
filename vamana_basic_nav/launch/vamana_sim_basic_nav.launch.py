from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # RViz: use Nav2's default layout
    pkg_nav2 = Path(get_package_share_directory('nav2_bringup'))
    rviz_cfg = str(pkg_nav2 / 'rviz' / 'nav2_default_view.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    # Fixed map->odom (no localization moving it)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Paths to params + BT XMLs
    pkg_share = Path(get_package_share_directory('acu_construction'))
    nav2_params = str(pkg_share / 'vamana_basic_nav' / 'config' / 'vamana_basic_nav_nav2_params.yaml')
    bt_to_pose = str(pkg_share / 'vamana_basic_nav' / 'config' / 'vamana_basic_nav_navigate_to_pose_no_replan.xml')
    bt_through = str(pkg_share / 'vamana_basic_nav' / 'config' / 'vamana_basic_nav_navigate_through_poses_no_replan.xml')

    # EKF: publish odom -> base_footprint using /diff_drive_base_controller/odom
    ekf_params_path = str(
        Path('/workspaces/Simulation_ACU/src/acu_construction/vamana_basic_nav/config/vamana_basic_nav_ekf.yaml')
    )
    ekf_odom = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_odom',
        output='screen',
        parameters=[
            ekf_params_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # Nav2 core nodes (NO map_server, NO amcl)
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params]
    )

    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params]
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'default_nav_to_pose_bt_xml': bt_to_pose},
            {'default_nav_through_poses_bt_xml': bt_through},
            nav2_params
        ]
    )

    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )

    # Lifecycle manager for the nodes we actually launched
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'bond_timeout': 0.0},
            {'node_names': [
                'controller_server',
                'planner_server',
                'smoother_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower'
            ]}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        static_tf,
        ekf_odom,          # <-- EKF added here
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz2
    ])
