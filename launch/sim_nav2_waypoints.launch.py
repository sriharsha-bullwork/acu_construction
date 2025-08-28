from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Ensure TurtleBot3 model is Burger
    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

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

    # Gazebo: empty world + TB3 (provided by turtlebot3_gazebo)
    tb3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory('turtlebot3_gazebo')) / 'launch' / 'empty_world.launch.py')
        )
    )

    # Fixed map->odom (no localization)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    # Paths to params
    pkg_share = Path(get_package_share_directory('construction'))
    nav2_params = str(pkg_share / 'config' / 'nav2_params.yaml')

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
        # IMPORTANT: do NOT override the BT file here; we set both defaults in YAML below
        parameters=[{'use_sim_time': use_sim_time}, nav2_params]
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
        set_tb3_model,
        tb3_gazebo_launch,
        static_tf,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz2
    ])
