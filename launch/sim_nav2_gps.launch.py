from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
import os


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')

    # Keep TB3 model env for robot_state_publisher (affects which URDF it loads)
    set_tb3_model = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='burger')

    # Bring up Gazebo Classic server/client with TB3 empty world
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world = os.path.join(pkg_tb3_gazebo, 'worlds', 'empty_world.world')

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world}.items(),
    )
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
    )

    # Run TB3 robot_state_publisher (same as turtlebot3_gazebo does)
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Spawn GPS-enabled TB3 from our custom SDF
    pkg_this = Path(get_package_share_directory('acu_construction'))
    tb3_gps_sdf = str(pkg_this / 'models' / 'turtlebot3_burger_gps' / 'model.sdf')

    spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'turtlebot3_burger_gps',
            '-file', tb3_gps_sdf,
            '-x', '0.0', '-y', '0.0', '-z', '0.01',
        ],
        output='screen',
    )

    # Robot Localization: dual EKF + navsat_transform
    # Use this package's share directory (already resolved above)
    rl_params = str(pkg_this / 'config' / 'robot_localization.yaml')

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
            # Input EKF odom for velocity/heading
            ('odometry/filtered', 'odometry/local'),
            # Output odometry from GPS
            ('odometry', 'odometry/gps'),
            # GPS fix in, filtered GPS out
            ('gps/fix', 'gps/fix'),
            ('gps/filtered', 'gps/filtered'),
            # IMU in
            ('imu', 'imu'),
        ],
    )

    # Publish gps_link TF (URDF from TB3 doesn’t have gps_link)
    static_gps_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_to_gps',
        arguments=['-0.02', '0', '0.20', '0', '0', '0', 'base_link', 'gps_link'],
        output='screen',
    )

    # Nav2 core nodes like your sim_nav2_waypoints
    pkg_share = Path(get_package_share_directory('acu_construction'))
    nav2_params = str(pkg_share / 'config' / 'nav2_params.yaml')
    bt_to_pose = str(pkg_share / 'config' / 'navigate_to_pose_no_replan.xml')
    bt_through = str(pkg_share / 'config' / 'navigate_through_poses_no_replan.xml')

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
        set_tb3_model,
        gzserver,
        gzclient,
        robot_state_publisher,
        spawner,
        # Robot Localization stack
        ekf_local,
        navsat,
        ekf_map,
        static_gps_tf,
        controller_server,
        planner_server,
        behavior_server,
        smoother_server,
        bt_navigator,
        waypoint_follower,
        lifecycle_manager,
        rviz2,
    ])
