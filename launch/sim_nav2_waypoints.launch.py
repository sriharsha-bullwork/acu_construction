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


    # Gazebo empty world + TB3 (re-uses the TurtleBot3 Gazebo launch)
    tb3_gazebo_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    str(Path(get_package_share_directory('turtlebot3_gazebo')) / 'launch' / 'empty_world.launch.py')
    )
    )


    # Fixed map->odom (no localization)
    static_tf = Node(
    package='tf2_ros', executable='static_transform_publisher',
    name='static_map_to_odom',
    arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )


    # Nav2 bringup with our params and custom BT
    nav2_params = str(Path(get_package_share_directory('construction')) / 'config' / 'nav2_params.yaml')
    bt_file = str(Path(get_package_share_directory('construction')) / 'config' / 'bt_strict_through_poses.xml')


    nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
    str(Path(get_package_share_directory('nav2_bringup')) / 'launch' / 'bringup_launch.py')
    ),
    launch_arguments={
    'use_sim_time': use_sim_time,
    'params_file': nav2_params,
    'default_bt_xml_filename': bt_file
    }.items()
    )


    return LaunchDescription([
    use_sim_time_arg,
    set_tb3_model,
    tb3_gazebo_launch,
    static_tf,
    nav2_bringup,
    ])