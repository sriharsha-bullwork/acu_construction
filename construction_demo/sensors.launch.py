import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Package share dirs (for sensor-related packages)

    # 1. GNSS drivers (rover + base)
    core_pkg = get_package_share_directory('acu_construction')

    # Expose serial arguments to select distinct devices
    rover_serial_arg = DeclareLaunchArgument(
        'rover_serial', default_value='tseT', description='Rover GNSS USB serial string'
    )
    base_serial_arg = DeclareLaunchArgument(
        'base_serial', default_value='Tset', description='Base GNSS USB serial string'
    )


    # Only launch GNSS containers if an explicit serial string is provided
    rover_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'ublox_mb+r_rover.launch.py')
        ),
        # Forward selected serial to child; only launch if provided and distinct from base
        launch_arguments={'device_serial_string': LaunchConfiguration('rover_serial')}.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration('rover_serial'), "' != '' and '",
                LaunchConfiguration('rover_serial'), "' != '",
                LaunchConfiguration('base_serial'), "'"
            ])
        ),
    )
    base_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'ublox_mb+r_base.launch.py')
        ),
        # Forward selected serial to child; launch if provided
        launch_arguments={'device_serial_string': LaunchConfiguration('base_serial')}.items(),
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('base_serial'), "' != ''"]) 
        ),
    )

    # 2. rosbridge server (XML launch)
    rosbridge_pkg = get_package_share_directory('rosbridge_server')
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(rosbridge_pkg, 'launch', 'rosbridge_websocket_launch.xml')
        ),
        # Avoid arg collision with NTRIP ('port') by pinning rosbridge port
        launch_arguments={'port': '9090'}.items(),
    )

    # 3. SL-LiDAR S2 driver
    sllidar_pkg = get_package_share_directory('sllidar_ros2')
    sllidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_pkg, 'launch', 'sllidar_s2_launch.py')
        )
    )

    # 4. ZED camera (zedx)
    zed_pkg = get_package_share_directory('zed_wrapper')
    zedx = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(zed_pkg, 'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={'camera_model': 'zedx', 'enable_ipc': 'false'}.items(),
    )

    # 5. NTRIP client (RTCM corrections)
    ntrip_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'ntrip_client.launch.py')
        )
    )


    # Static TF publishers
    static_tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu',
        arguments=['-0.2', '0.0', '0.00', '0.0', '0.0', '0.0', 'base_link', 'imu_link'],
    )
    static_tf_base_to_imu_gps = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_imu_gps',
        arguments=['0.0', '0.0', '0.00', '0.0', '0.0', '0.0', 'base_link', 'imu_gps_link'],
    )
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0.3', '0.0', '0.30', '0.0', '0.0', '0.0', 'base_link', 'laser'],
    )
    static_tf_base_to_zedx = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_camera',
        arguments=['0.85', '0.0', '0.05', '0.0', '0.0', '0.0', 'base_link', 'zed_camera_link'],
    )
    static_tf_gps_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_gps',
        arguments=['-0.3', '0.0', '0.10', '0.0', '0.0', '0.0', 'base_link', 'gps_link'],
    )
    static_tf_basefoot_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_basefoot_to_base',
        arguments=['0.0', '0.0', '1.06', '0.0', '0.0', '0.0', 'base_footprint', 'base_link'],
    )

    # Peripheral/vehicle nodes
    wheel_encoder_node = Node(
        package='core_cpp',
        executable='rpm_node',
        name='wheel_encoder_node',
        output='screen',
    )
    can_node = Node(
        package='core_cpp',
        executable='can_node',
        name='can_node',
        output='screen',
    )

    # Assemble launch description
    ld = LaunchDescription()
    # Top-level args
    ld.add_action(rover_serial_arg)
    ld.add_action(base_serial_arg)



    # Early stack
    ld.add_action(rover_gnss)
    ld.add_action(base_gnss)
    # ld.add_action(ntrip_client)
    ld.add_action(rosbridge)
    ld.add_action(sllidar)
    ld.add_action(zedx)



    # Static TFs
    ld.add_action(static_tf_base_to_imu)
    ld.add_action(static_tf_gps_to_base)
    ld.add_action(static_tf_base_to_laser)
    ld.add_action(static_tf_base_to_zedx)
    ld.add_action(static_tf_base_to_imu_gps)
    ld.add_action(static_tf_basefoot_to_base)

    # Vehicle IO
    ld.add_action(wheel_encoder_node)
    ld.add_action(can_node)

    return ld
