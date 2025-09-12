import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Package share dirs
    pkg_share = get_package_share_directory('acu_construction')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # Nav2 parameters and behavior tree config
    nav2_params = os.path.join(pkg_share, 'config_construction', 'nav2_par.yaml')
    bt_xml_path = PathJoinSubstitution([
        FindPackageShare('acu_construction'),
        'config_construction',
        'vamana_basic_nav_navigate_to_pose_no_replan.xml',
    ])

    configured_params = RewrittenYaml(
        source_file=nav2_params,
        root_key='',
        param_rewrites={'default_nav_to_pose_bt_xml': bt_xml_path},
        convert_types=True,
    )

    # Robot Localization (dual EKF)
    robot_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'dual_ekf_map_launch.py')
        )
    )

    # Nav2 bringup
    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'False',
            'params_file': configured_params,
            'autostart': 'True',
        }.items(),
    )

    # Supporting node for GNSS yaw estimation
    gps_yaw = Node(package='trailblazer_core_py', executable='gnss_yaw_publisher')

    ld = LaunchDescription()
    ld.add_action(robot_localization_cmd)
    ld.add_action(navigation2_cmd)
    ld.add_action(gps_yaw)

    return ld
