from setuptools import setup

package_name = 'acu_construction'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Package index + manifest
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch files
        ('share/' + package_name + '/launch', [
            'launch/construction_nav.launch.py',
            'launch/nav2_tb3.launch.py',
            'launch/sim_nav2_waypoints.launch.py',
             'launch/sim_nav2_gps.launch.py',
             'launch/sim_nav2_gps_vamana.launch.py',
        ]),

        # Extra params (keep your existing burger.yaml)
        ('share/' + package_name + '/params', [
            'params/burger.yaml',
        ]),

        # Nav2 config + Behavior Trees (NO-REPLAN)
        ('share/' + package_name + '/config', [
            'config/nav2_params.yaml',
            'config/navigate_to_pose_no_replan.xml',
            'config/navigate_through_poses_no_replan.xml',
            'config/robot_localization.yaml',
            'config/tb3_nav2_gps.rviz',
        ]),

        # Vamana-specific Nav2 + RL config
        ('share/' + package_name + '/vamana_sim', [
            'vamana_sim/vamana_nav2_params.yaml',
            'vamana_sim/simple_nav2_params.yaml',
            'vamana_sim/vamana_robot_localization.yaml',
            'vamana_sim/vamana_navigate_to_pose_no_replan.xml',
            'vamana_sim/vamana_navigate_through_poses_no_replan.xml',
        ]),

        # Vamana Basic Nav (new folder)
        ('share/' + package_name + '/vamana_basic_nav/launch', [
            'vamana_basic_nav/launch/vamana_sim_basic_nav.launch.py',
        ]),
        ('share/' + package_name + '/vamana_basic_nav/config', [
            'vamana_basic_nav/config/vamana_basic_nav_nav2_params.yaml',
            'vamana_basic_nav/config/vamana_basic_nav_navigate_to_pose_no_replan.xml',
            'vamana_basic_nav/config/vamana_basic_nav_navigate_through_poses_no_replan.xml',
        ]),

        # Models (GPS-enabled TB3 SDF)
        ('share/' + package_name + '/models/turtlebot3_burger_gps', [
            'models/turtlebot3_burger_gps/model.sdf',
        ]),

        # Static maps used by map_server
        ('share/' + package_name + '/maps', [
            'maps/empty_world.yaml',
            'maps/empty_world.pgm',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sriharsha Sheshanarayana',
    maintainer_email='sriharsha@bullworkmobility.com',
    description='Waypoint logic and Nav2 configuration for TurtleBot3 simulation (no-replan BTs).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Your waypoint runner (if you still have it)
            'waypoint_runner = acu_construction.waypoint_runner:main',
            # The interactive commander we built together
            'waypoint_commander = acu_construction.waypoint_commander:main',
        ],
    },
)
