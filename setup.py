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
            'launch/nav2_tb3.launch.py',
            'launch/sim_nav2_waypoints.launch.py',
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
