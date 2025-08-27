from setuptools import setup

package_name = 'construction'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/nav2_tb3.launch.py',
                                               'launch/waypoints.launch.py']),
        ('share/' + package_name + '/params', ['params/burger.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sriharsha Sheshanarayana',
    maintainer_email='sriharsha@bullworkmobility.com',
    description='Waypoint logic and Nav2 params for TB3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_runner = construction.waypoint_runner:main',
        ],
    },
)
