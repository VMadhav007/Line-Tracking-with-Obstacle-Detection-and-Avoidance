import os
from glob import glob
from setuptools import setup

package_name = 'line_follower_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
        # World files
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # Model files — line_track
        (os.path.join('share', package_name, 'models', 'line_track'),
            glob('models/line_track/*')),
        # Model files — obstacle_box
        (os.path.join('share', package_name, 'models', 'obstacle_box'),
            glob('models/obstacle_box/*')),
        # Model files — turtlebot3_waffle_cam
        (os.path.join('share', package_name, 'models', 'turtlebot3_waffle_cam'),
            glob('models/turtlebot3_waffle_cam/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='Line tracking with obstacle detection and avoidance using TurtleBot3 in Gazebo',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_tracker_node = line_follower_robot.line_tracker_node:main',
            'obstacle_detector_node = line_follower_robot.obstacle_detector_node:main',
            'navigation_controller_node = line_follower_robot.navigation_controller_node:main',
        ],
    },
)
