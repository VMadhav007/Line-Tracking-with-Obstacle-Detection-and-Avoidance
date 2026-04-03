from setuptools import setup
import os
from glob import glob

package_name = 'line_tracker'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # ament resource index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # worlds
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.world')),
        # URDF
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.urdf') + glob('urdf/*.xacro')),
        # config / rviz
        (os.path.join('share', package_name, 'config'),
            glob('config/*.rviz') + glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student',
    maintainer_email='student@university.edu',
    description='Autonomous line tracker with obstacle avoidance (ROS 2 Humble)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_detector   = line_tracker.line_detector:main',
            'obstacle_detector = line_tracker.obstacle_detector:main',
            'controller      = line_tracker.controller:main',
        ],
    },
)
