#!/usr/bin/env python3
"""
Launch file for Line Follower Robot simulation.

Launches:
  1. Gazebo with custom line_track_world
  2. Robot state publisher (TurtleBot3 Waffle URDF)
  3. Spawns the modified waffle model (camera tilted down) via SDF
  4. line_tracker_node
  5. obstacle_detector_node
  6. navigation_controller_node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # Package directories
    pkg_line_follower = get_package_share_directory('line_follower_robot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    # Paths
    world_file = os.path.join(pkg_line_follower, 'worlds', 'line_track_world.world')
    params_file = os.path.join(pkg_line_follower, 'config', 'params.yaml')
    models_dir = os.path.join(pkg_line_follower, 'models')
    sdf_file = os.path.join(models_dir, 'turtlebot3_waffle_cam', 'model.sdf')

    # URDF for robot_state_publisher (from turtlebot3_gazebo)
    urdf_file = os.path.join(pkg_tb3_gazebo, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Build GAZEBO_MODEL_PATH to include our custom models + turtlebot3 models
    tb3_models_dir = os.path.join(pkg_tb3_gazebo, 'models')
    gazebo_model_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    new_model_path = models_dir + ':' + tb3_models_dir
    if gazebo_model_path:
        new_model_path = new_model_path + ':' + gazebo_model_path

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([

        # ── Environment Variables ────────────────────────────────────────
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path),
        SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle'),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock',
        ),

        # ── 1. Gazebo Server (with custom world) ────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file}.items(),
        ),

        # ── 2. Gazebo Client (GUI) ──────────────────────────────────────
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

        # ── 3. Robot State Publisher ────────────────────────────────────
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
        ),

        # ── 4. Spawn the Robot (custom SDF with tilted camera) ─────────
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_entity',
            output='screen',
            arguments=[
                '-entity', 'turtlebot3_waffle',
                '-file', sdf_file,
                '-x', '2.0',
                '-y', '0.0',
                '-z', '0.01',
                '-Y', '3.14159',  # facing left along the infinity track
            ],
        ),

        # ── 5. Line Tracker Node ───────────────────────────────────────
        Node(
            package='line_follower_robot',
            executable='line_tracker_node',
            name='line_tracker_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # ── 6. Obstacle Detector Node ──────────────────────────────────
        Node(
            package='line_follower_robot',
            executable='obstacle_detector_node',
            name='obstacle_detector_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),

        # ── 7. Navigation Controller Node ──────────────────────────────
        Node(
            package='line_follower_robot',
            executable='navigation_controller_node',
            name='navigation_controller_node',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
        ),
    ])
