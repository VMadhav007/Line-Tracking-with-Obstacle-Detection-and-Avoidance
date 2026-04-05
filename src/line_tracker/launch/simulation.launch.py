import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('line_tracker')

    world_file  = os.path.join(pkg_share, 'worlds', 'line_world.world')
    rviz_config = os.path.join(pkg_share, 'config', 'rviz_config.rviz')
    urdf_file   = os.path.join(pkg_share, 'urdf', 'turtlebot3_burger.urdf')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ── 0. Kill any stale Gazebo instance ─────────────────────────────
    kill_gazebo = ExecuteProcess(
        cmd=['bash', '-c', 'pkill -9 -f "gzserver|gzclient" 2>/dev/null; sleep 1; echo "Gazebo cleared"'],
        output='screen',
    )

    # ── 1. Robot State Publisher ───────────────────────────────────────
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    # ── 2. Gazebo ──────────────────────────────────────────────────────
    gazebo = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'gazebo', '--verbose', world_file,
                    '-s', 'libgazebo_ros_init.so',
                    '-s', 'libgazebo_ros_factory.so',
                ],
                output='screen',
            ),
        ],
    )

    # ── 3. Spawn robot ─────────────────────────────────────────────────
    spawn_robot = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', 'turtlebot3_burger',
                    '-file',   urdf_file,
                    '-x', '0.0', '-y', '-1.25', '-z', '0.02',
                    '-Y', '0.0',
                ],
                output='screen',
            ),
        ],
    )

    # ── 4. Line detector ───────────────────────────────────────────────
    line_detector_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='line_tracker',
                executable='line_detector',
                name='line_detector_node',
                output='screen',
                parameters=[{
                    'use_sim_time':      True,
                    'roi_top_fraction':  0.6,
                    'hsv_lower':         [0, 0, 0],
                    'hsv_upper':         [180, 255, 50],
                    'min_contour_area':  500.0,
                }],
            ),
        ],
    )

    # ── 5. Obstacle detector ───────────────────────────────────────────
    obstacle_detector_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='line_tracker',
                executable='obstacle_detector',
                name='obstacle_detector_node',
                output='screen',
                parameters=[{
                    'use_sim_time':                 True,
                    'obstacle_distance_threshold':  0.55,
                    'front_fov_deg':                90,
                }],
            ),
        ],
    )

    # ── 6. Controller ──────────────────────────────────────────────────
    controller_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='line_tracker',
                executable='controller',
                name='robot_controller_node',
                output='screen',
                parameters=[{
                    'use_sim_time':                 True,
                    'Kp':                           0.005,
                    'Ki':                           0.0001,
                    'Kd':                           0.001,
                    'linear_speed':                 0.15,
                    'max_angular':                  2.0,
                    'obstacle_distance_threshold':  0.55,
                    'phase1_turn_duration':         0.78,   # turn away ~90°
                    'phase2_forward_duration':      2.0,    # drive alongside (0.40m)
                    'phase3_turn_duration':         0.78,   # turn back ~90°
                    'phase4_forward_duration':      2.5,    # drive past obstacle (0.50m)
                }],
            ),
        ],
    )

    # ── 7. RViz2 ───────────────────────────────────────────────────────
    rviz2 = TimerAction(
        period=11.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config],
                output='screen',
                parameters=[{'use_sim_time': True}],
            ),
        ],
    )

    return LaunchDescription([
        kill_gazebo,
        robot_state_pub,
        gazebo,
        spawn_robot,
        line_detector_node,
        obstacle_detector_node,
        controller_node,
        rviz2,
    ])