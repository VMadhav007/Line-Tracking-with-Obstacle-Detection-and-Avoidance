# Line Tracker — ROS 2 Humble Autonomous Robot

Autonomous line-following robot with LIDAR obstacle avoidance, built for
ROS 2 Humble + Gazebo Classic on Ubuntu 22.04.

## Architecture

```
/camera/image_raw ──► line_detector_node  ──► /line_error    ──┐
/scan             ──► obstacle_detector_node ──► /obstacle_flag ──┼──► robot_controller_node ──► /cmd_vel
```

**FSM states:** `LINE_FOLLOW → STOP → AVOID (Bug0) → RETURN → LINE_FOLLOW`

## Quick Start

```bash
# 1. Source ROS 2 + workspace (do this in every terminal)
source /opt/ros/humble/setup.bash
source /home/pes1ug23cs679/mars_anti2/install/setup.bash

# 2. Launch everything (Gazebo + robot + nodes + RViz2)
ros2 launch line_tracker simulation.launch.py
```

## Rebuild after edits

```bash
cd /home/pes1ug23cs679/mars_anti2
source /opt/ros/humble/setup.bash
colcon build --packages-select line_tracker
source install/setup.bash
```

## Monitor topics (separate terminals)

```bash
ros2 topic echo /line_error       # lateral error in pixels (NaN = line lost)
ros2 topic echo /obstacle_flag    # true when obstacle < 0.5 m ahead
ros2 topic echo /cmd_vel          # velocity commands sent to robot
rqt_graph                         # node/topic graph
ros2 run rqt_plot rqt_plot /line_error/data   # PID tuning plot
```

## PID Tuning

Edit parameters in `launch/simulation.launch.py` under the `controller` node:

| Parameter     | Default | Effect                    |
|---------------|---------|---------------------------|
| `Kp`          | 0.005   | Proportional gain         |
| `Ki`          | 0.0001  | Integral gain (drift fix) |
| `Kd`          | 0.001   | Derivative (damping)      |
| `linear_speed`| 0.15    | Forward speed (m/s)       |

## File Structure

```
src/line_tracker/
├── line_tracker/
│   ├── line_detector.py       # Camera → /line_error
│   ├── obstacle_detector.py   # LIDAR  → /obstacle_flag
│   └── controller.py          # FSM + PID + Bug0 → /cmd_vel
├── launch/simulation.launch.py
├── worlds/line_world.world     # Oval track + 2 obstacles
├── urdf/turtlebot3_burger.urdf # Robot with camera + LIDAR
├── config/rviz_config.rviz
├── package.xml
└── setup.py
```
