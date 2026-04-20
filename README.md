# 🤖 Line Tracking with Obstacle Detection and Avoidance

> **Autonomous line-following robot with real-time LiDAR-based obstacle avoidance using ROS 2 Humble and Gazebo Classic**

An autonomous mobile robot built on the TurtleBot3 Burger platform that follows a black line on a white surface using computer vision and a PID controller, while dynamically detecting and bypassing obstacles using a LiDAR-driven finite state machine (FSM) inspired by the **Bug0 path-planning algorithm**.

---

## 📑 Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Algorithms](#algorithms)
  - [1. Line Detection — HSV Color Thresholding](#1-line-detection--hsv-color-thresholding)
  - [2. Line Following — PID Controller](#2-line-following--pid-controller)
  - [3. Obstacle Detection — LiDAR Front-Cone Scanning](#3-obstacle-detection--lidar-front-cone-scanning)
  - [4. Obstacle Avoidance — Bug0-Inspired FSM with Rectangular Bypass](#4-obstacle-avoidance--bug0-inspired-fsm-with-rectangular-bypass)
- [Finite State Machine (FSM)](#finite-state-machine-fsm)
- [How It All Works Together](#how-it-all-works-together)
- [Project Structure](#project-structure)
- [Prerequisites](#prerequisites)
- [Installation & Build](#installation--build)
- [Running the Simulation](#running-the-simulation)
- [ROS 2 Topics](#ros-2-topics)
- [Parameter Tuning Guide](#parameter-tuning-guide)
- [Troubleshooting](#troubleshooting)
- [Contributors](#contributors)
- [License](#license)

---

## Overview

This project implements a complete autonomous navigation pipeline for a differential-drive robot operating on a closed oval track (≈5 m × 2.5 m) with two static obstacles placed directly on the path. The robot must:

1. **Detect** the black line on the white floor using a forward-facing camera
2. **Follow** the line smoothly using closed-loop PID control
3. **Detect** obstacles in its path using a 360° LiDAR sensor
4. **Avoid** obstacles via a committed rectangular bypass maneuver
5. **Re-acquire** the line after bypassing and resume tracking

The entire system runs as three decoupled ROS 2 nodes communicating via publish-subscribe topics, enabling modular development and independent tuning.

### Simulation Environment

- **Platform**: TurtleBot3 Burger (differential drive)
- **Sensors**: Forward-facing RGB camera + 360° LDS-01 LiDAR
- **Track**: Closed oval circuit (black line on white floor, 6 cm wide)
- **Obstacles**: Two 20 cm × 20 cm × 30 cm boxes placed on the straight sections
- **Simulator**: Gazebo Classic 11 with `gazebo_ros` bridge
- **Visualizer**: RViz2 for real-time monitoring

---

## System Architecture

```
┌─────────────────┐       /line_error (Float32)       ┌──────────────────────────┐
│  Camera Sensor  │──►  line_detector_node  ──────────►│                          │
│ /camera/image_raw│                                   │                          │
└─────────────────┘                                    │   robot_controller_node  │──► /cmd_vel
                                                       │                          │    (Twist)
┌─────────────────┐       /obstacle/detected (Bool)    │   ┌──────────────────┐   │
│  LiDAR Sensor   │──►  obstacle_detector_node ───────►│   │  FSM + PID +     │   │
│     /scan       │       /obstacle/min_distance       │   │  Bug0 Bypass     │   │
└─────────────────┘       /obstacle/left_min   ───────►│   └──────────────────┘   │
                          /obstacle/right_min  ───────►│                          │
                                                       └──────────────────────────┘
```

### Node Descriptions

| Node | File | Input Topics | Output Topics | Purpose |
|------|------|-------------|---------------|---------|
| `line_detector_node` | `line_detector.py` | `/camera/image_raw` | `/line_error` | Vision-based line detection |
| `obstacle_detector_node` | `obstacle_detector.py` | `/scan` | `/obstacle/*` | LiDAR-based obstacle detection |
| `robot_controller_node` | `controller.py` | `/line_error`, `/obstacle/*` | `/cmd_vel` | Decision-making and motor control |

---

## Algorithms

### 1. Line Detection — HSV Color Thresholding

**File**: `src/line_tracker/line_tracker/line_detector.py`

The line detection pipeline uses classical computer vision techniques to identify a dark/black line on a white floor from the robot's camera feed.

#### Algorithm Pipeline

```
Raw Camera Frame (BGR)
        │
        ▼
┌─── ROI Extraction ───┐    Only the bottom 40% of the frame is used
│   (bottom 40%)       │    to focus on the immediate path ahead and
└──────────┬───────────┘    reduce false positives from distant objects
           │
           ▼
┌─── BGR → HSV ────────┐    Convert to HSV color space for robust
│   Conversion         │    color-based segmentation independent
└──────────┬───────────┘    of lighting conditions
           │
           ▼
┌─── HSV Thresholding ─┐    H: [0, 180]  — any hue
│   cv2.inRange()      │    S: [0, 255]  — any saturation
└──────────┬───────────┘    V: [0, 50]   — very dark pixels only
           │
           ▼
┌─── Morphological Ops ┐    MORPH_CLOSE: fill small gaps in the line
│   Close → Open       │    MORPH_OPEN:  remove salt-and-pepper noise
└──────────┬───────────┘    Kernel: 5×5 rectangular structuring element
           │
           ▼
┌─── Contour Detection ┐    cv2.findContours() with RETR_EXTERNAL
│   + Area Filtering   │    Filter: contourArea ≥ 500 pixels (noise reject)
└──────────┬───────────┘    Select: largest valid contour
           │
           ▼
┌─── Centroid via      ┐    cx = M10 / M00  (image moments)
│   Image Moments      │    error = cx - (image_width / 2)
└──────────┬───────────┘    Positive error → line is RIGHT of center
           │
           ▼
    Publish /line_error
    (Float32 or NaN if lost)
```

#### Key Design Decisions

- **HSV over RGB**: HSV color space separates brightness (V channel) from color information, making black-line detection robust under varying simulation lighting
- **ROI Cropping**: Using only the bottom 40% of the frame (`roi_top_fraction = 0.6`) focuses detection on the near-field path, reducing latency and false positives
- **Morphological Cleanup**: A `CLOSE → OPEN` sequence first bridges small line gaps, then removes isolated noise pixels
- **NaN Signaling**: When no valid contour is found, `NaN` is published to signal the controller that the line is lost, triggering a slow search behavior

#### Tunable Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `roi_top_fraction` | `0.6` | Fraction of image height to skip from top |
| `hsv_lower` | `[0, 0, 0]` | Lower bound of HSV threshold |
| `hsv_upper` | `[180, 255, 50]` | Upper bound of HSV threshold |
| `min_contour_area` | `500.0` | Minimum contour area to be considered a valid line |

---

### 2. Line Following — PID Controller

**File**: `src/line_tracker/line_tracker/controller.py` (method: `pid()`)

A discrete PID (Proportional-Integral-Derivative) controller converts the lateral pixel error from the line detector into angular velocity commands, steering the robot to keep the line centered in its field of view.

#### PID Control Law

```
                  ┌─────────────────────────────────────────┐
                  │                                         │
  error(t) ──────┤  output = Kp·e(t) + Ki·∫e(t)dt + Kd·de/dt  │──── angular_velocity
                  │                                         │
                  └─────────────────────────────────────────┘
```

**Discrete Implementation:**

```python
dt = (now - prev_time)                          # Time step (seconds)
integral += error * dt                           # Trapezoidal integration
integral = clamp(integral, -500, +500)           # Anti-windup
derivative = (error - prev_error) / dt           # Backward difference
output = Kp * error + Ki * integral + Kd * derivative
output = clamp(output, -max_angular, +max_angular)  # Saturation limit
```

#### How Each Term Works

| Term | Gain | Default | Role |
|------|------|---------|------|
| **Proportional (P)** | `Kp` | `0.005` | Steers proportionally to the current error. Larger error = sharper turn. The primary correction force. |
| **Integral (I)** | `Ki` | `0.0001` | Accumulates past errors over time. Corrects steady-state drift when the robot consistently leans to one side. Anti-windup clamp at ±500 prevents runaway. |
| **Derivative (D)** | `Kd` | `0.001` | Reacts to the rate of change of error. Provides damping to prevent oscillation and overshoot around the line center. |

#### Motion Commands

- **Linear velocity**: Constant at `0.15 m/s` during normal line following
- **Angular velocity**: `-pid(error)` (negated because positive error = line to the right = need to turn right = negative angular.z in ROS convention)
- **Line lost behavior**: If `NaN` received, robot creeps forward at `0.04 m/s` while rotating at `0.25 rad/s` to search

---

### 3. Obstacle Detection — LiDAR Front-Cone Scanning

**File**: `src/line_tracker/line_tracker/obstacle_detector.py`

The obstacle detection node processes 360° LiDAR scans to detect obstacles in the robot's forward path and provide directional clearance information for turn-direction decisions.

#### LiDAR Coordinate Convention

```
                        index 180 (FRONT)
                             ▲
                             │
            index 225        │        index 135
              ╲              │              ╱
                ╲            │            ╱
                  ╲    FRONT CONE     ╱      ← 90° FOV
                    ╲   (±45°)    ╱
         index 270 ──── ROBOT ──── index 90
         (LEFT)        │     │        (RIGHT)
                       │     │
                       ▼     ▼
                    index 0 (BEHIND)
                    (angle_min = -π)
```

> **Important**: The LDS-01 LiDAR on TurtleBot3 starts at `angle_min = -π` (directly behind). Index 0 = BEHIND, Index 180 = FRONT.

#### Detection Algorithm

1. **Front Cone Extraction**: From the 360-element range array, extract indices `[180 - 45, 180 + 45]` (a 90° forward cone)
2. **Range Filtering**: Discard invalid readings: `NaN`, `Inf`, values below `0.005 m` (too close / sensor noise), and values above `range_max`
3. **Minimum Distance**: Compute `d_min_front = min(front_left_min, front_right_min)`
4. **Threshold Test**: If `d_min_front < 0.55 m` → obstacle detected
5. **Side Sector Analysis**: Compute minimum distances in left sector `[225°, 315°]` and right sector `[45°, 135°]` for turn-direction decisions

#### Published Data

| Topic | Type | Description |
|-------|------|-------------|
| `/obstacle/detected` | `Bool` | `True` if any object within 0.55 m in front cone |
| `/obstacle/min_distance` | `Float32` | Closest object distance in front cone (9.99 if clear) |
| `/obstacle/left_min` | `Float32` | Closest object in left sector (for turning decision) |
| `/obstacle/right_min` | `Float32` | Closest object in right sector (for turning decision) |

---

### 4. Obstacle Avoidance — Bug0-Inspired FSM with Rectangular Bypass

**File**: `src/line_tracker/line_tracker/controller.py`

The avoidance algorithm is a **Bug0-inspired** strategy implemented as a finite state machine. Unlike classical Bug0 (which follows the obstacle boundary until the goal line is reached), this implementation uses a **committed 4-phase rectangular bypass maneuver** — a fixed-geometry detour that guarantees the robot clears the obstacle without re-triggering.

#### Why Bug0?

The Bug0 algorithm is a foundational motion-planning strategy for mobile robots:

- **Bug0 Principle**: Move toward the goal. When an obstacle is hit, follow its boundary until the goal direction is clear, then resume.
- **Our Adaptation**: Instead of boundary-following (which can fail on small, isolated obstacles), we execute a fixed rectangular detour sized to clear the 20×20 cm obstacles with margin.

#### The 4-Phase Rectangular Bypass

```
     (Start: Robot facing obstacle)
            │
            ▼
    ┌───────────────┐
    │  Phase 1:     │   Turn 90° AWAY from obstacle
    │  TURN AWAY    │   Duration: 0.78s at 2.0 rad/s
    │  (in-place)   │   Direction: toward side with more clearance
    └───────┬───────┘
            │
            ▼
    ┌───────────────┐
    │  Phase 2:     │   Drive straight (perpendicular to line)
    │  DRIVE ASIDE  │   Duration: 2.0s at 0.20 m/s ≈ 0.40 m lateral
    │  (forward)    │   Clears the obstacle side
    └───────┬───────┘
            │
            ▼
    ┌───────────────┐
    │  Phase 3:     │   Turn 90° BACK toward original heading
    │  TURN BACK    │   Duration: 0.78s at 2.0 rad/s
    │  (in-place)   │   Now facing parallel to the line again
    └───────┬───────┘
            │
            ▼
    ┌───────────────┐
    │  Phase 4:     │   Drive forward past the obstacle
    │  DRIVE PAST   │   Duration: 2.5s at 0.20 m/s ≈ 0.50 m forward
    │  (forward)    │   Early exit if: line visible AND obstacle clear
    └───────┬───────┘
            │
            ▼
     Search for line or resume
```

**Spatial geometry of the bypass:**

```
                    Obstacle
                    ┌─────┐
                    │     │
    ─ ─ ─ ─ ─ ─ ─ ─│─ ─ ─│─ ─ ─ ─ ─ ─ ─ ─    ← Black line
                    │     │
                    └─────┘
                      ▲
    Robot ────► P1 turn ──► P2 (0.4m) ──► P3 turn ──► P4 (0.5m) ──►
               90°                        -90°
```

#### Turn Direction Selection

When an obstacle is first detected, the robot compares LiDAR side-sector distances:

```python
if left_min >= right_min:
    turn LEFT (more clearance on the left)
else:
    turn RIGHT (more clearance on the right)
```

This ensures the robot always bypasses toward the side with more open space.

#### Anti-Loop Mechanisms

A critical challenge is preventing the robot from re-triggering obstacle avoidance immediately after completing a bypass (since the robot may re-acquire the line while still adjacent to the obstacle). Three mechanisms address this:

1. **Phase 4 Early Exit Guard**: Phase 4 only exits early if the line is visible **AND** the obstacle is no longer detected (`obs_min_dist > threshold`). If the line is visible but the obstacle is still in range, the bypass continues.

2. **Search State Guard**: After the bypass, the `SEARCH_LINE` state only transitions back to `FOLLOW_LINE` if `obstacle_detected == False`. If the line is visible but an obstacle remains ahead, the robot drives forward to get past it first.

3. **Post-Avoidance Cooldown**: After any successful avoidance → `FOLLOW_LINE` transition, obstacle detection is **suppressed for 3 seconds**. This gives the robot time to drive away from the obstacle before re-enabling detection. Emergency override remains active at `< 0.20 m`.

---

## Finite State Machine (FSM)

```
           ┌──────────────────────────────────────────────────────────┐
           │                                                          │
           ▼                                                          │
    ╔═══════════════╗    obstacle detected     ╔════════════════════╗  │
    ║  FOLLOW_LINE  ║ ─────────────────────►   ║ OBSTACLE_DETECTED ║  │
    ║  (PID track)  ║    (not in cooldown)     ║   (full stop)     ║  │
    ╚═══════════════╝                          ╚════════╤═══════════╝  │
           ▲                                            │              │
           │                                   pick turn direction     │
           │                                            │              │
           │                                            ▼              │
           │                                   ╔════════════════════╗  │
           │   line found + obstacle clear     ║  AVOID_OBSTACLE   ║  │
           │◄──────────────────────────────────║  (4-phase bypass) ║  │
           │        (+ start cooldown)         ╚════════╤═══════════╝  │
           │                                            │              │
           │                                   phases complete         │
           │                                            │              │
           │                                            ▼              │
           │                                   ╔════════════════════╗  │
           │   line found + path clear         ║   SEARCH_LINE    ║  │
           └───────────────────────────────────║  (spin + creep)  ║──┘
                    (+ start cooldown)         ╚════════════════════╝
                                                        │
                                              emergency collision
                                                        │
                                                        ▼
                                               (back to OBSTACLE_DETECTED)
```

### State Descriptions

| State | Behavior | Transition Out |
|-------|----------|----------------|
| **FOLLOW_LINE** | PID tracks the line at 0.15 m/s. If line is lost (NaN), creeps forward + rotates slowly to search. | → `OBSTACLE_DETECTED` when obstacle < 0.55 m in front cone (respects cooldown) |
| **OBSTACLE_DETECTED** | Full stop. Reads left/right LiDAR sectors to choose bypass direction. | → `AVOID_OBSTACLE` immediately (1 control cycle) |
| **AVOID_OBSTACLE** | Executes committed 4-phase rectangular bypass. Emergency collision check (< 0.12 m) in all phases → re-evaluate. | → `FOLLOW_LINE` (Phase 4 early exit if line + clear) or → `SEARCH_LINE` (phases complete) |
| **SEARCH_LINE** | Spins opposite to avoidance turn at 0.80 rad/s + creeps forward at 0.10 m/s. After 6 s timeout, increases forward speed. | → `FOLLOW_LINE` when line visible + no obstacle ahead. → `OBSTACLE_DETECTED` if emergency collision. |

---

## How It All Works Together

### Complete Operational Flow

1. **Startup**: The launch file starts Gazebo, spawns the TurtleBot3 at `(0, -1.25)` on the oval track, and launches all three ROS 2 nodes with a staggered delay to ensure Gazebo is ready.

2. **Normal Operation** — The robot operates in a continuous sense-plan-act loop at **20 Hz**:

   ```
   Camera → Line Detector → /line_error → Controller → PID → /cmd_vel → Robot
   LiDAR  → Obstacle Det. → /obstacle/* → Controller ──┘
   ```

3. **Line Following**: The camera captures frames at the simulation rate. The line detector extracts the bottom 40% of each frame, converts to HSV, thresholds for dark pixels, finds the largest contour, computes the centroid, and publishes the lateral pixel error. The controller feeds this error into the PID controller which outputs an angular velocity proportional to the error, steering the robot back to center.

4. **Obstacle Encounter**: When the LiDAR front-cone scan detects an object within 0.55 m, the obstacle detector publishes `detected = True`. The controller immediately transitions to `OBSTACLE_DETECTED`, stops the robot, reads left/right clearance, and begins the 4-phase rectangular bypass.

5. **Bypass Execution**: The robot executes four timed phases — turn 90° away, drive 0.40 m perpendicular to the line, turn 90° back, drive 0.50 m past the obstacle. During Phase 4, it optionally exits early if it detects the line and confirms the obstacle is clear.

6. **Line Re-acquisition**: If the line isn't found by the end of Phase 4, the robot enters `SEARCH_LINE`, spinning back toward the line while creeping forward. Once the line is detected and no obstacle blocks the path, it transitions back to `FOLLOW_LINE` with a 3-second cooldown to prevent re-triggering.

7. **Cycle Repeats**: The robot continues around the oval track, autonomously avoiding both obstacles on each lap.

### Timing & Control Rates

| Component | Rate | Rationale |
|-----------|------|-----------|
| Controller loop | 20 Hz (50 ms) | Fast enough for smooth PID response at 0.15 m/s |
| Line detector | Camera publish rate | Processes every frame (typically 30 Hz in Gazebo) |
| Obstacle detector | LiDAR publish rate | Processes every scan (5-10 Hz for LDS-01) |
| Bypass phase timings | Wall-clock seconds | Consistent behavior independent of sim speed |

---

## Project Structure

```
Line-Tracking-with-Obstacle-Detection-and-Avoidance/
├── README.md                          ◄── You are here
├── Finally_done.mp4                   ◄── Demo video of successful run
├── run.txt                            ◄── Quick-start command reference
├── .gitignore
└── src/
    └── line_tracker/                  ◄── ROS 2 Python package
        ├── package.xml                    Package metadata + dependencies
        ├── setup.py                       Entry points + data files
        ├── setup.cfg                      ament configuration
        ├── line_tracker/                  ◄── Python source modules
        │   ├── __init__.py
        │   ├── line_detector.py           Camera → HSV → contour → /line_error
        │   ├── obstacle_detector.py       LiDAR → front cone → /obstacle/*
        │   └── controller.py              FSM + PID + Bug0 bypass → /cmd_vel
        ├── launch/
        │   └── simulation.launch.py       Full simulation launch (Gazebo + nodes + RViz2)
        ├── worlds/
        │   └── line_world.world           Oval track + 2 obstacles (SDF)
        ├── urdf/
        │   └── turtlebot3_burger.urdf     Robot model (camera + LiDAR)
        ├── config/
        │   └── rviz_config.rviz           RViz2 display configuration
        └── resource/
            └── line_tracker               ament resource index marker
```

---

## Prerequisites

| Dependency | Version | Notes |
|-----------|---------|-------|
| **Ubuntu** | 22.04 LTS | Tested on this version |
| **ROS 2** | Humble Hawksbill | Full desktop install recommended |
| **Gazebo** | Classic 11 | Included with `ros-humble-gazebo-ros-pkgs` |
| **Python** | 3.10+ | Ships with Ubuntu 22.04 |
| **OpenCV** | 4.x | `python3-opencv` or via `cv_bridge` |
| **cv_bridge** | ROS 2 package | `ros-humble-cv-bridge` |

### Install Dependencies

```bash
# ROS 2 Humble (if not already installed)
sudo apt update && sudo apt install -y \
    ros-humble-desktop \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-cv-bridge \
    ros-humble-robot-state-publisher \
    python3-colcon-common-extensions
```

---

## Installation & Build

```bash
# 1. Clone the repository (if not already done)
cd ~/Mars_mini_vids
git clone <repository-url> Line-Tracking-with-Obstacle-Detection-and-Avoidance

# 2. Navigate to the project root
cd Line-Tracking-with-Obstacle-Detection-and-Avoidance

# 3. Source ROS 2 environment
source /opt/ros/humble/setup.bash

# 4. Build the package
colcon build --packages-select line_tracker

# 5. Source the workspace overlay
source install/setup.bash
```

---

## Running the Simulation

### One-Command Launch

```bash
source /opt/ros/humble/setup.bash && \
colcon build --packages-select line_tracker && \
source install/setup.bash && \
ros2 launch line_tracker simulation.launch.py
```

This single command will:
1. Kill any stale Gazebo instances
2. Start the Robot State Publisher
3. Launch Gazebo with the oval track world
4. Spawn the TurtleBot3 at the starting position
5. Start the line detector, obstacle detector, and controller nodes
6. Open RViz2 for visualization

### Monitoring (in separate terminals)

```bash
# Watch the lateral line error (NaN = line lost)
ros2 topic echo /line_error

# Watch obstacle detection events
ros2 topic echo /obstacle/detected

# Watch velocity commands sent to the robot
ros2 topic echo /cmd_vel

# Visualize the ROS 2 computation graph
rqt_graph

# Plot line error over time (useful for PID tuning)
ros2 run rqt_plot rqt_plot /line_error/data
```

---

## ROS 2 Topics

| Topic | Type | Publisher | Subscriber | Description |
|-------|------|----------|------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo (camera plugin) | `line_detector_node` | Raw RGB camera frames |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo (LiDAR plugin) | `obstacle_detector_node` | 360° LiDAR range data |
| `/line_error` | `std_msgs/Float32` | `line_detector_node` | `robot_controller_node` | Lateral pixel error (NaN if lost) |
| `/obstacle/detected` | `std_msgs/Bool` | `obstacle_detector_node` | `robot_controller_node` | Obstacle presence flag |
| `/obstacle/min_distance` | `std_msgs/Float32` | `obstacle_detector_node` | `robot_controller_node` | Closest front-cone distance |
| `/obstacle/left_min` | `std_msgs/Float32` | `obstacle_detector_node` | `robot_controller_node` | Closest left-sector distance |
| `/obstacle/right_min` | `std_msgs/Float32` | `obstacle_detector_node` | `robot_controller_node` | Closest right-sector distance |
| `/cmd_vel` | `geometry_msgs/Twist` | `robot_controller_node` | Gazebo (diff_drive plugin) | Velocity commands |

---

## Parameter Tuning Guide

### PID Gains (in `launch/simulation.launch.py`)

| Parameter | Default | Increase Effect | Decrease Effect |
|-----------|---------|-----------------|-----------------|
| `Kp` | `0.005` | More aggressive correction; may oscillate | Sluggish response; cuts corners |
| `Ki` | `0.0001` | Eliminates steady-state offset faster; can cause windup | Allows persistent drift |
| `Kd` | `0.001` | More damping; smoother but slower response | Faster response; may oscillate |
| `linear_speed` | `0.15` | Faster laps; requires tighter PID tuning | Safer; easier to tune |
| `max_angular` | `2.0` | Can handle sharper turns | Limits correction authority |

### Obstacle Avoidance (in `launch/simulation.launch.py`)

| Parameter | Default | Description |
|-----------|---------|-------------|
| `obstacle_distance_threshold` | `0.55 m` | Distance at which obstacle is flagged |
| `front_fov_deg` | `90°` | Width of the forward detection cone |
| `phase1_turn_duration` | `0.78 s` | Time to turn 90° away (at 2.0 rad/s) |
| `phase2_forward_duration` | `2.0 s` | Time to drive perpendicular to line |
| `phase3_turn_duration` | `0.78 s` | Time to turn 90° back |
| `phase4_forward_duration` | `2.5 s` | Time to drive past the obstacle |

---

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| Robot doesn't move | Nodes started before Gazebo is ready | Wait for Gazebo GUI to fully load; relaunch |
| Robot oscillates on line | PID gains too aggressive | Reduce `Kp`, increase `Kd` |
| Robot loses line on curves | ROI too small or speed too high | Decrease `roi_top_fraction` or `linear_speed` |
| Robot collides with obstacle | Detection threshold too small | Increase `obstacle_distance_threshold` |
| Infinite avoidance loop | Re-trigger after bypass | Check cooldown (`avoidance_cooldown` default: 3s) |
| "Line lost" after avoidance | Search didn't find line | Increase `phase4_forward_duration` or `search_timeout` |
| Gazebo crashes on launch | Stale process | `pkill -9 gzserver gzclient` then relaunch |

---

## Contributors

| Name | Email | Role |
|------|-------|------|
| **MARS CIoT Lab** | mars.ciot@pes.edu | Admin / Contributor |

---

## License

This project is licensed under the **MIT License**. See [LICENSE](LICENSE) for details.

---

<p align="center">
  <b>Built with ROS 2 Humble · Gazebo Classic · OpenCV · Python 3</b><br>
  <i>PES University — MARS CIoT Lab</i>
</p>
