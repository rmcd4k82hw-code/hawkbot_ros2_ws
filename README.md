# Hawkbot ROS 2 - Multi-Robot Navigation & Follow-the-Leader

A complete ROS 2 (Humble) navigation and multi-robot coordination system featuring Cartographer 2D pure localization, real-time blue trail recording, Pure Pursuit path following, and a real-time **Follow-the-Leader** mode.

---

## 🌟 Key Features

1. **Follow-the-Leader Mode (`follow_leader.py` & `follow_leader_menu.py`)**:
   - **Real-Time Trail Tracking**: Robot 2 streams and follows Robot 1's blue trail in real-time as Robot 1 drives.
   - **Leader Motion Detection**: Subscribes directly to Robot 1's `cmd_vel` velocity commands for immediate start/stop synchronization (immune to Cartographer TF noise).
   - **Safety Separation Distance**: Enforces a strict 35 cm minimum distance between robot centers (~15-20 cm physical bumper gap), preventing collisions even when Robot 1 rotates on the spot.
   - **Strict Sequential Waypoint Progress**: Uses a forward sliding search window (max 5 waypoints) and waypoint consumption to guarantee Robot 2 never jumps or backtracks at path intersections.

2. **Trajectory Recording & Playback (`trajectory_recorder.py` & `route_follower.py`)**:
   - Live blue trail visual marker publishing (`recorded_trail_marker`) and ROS 2 path streaming (`nav_msgs/Path`).
   - Loop closure smoothing (micro-zigzag filtering & Chaikin curve smoothing) with JSON saving/loading.
   - Pure Pursuit controller with automatic pivot-on-spot heading alignment.

3. **Multi-Robot Localization & Stability (`hawkbot_cartographer` & `hawkbot_navigation2`)**:
   - Cartographer pure localization against reference `.pbstream` submaps (`optimize_every_n_nodes = 0` for 200 Hz smooth continuous Ceres scan matching).
   - `scan_match_monitor`: Real-time scan-to-map match scoring with safety pause triggers.
   - `odom_recalibrator`: 50 Hz smooth odometry drift decay to origin.
   - `cartographer_initial_pose_bridge`: 2D Initial Pose synchronization.

---

## 📁 Package Overview

* `hawkbot`: Core Python nodes, YDLidar launcher, teleoperation, trajectory recorder, and follow-leader controllers.
* `hawkbot_navigation2`: Multi-robot launch files (`multi_navigation_launch.py`) and RViz configurations.
* `hawkbot_cartographer`: Cartographer localization Lua configs (`lidar_robot1_localization.lua`, `lidar_robot2_localization.lua`).
* `hawkbotcar_laser`, `hawkbotcar_ai`, `hawkbotcar_msgs`: Hardware drivers and message definitions.

---

## 🚀 Quick Start Guide

### 1. Build the Workspace
```bash
cd ~/ROS2_WS
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

### 2. Launch Robot Hardware Bringup
```bash
ros2 launch hawkbot multi_bringup_launch.py
```

### 3. Launch Multi-Robot Navigation & RViz
```bash
ros2 launch hawkbot_navigation2 multi_navigation_launch.py
```

### 4. Run Follow-the-Leader Mode (Robot 2 follows Robot 1)
In a new terminal:
```bash
source ~/ROS2_WS/install/setup.bash
ros2 run hawkbot follow_leader_menu
```
Select Option **1** to activate Follow-the-Leader mode.

### 5. Drive Robot 1
In another terminal:
```bash
source ~/ROS2_WS/install/setup.bash
ros2 run hawkbot teleop_key robot1
```
Drive Robot 1 using keyboard controls. Robot 2 will automatically follow Robot 1's live blue trail!

---

## 📋 Control Menus

* **`follow_leader_menu`**:
  - `1`: Activate Follow-the-Leader mode (starts Robot 1 trail recording + Robot 2 live following).
  - `2`: Deactivate Follow-the-Leader mode (stops Robot 2 and saves trail).
  - `3-5`: Manual trail controls.

* **`trajectory_control_menu`**:
  - `1-3`: Record, stop & save, or clear static blue trail.
  - `4-6`: Follow saved static route for Robot 1 or Robot 2.

---

## 📜 License
Internal / Educational Project - Hawkbot Robotics.
