# Multi-Robot Cooperative Transport for TurtleBot3 (ROS 2 Jazzy)

## Overview

This repository contains the implementation of adaptive formation control and navigation for multi-robot tethered cooperative transport.

The work is built on top of the official TurtleBot3 ROS 2 ecosystem and extends it with custom modifications for multi-robot coordination, obstacle-aware navigation, and cooperative transport using two TurtleBot3 Burger robots coupled by a flexible tether.

The project focuses on coordinated motion in cluttered indoor environments, with particular attention to formation control, navigation integration, and management of physical coupling constraints.

> Based on the official [ROBOTIS TurtleBot3](https://github.com/ROBOTIS-GIT/turtlebot3) ROS 2 packages.

---

## Repository Scope

This repository is based on the official TurtleBot3 ROS 2 packages and includes custom extensions and modifications developed for the thesis project.

The original TurtleBot3 stack is used as the experimental and software foundation, while the main contributions of this work focus on multi-robot coordination, adaptive formation control, customized navigation behavior, and cooperative transport using tethered robots.

---

## Main Contributions

- Development of a multi-robot system based on two TurtleBot3 Burger platforms
- Implementation of adaptive formation control strategies
- Integration of cooperative transport using a flexible tether between robots
- Custom configuration of the ROS 2 Nav2 stack for multi-robot navigation
- Obstacle-aware navigation with formation adaptation
- Implementation of a virtual frame-based coordination strategy
- Custom launch files and system configuration for multi-robot deployment

---

## Modified Files

The following files were modified or added with respect to the official TurtleBot3 repository:

### Modified
| File | Description |
|------|-------------|
| `turtlebot3/CMakeLists.txt` | Updated build configuration |
| `turtlebot3_cartographer/config/turtlebot3_lds_2d.lua` | Custom LiDAR configuration for experimental scenarios |
| `turtlebot3_cartographer/launch/cartographer.launch.py` | Adapted SLAM launch for multi-robot setup |
| `turtlebot3_navigation2/launch/navigation2.launch.py` | Multi-robot navigation launch with namespace support |
| `turtlebot3_navigation2/param/burger.yaml` | Tuned Nav2 parameters for cooperative transport |
| `turtlebot3_navigation2/rviz/tb3_navigation2.rviz` | Custom RViz layout for multi-robot visualization |

### Added
| File | Description |
|------|-------------|
| `turtlebot3_navigation2/param/burger_3.yaml` | Nav2 parameters for the second robot |
| `turtlebot3_navigation2/map/map_free.*` | Map of open environment |
| `turtlebot3_navigation2/map/map_room.*` | Map of room environment |
| `turtlebot3_navigation2/map/mappa_buona.*` | Final experimental map |
| `turtlebot3_navigation2/scripts/central_formation_control.py` | Central formation control node |
| `turtlebot3_navigation2/scripts/central_formation_lateral.py` | Lateral formation control node |
| `turtlebot3_navigation2/scripts/formation_controller_speed.py` | Speed-based formation controller |
| `turtlebot3_navigation2/scripts/formation_follower.py` | Follower robot formation node |
| `turtlebot3_navigation2/scripts/virtual_center.py` | Virtual frame center computation |
| `turtlebot3_navigation2/scripts/tf_aggregator.py` | TF frame aggregation for multi-robot coordination |
| `turtlebot3_navigation2/scripts/robot_pose_reader.py` | Robot pose reading utility |
| `turtlebot3_navigation2/scripts/follow_point_tb3_3.xml` | Behavior tree for follower robot |

---

## Requirements

- ROS 2 Jazzy
- TurtleBot3 packages
- Nav2 stack
- Cartographer ROS

---

## How to Run

### 1. Build the workspace

```bash
cd ~/turtlebot3_ws
colcon build
source install/setup.bash
```

### 2. Launch SLAM (mapping)

```bash
# Terminal 1 — bring up the robot
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2 — start Cartographer
ros2 launch turtlebot3_cartographer cartographer.launch.py
```

### 3. Launch Navigation

```bash
# Terminal 1 — bring up the robot
ros2 launch turtlebot3_bringup robot.launch.py

# Terminal 2 — start Nav2
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=<path_to_map.yaml>
```

### 4. Launch Formation Control

```bash
# Terminal 3 — start formation controller
ros2 run turtlebot3_navigation2 central_formation_control.py
```

---

## Author

**Andrea Esposito**  
Master's Degree in Computer Engineering  
Politecnico di Torino
