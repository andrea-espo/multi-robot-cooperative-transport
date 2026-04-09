# Multi-Robot Cooperative Transport for TurtleBot3 (ROS 2 Jazzy)

## Overview
This repository contains the implementation of adaptive formation control and navigation for multi-robot tethered cooperative transport.

The work is built on top of the official TurtleBot3 ROS 2 ecosystem and extends it with custom modifications for multi-robot coordination, obstacle-aware navigation, and cooperative transport using two TurtleBot3 Burger robots coupled by a flexible tether.

The project focuses on coordinated motion in cluttered indoor environments, with particular attention to formation control, navigation integration, and management of physical coupling constraints.

## Repository Scope

This repository is based on the official TurtleBot3 ROS 2 packages and includes custom extensions and modifications developed for the thesis project.

The original TurtleBot3 stack is used as the experimental and software foundation, while the main contributions of this work focus on multi-robot coordination, adaptive formation control, customized navigation behavior, and cooperative transport using tethered robots.

## Main Contributions

The main contributions of this project include:

- Development of a multi-robot system based on two TurtleBot3 platforms  
- Implementation of adaptive formation control strategies  
- Integration of cooperative transport using a flexible tether between robots  
- Custom configuration of the ROS 2 Nav2 stack for multi-robot navigation  
- Obstacle-aware navigation with formation adaptation  
- Implementation of a virtual frame-based coordination strategy  
- Custom launch files and system configuration for multi-robot deployment

## Modified Components

The following TurtleBot3 components have been adapted and extended for this project:

- `turtlebot3_bringup`: customized launch files and robot configuration  
- `turtlebot3_navigation2`: modified navigation parameters and multi-robot setup  
- `turtlebot3_cartographer`: adjusted SLAM configuration for experimental scenarios  

Additional custom configurations include:
- multi-robot namespace management  
- TF remapping and frame coordination  
- navigation parameter tuning  
  
