# Nav2 Custom Planners
This Repository evaluates and compares Global and Local Planners in ROS2

## Overview
This project involves the creation and testing of custom ROS2 components for evaluating the performance of various global and local planners in the ROS2 Navigation2 stack. It provides a simulation setup in RViz to test and visualize the navigation paths for robots with different footprints.

## Key Features:

Supports global planners like NavFn, ThetaStar, and SmacPlanner2D.
Supports local planners like DWBLocalPlanner and TEBLocalPlanner.
Configurable for robots with different tail angles (0° and 45°).
Includes predefined scenarios for testing and evaluation.

## Project Structure
in the documentation file

## Installation
  1. Clone the repository:
bash
git clone https://github.com/BojanAndonovski71/navigation2-custom-planners.git
cd navigation2-custom-planners
  2. Build the workspace:
bash
colcon build
source install/setup.bash
  3. Install dependencies:
  Ensure ROS2 Humble and Nav2 are installed.
  Install any missing dependencies with:
bash
rosdep install --from-paths src --ignore-src -r -y

Usage
Running the Navigation Stack
  1. Start the Navigation2 stack:
    In the first terminal:
bash
source install/setup.bash
ros2 launch lp_nav navigation_launch2.py
  2. Load a custom planner configuration:



