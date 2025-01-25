Navigation2 Custom Planners

Evaluating and Comparing Global and Local Planners in ROS2

Overview
This project involves the creation and testing of custom ROS2 components for evaluating the performance of various global and local planners in the ROS2 Navigation2 stack. It provides a simulation setup in RViz to test and visualize the navigation paths for robots with different footprints.

Key Features:

Supports global planners like NavFn, ThetaStar, and SmacPlanner2D.
Supports local planners like DWBLocalPlanner and TEBLocalPlanner.
Configurable for robots with different tail angles (0° and 45°).
Includes predefined scenarios for testing and evaluation.

Project Structure
lp_nav/
├── launch/
│   ├── navigation_launch2.py          # Launch file for navigation
├── maps/
│   ├── map.yaml
│   ├── simulation-map.pgm             # Map for simulation
├── urdf/
│   ├── robot.urdf                     # Robot URDF model
├── config/
│   ├── nav2_params-0.yaml             # Params for 0° tail
│   ├── nav2_params-45.yaml            # Params for 45° tail
├── scripts/
│   ├── fake_odometry_node.py          # Script for generating fake odometry
├── CMakeLists.txt                     # Build system configuration
├── package.xml                        # ROS2 package configuration
