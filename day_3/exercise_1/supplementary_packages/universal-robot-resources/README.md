# The Universal Robot Repository

This repository contains all the information required for operating a Universal Robot as well as code examples for integration.


# Download and Setup Guide
Please read the [Download and Setup Guide](docs/setup.md)


# Guides
1. [How to run UR teach pendant (WIP)]()
2. [How to Setup UR ROS2 Driver (WIP)]()
2. [How to Setup UR With Moveit2](docs/moveit_integration.md)
3. [How to use UR with custom ROS2 Projects (WIP)]()

# Repository Breakdown
This repository contains the following packages:

| Package name | Description |
| ----------- | ---------------- |
| _ur_config_utils_ | Helper Package for integration of UR robots with external applications |
| _ur_demo_description_ | Package containing the URDFs of all the demo scenarios | 
| _ur_epick_moveit_config_ | Auto generated Package via the [Moveit2 Setup Assistant ](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) for the Single Robot, Single Suction Cup Demo | 
| _ur_robotiq85_moveit_config_ | Auto generated Package via the [Moveit2 Setup Assistant ](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html) for the Single Robot, Two Finger Gripper Demo |
