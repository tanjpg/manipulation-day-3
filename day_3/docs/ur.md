# Introduction to the Universal Robot (Operation in a ROS2 Context)

This document serves to provide ROS2 Driver specific information for running the Universal Robot Driver.

## Table of Contents  
### [Mock UR Driver](#mock-ur-driver) 

### [URSim](#ur-sim)

### [Real Hardware](#real-robot-hardware)  

### [Other Resources](#other-resources)  

---

# Mock UR Driver
The mock UR driver is a ROS2 Control FakeHardware Driver, hence no prior UR knowledge is required

## Terminal 1: Run FakeHardware

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_mock_hardware:=true launch_rviz:=false initial_joint_controller:=joint_trajectory_controller
```

## Terminal 2: Run Move Group

You may want to then run a move group node to test the driver. Since this is a mock driver, you will need to run the following to launch a move group node.

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true use_mock_hardware:=true
```

# UR Sim 
UR Sim is a virtual teach pendant that simulates a real UR Robot. 

## Terminal 1: Run URSim
```bash
ros2 run ur_client_library start_ursim.sh -m ur5e
```

Next, in the browser go to the following link : http://192.168.56.101:6080/vnc.html and click `connect`. You should then be able to access the virtual Teach Pendant

Ensure that the [robot is set up](./teach_pendant.md#setting-up-the-robot) for use.

Next [create a program](./teach_pendant.md#creating-a-program) and adding an External Control component

## Terminal 2: Run the UR Driver

Ensure that the `use_fake_hardware` option is set to `false`, and the `robot_ip` is set to `192.168.56.101`

This also runs the ROS2 Control Controller Manager and the relevant Controllers you declare in the controller yaml files.

```bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_fake_hardware:=false robot_ip:=192.168.56.101 launch_rviz:=true

```

**After running this line, make sure to [play the External Control program](./teach_pendant.md#running-a-program) on the UR Robot.**

## Terminal 3 and beyond

Next you will need to then run your path planning nodes. the example below launches a move group node for the robot. Additionally you may have custom MoveGroupInterface instructions that can be launched as well. 
```bash

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

```

# Real Robot Hardware

Ensure that the [robot is set up](./teach_pendant.md#setting-up-the-robot) for use.

Next [create a program](./teach_pendant.md#creating-a-program) and adding an External Control component

First, [Check the IP of the robot](./teach_pendant.md#check-or-edit-robot-ip-address). Next [Check the External Control IP for your computer](./teach_pendant.md#check-or-edit-external-control-ip-address). Ensure that they do not clash. Next, Connect the ethernet cable to your computer and [Configure the IP of your computer]() to match the External Control IP. You should be now connected and be able to ping the robot's IP

```bash
ping <ip address>
```

## Terminal 1: Run the UR Driver
```bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_fake_hardware:=false robot_ip:=<robot_ip> launch_rviz:=true

```
**After running this line, make sure to [play the External Control program](./teach_pendant.md#running-a-program) on the UR Robot.**

## Terminal 2 and beyond

Next you will need to then run your path planning nodes. the example below launches a move group node for the robot. Additionally you may have custom MoveGroupInterface instructions that can be launched as well. 
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

```
# Additional Resources
[Universal Robot](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

[Teach Pendant Features Full](./teach_pendant.md)
