# Moveit2 Example (Panda VS UR Arm)

This Demo aims to show that Moveit2 is robot agnostic. Move Group Interface code remains the same while being implemented on a separate robot. Robot motion differs due to the Robot's dimensions and randomness of the planners used.

## Running the Panda demo

Terminal 1(ROS2 Control FakeHardware Driver + Move Group Demo)
```bash
ros2 launch move_group_interface_example move_group.launch.py

```
Terminal 2(Move Group Interface demo)
```bash
ros2 launch move_group_interface_example move_group_interface_tutorial.launch.py 
```

## Running the URSim demo
Terminal 1 (Virtual Teach pendant)

```bash
ros2 run ur_client_library start_ursim.sh -m ur5e

```

Ensure to go into the browser link and prepare the teach pendant for the UR driver to work

Terminal 2 (Robot Driver (URSim))

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.56.101 launch_rviz:=true
```
Terminal 3 (Move group Node)

```bash
ros2 launch move_group_interface_example ur_move_group.launch.py ur_type:=ur5e

```
Terminal 4 (Move group Interface demo)

```bash
ros2 launch move_group_interface_example ur_move_group_interface_tutorial.launch.py 
```
Terminal 5

```bash

```