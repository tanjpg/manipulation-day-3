# Introduction to ROS2 Manipulation Exercise 3

## Exercise Objective

This exercise will require the learner to add grasping capabilities to the pick and place in Exercise 3

## Technical requirements

You will need to edit the `pick_and_place.cpp` file in `exercise_3` to include capabilities to actuate the Robotiq2f Gripper. 

**If you are using the real hardware, the service name is now `/robotiq_urcap_control`. If you are not connected to real gripper hardware, the service name remains as /gripper_service**

As a hint, you can use what you have developed in exercise_5 and include it into exercise 3.

**NOTE**
To prevent collision of the object with inner parts of the gripper, you can limit the **downward (z) movement** to half of the actual value.

Also, the grasping width of the gripper should be limited between `30` to `85`

## Test if it works!

To test if your package works, first make sure to build and source your workspace
```bash
cd <workspace>

colcon build

source install/setup.bash
```

You will now need 2 terminals. 

In Terminal 1: 
```bash
cd <workspace>

source install/setup.bash

ros2 launch pick_and_place ur_bringup.launch.py
```

In Terminal 2: 
```bash
cd <workspace>

source install/setup.bash

ros2 launch gripper_driver_interface gripper_bringup.launch.py 
```

In Terminal 3: 

**Set `use_fake_hardware` to `false` if you are using a real gripper!**
```bash
cd <workspace>

source install/setup.bash

ros2 launch robotiq_ros_service robotiq_ros_service.launch.py  use_fake_hardware:=true
```

In Terminal 4: 
```bash
cd <workspace>

source install/setup.bash

ros2 launch pick_and_place pick_and_place.launch.py 
```
You should now be able to see the Pick and Place solution you have created executed in the RViz Simulation, with the gripper actuation.