# Introduction to ROS2 Manipulation Exercise 1

## Exercise Objective

This exercise will require the learner to use the Moveit Setup Assiscreate a Moveit Config Package of the Universal Robot UR5e.


In the `exercise_1/supplementary_packages` folder in exercise 1, you will see a `universal-robot-resources` folder. This folder contains essential files needed
for the entire Day 3. 

For now, take a look at the `universal-robot-resources/ur_demo_description/urdf` folder. This folder contains a list of URDFs for various UR Robot variants. In our particular exercise,  you will be required to work with the **UR5e**

Remember to launch moveit setup assistant, run the following command 

```bash
source /
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

## Technical requirements

The following are a list of requirements to take into consideration when using the Moveit Setup Assistant

1. Generate Collision Matrix
2. No Virtual Joints
3. We will define 2 planning groups as shown:
*ur_manipulator*
- Set group name as `ur_manipulator`
- set Kinematic SOlver as `KDLKinematicsPlugin`
- Set Group Default Planner as `RRTStar`
- Add a Kinematic Chain with the Base Link as `base_link` and tip link as `tool0`

*ur_hand*
- Set group name as `ur_hand`
- set Kinematic Solver as `KDLKinematicsPlugin`
- Set Group Default Planner as `RRTStar`
- Add a Kinematic Chain with the Base Link as `flange` and tip link as `tool0`
4. No Robot Poses for Now
5. Define an End Effector as follows
- Set End effector name as `end_effector`
- Set the group name as `ur_hand`
- Set the parent link as `flange`
- Set the parent group as `ur_manipulator`
6. No Passive Joints
7. Leave the interface options as it is.

8. Automatically Add ROS2 Control JointTrajectoryController Controllers

9. Automatically Add Moveit JointTrajectoryController Controllers

10. Skip Configuring Perception elements

11. Fill In Author Information

12. Ensure that the Configuration path is set to `/home/rosi/workspaces/training_ws/src/ur5e_moveit_config`

13. Generate the Moveit config package

**IMPORTANT. Make sure to do the following. This is required due to some changes in the XACRO library** 

```
After Generating the moveit_config folder, edit the `ur5e_moveit_config/config/ur_demo.ros2_control.xacro`, on line 4, from 

....value="${load_yaml(....)}

to 

....value="${xacro.load_yaml(....)}

```