# Introduction to ROS2 Manipulation Exercise 2

## Exercise Objective

This exercise will require the learner to use the Moveit Setup Assistant to create a new URDF that adds the Robotiq 2F Gripper to the Universal Robot UR5e, and create a Moveit Config Package of the new system

Take a look at the `robotiq_85_description` folder inside `exercise_2`'s `supplementary_packages` folder. This is a ROS2 Package that provide you with descriptive information of the Robotiq 85 Gripper

## 1. Create a URDF of a UR5e with an attached Robotiq 85 Gripper.

A new package, `ur_gripper_description` has been created for you. in the `urdf` folder, fill up the `ur_gripper.xacro` with the correct urdf description a UR5e robot with a Robotiq 85 gripper connected to it.

**Tips to write the URDF File**

In order to spawn the UR5e, refer to the `ur5e.urdf.xacro` file used in exercise 1. 

In order to spawn and add the Robotiq Gripper, refer to the file in `robotiq_85_description/urdf/spawn_gripper.xacro`'s `robotiq_gripper` macro.

When instantiating the `robotiq_gripper` macro,
- Set the parent parameter to tool0
- Set the prefix parameter to ""
- Set the end_effector parameter to "robotiq_2f"
- Set the origin to be 0,0,0 (xyz) and 0,0,0 (rpy)


Note that we can only have 1 `world` link declared in the URDF.

once you are done with the urdf, remember to build and source your workspace

```bash

cd <your workspace>

colcon build

source install/setup.bash
```

## 2. Create a Moveit Config Package of the above urdf
Next, we will create a Moveit Config Package for this URDF

Remember to launch moveit setup assistant, run the following command 

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### Technical requirements

The following are a list of requirements to take into consideration when using the Moveit Setup Assistant.

Remember to select the `ur_gripper.xacro` file as the target file for config creation.

1. Automatically Generate Collision Matrix
2. No Virtual Joints
3. We will define 2 planning groups as shown:
*ur_manipulator*
- Set group name as `ur_manipulator`
- set Kinematic SOlver as `KDLKinematicsPlugin`
- Set Group Default Planner as `RRTStar`
- Add a Kinematic Chain with the Base Link as `base_link` and tip link as `ur_to_robotiq_link`
*ur_hand*
- Set group name as `gripper`
- set Kinematic Solver as `KDLKinematicsPlugin`
- Set Group Default Planner as `RRTStar`
- Add a new joint, `gripper_finger1_joint`
4. No Robot Poses for Now
5. Define an End Effector as follows
- Set End effector name as `gripper`
- Set the group name as `gripper`
- Set the parent link as `ur_to_robotiq_link`
- Set the parent group as `ur_manipulator`
6. Add the following joints as passive joints
- `gripper_finger1_inner_knuckle_joint`
- `gripper_finger1_finger_tip_joint`
- `gripper_finger2_inner_knuckle_joint`
- `gripper_finger2_finger_tip_joint`
- `gripper_finger2_joint`

7. Leave the interfaces being added as it is

8. Automatically Add ROS2 Control JointTrajectoryController Controllers
- Edit the `ur_manipulator_controller` field, and rename the controller name to `joint_trajectory_controller`

9. Automatically Add Moveit JointTrajectoryController Controllers
- Edit the `ur_manipulator_controller`field, and rename the controller name to `joint_trajectory_controller`

10. Skip Configuring Perception elements

11. Fill In Author Information

12. Ensure that the Configuration path is set to `/home/rosi/workspaces/training_ws/src/ur_gripper_moveit_config`

13. Generate the Moveit config package


**IMPORTANT. Make sure to do the following. This is required due to some changes libraries** 
1. Updated Xacro Libraries

    After Generating the moveit_config folder, edit the `ur_gripper_moveit_config/config/ur_demo.ros2_control.xacro`, on line 4, from `load_yaml` to `xacro.load_yaml`

```
....value="${load_yaml(....)}

to 

....value="${xacro.load_yaml(....)}

```

2. Update Gripper Limits (Bug)
    After Generating the moveit_config folder, edit the `ur_gripper_moveit_config/config/joint_limits.yaml`, and change the `max_velocity` component
    of `gripper_finger1_joint` from `2` to `2.0`

```

  ...
  gripper_finger1_joint:
    has_velocity_limits: true
    max_velocity: 2
    has_acceleration_limits: false
    max_acceleration: 0
 ...

to 

  ...
  gripper_finger1_joint:
    has_velocity_limits: true
    max_velocity: 2.0
    has_acceleration_limits: false
    max_acceleration: 0
 ...
```