from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("ur_gripper").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="pick_and_place",
        package="pick_and_place",
        executable="pick_and_place",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"planning_group" : "ur_manipulator"},
            {"base_link" : "base_link"},
            {"end_effector_planning_group" : "gripper"},
            {"end_effector_finger_links" : ["gripper_finger1_finger_tip_link", "gripper_finger2_finger_tip_link"]},
            {"move_robot" : True}
        ],
    )

    return LaunchDescription([move_group_demo])
