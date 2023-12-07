from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()

    # MoveGroupInterface demo executable
    move_group_demo = Node(
        name="move_group_interface_example",
        package="move_group_interface_example",
        executable="move_group_interface_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"planning_group" : "panda_arm"},
            {"base_link" : "panda_link0"},
            {"end_effector_planning_group" : "panda_hand"},
            {"end_effector_finger_links" : ["panda_rightfinger", "panda_leftfinger"]},
            {"move_robot" : True}
        ],
    )

    return LaunchDescription([move_group_demo])
