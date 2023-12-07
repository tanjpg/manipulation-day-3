from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    #moveit_config = MoveItConfigsBuilder("ur5e").to_moveit_configs()
    moveit_config = MoveItConfigsBuilder("ur_demo", package_name="ur5e_moveit_config").to_moveit_configs()    
    move_group_demo = Node(
        name="move_group_interface_example",
        package="move_group_interface_example",
        executable="move_group_interface_tutorial",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"planning_group" : "ur_manipulator"},
            {"base_link" : "base_link"},
            {"end_effector_planning_group" : "ur_hand"},
            {"end_effector_finger_links" : ["tool0"]},
            {"move_robot" : True}
        ],
    )

    return LaunchDescription([move_group_demo])
