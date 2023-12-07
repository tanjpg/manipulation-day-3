from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    # There may be other controllers of the joints, but this is the initially-started one
    gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "-p",
            os.path.join( 
                get_package_share_directory('gripper_driver_interface'), 'config', 'controllers.yaml'),
            "-t",
            "joint_trajectory_controller/JointTrajectoryController",
            "--controller-manager",
            "/controller_manager"
        ]
    )
    launch_description = LaunchDescription()
    launch_description.add_action(gripper_controller)
    return launch_description
