from launch import LaunchDescription
from launch_ros.actions import Node
import os
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    fake_gripper_config = os.path.join( 
        get_package_share_directory('robotiq_fake_service'), 'config', 'config.yaml')
    # Fake gripper service server
    fake_finger_gripper_server_node = Node(
        package='robotiq_fake_service',
        executable='fake_service_server',
        name='robotiq_fake_service',
        output='screen',
        parameters=[fake_gripper_config]
    )

    launch_description = LaunchDescription()
    launch_description.add_action(fake_finger_gripper_server_node)
    return launch_description
