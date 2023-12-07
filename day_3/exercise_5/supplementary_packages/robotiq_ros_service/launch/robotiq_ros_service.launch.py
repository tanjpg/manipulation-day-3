from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import  LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.conditions import UnlessCondition, IfCondition

def generate_launch_description():
    launch_description = LaunchDescription()

    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
 
    launch_description.add_action(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value="true"
        )
    )
    gripper_config = os.path.join( 
        get_package_share_directory('robotiq_ros_service'), 'config', 'config.yaml')
    
    fake_server_node = Node(
        package='robotiq_ros_service',
        executable='robotiq_ros2_server',
        name='robotiq_ros2_server',
        condition=IfCondition(use_fake_hardware),
        output='screen',
        parameters=[
            gripper_config]
    )

    launch_description.add_action(fake_server_node)

    real_gripper_hardware = Node(
        package="robotiq_urcap_control",
        executable="cmodel_urcap_driver",
        output="screen",
        emulate_tty=True,
        condition=UnlessCondition(use_fake_hardware),
        parameters=[
            {'gripper_service': "/robotiq_urcap_control"},
            {'gripper_joint_publisher' : "/gripper_controller/joint_trajectory"},
            {'ip_address': "192.168.0.30"},
            {"gripper_joint" : "gripper_finger1_joint"},
            {"use_fake_hardware" : use_fake_hardware}]
    )

    launch_description.add_action(real_gripper_hardware)
    return launch_description
