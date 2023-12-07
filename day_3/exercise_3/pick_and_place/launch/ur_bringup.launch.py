from moveit_configs_utils import MoveItConfigsBuilder
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python import get_package_share_directory
import yaml

def generate_launch_description():
    moveit_config_package = "ur_gripper_moveit_config"
    demo_package = "pick_and_place"
    config_file = "config.yaml"
    planning_plugin = "ompl_interface/OMPLPlanner"

    # moveit_config = MoveItConfigsBuilder(moveit_config_package, package_name=moveit_config_package)
    # .joint_limits(Path("my_config") / "my_file.srdf")
    # .to_moveit_configs()
    
    moveit_config = MoveItConfigsBuilder(moveit_config_package, package_name=moveit_config_package).to_moveit_configs()
    ld = LaunchDescription() 

    # Load config file for UR
    with open(get_package_share_directory(demo_package) + "/config/" + config_file, 'r') as stream:
        try:
            # Converts yaml document to python object
            loaded_yaml=yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e)

    # Start the actual move_group node/action server
    # ld.add_action(Node(
    #     package="moveit_ros_move_group",
    #     executable="move_group",
    #     output="screen",
    #     parameters=[
    #         moveit_config.to_dict(),
    #         {"ompl.planning_plugin" : planning_plugin}],
    # ))

    # Run MoveGroup
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run RViz
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(moveit_config.package_path / "launch/moveit_rviz.launch.py")
            ),
        )
    )
        

    # Run UR_Driver
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare("ur_config_utils"), "launch/ur_control.launch.py"]
                )
            ),

            launch_arguments=
                loaded_yaml.items()
        )
    )

    return ld


