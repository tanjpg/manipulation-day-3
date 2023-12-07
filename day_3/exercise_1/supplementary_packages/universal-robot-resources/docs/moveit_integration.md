# Integration with Moveit2

## Examples

The UR Driver already provides [examples](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html) of integration with Moveit2, however this repository provides additional examples of integration with:
1. [Single Robot, Robotiq 2f-85 Finger Gripper](../ur_demo_description/urdf/ur_robotiq_85.xacro)
2. [Single Robot, Robotiq Epick Suction Gripper](../ur_demo_description/urdf/ur_epick.xacro)
3. Multiple robots (WIP)

A [simplified launch file](../ur_demo/launch/robotiq_85_demo.launch.py) is also provided to highlight modular methods of launching a moveit2 based UR manipulation system with other packages

## Creating your own Moveit2 Application
1. Create a description package similar to [ur_description](../ur_demo_description/)
2. Use the Moveit2 Setup Assistant GUI to create a moveit_config package [Like this](../ur_robotiq85_moveit_config/)
    ```bash
    ros2 launch moveit_setup_assistant setup_assistant.launch.py 
    ```

    This repository's instructions and example code assumes you do not modify the automatically generated folders and files.

3. Create a config file. This file should be residing in your example package's `config` folder with the following contents:
    ```yaml
    ur_type: 'ur10e'
    robot_ip: '192.168.56.101'
    safety_limits: 'true'
    safety_pos_margin: '0.15'
    safety_k_position: '20'
    demo_package: 'ur_demo'
    runtime_config_package: 'ur_config_utils'
    controllers_file: 'ur_robotiq85_controllers.yaml'
    description_package: "ur_robotiq85_moveit_config"
    description_file: 'ur_demo.urdf.xacro'
    tf_prefix: '""'
    use_fake_hardware: 'true'
    fake_sensor_commands: 'false'
    headless_mode: 'false'
    controller_spawner_timeout: '10'
    initial_joint_controller: 'joint_trajectory_controller'
    activate_joint_controller: 'true'
    launch_rviz: 'false'
    launch_dashboard_client: 'true'
    use_tool_communication: 'false'
    tool_parity: '0'
    tool_baud_rate: '115200'
    tool_stop_bits: '1'
    tool_rx_idle_chars: '1.5'
    tool_tx_idle_chars: '3.5'
    tool_device_name: '/tmp/ttyUR'
    tool_tcp_port: '54321'
    tool_voltage: '0'
    reverse_ip: '0.0.0.0'
    script_command_port: '50004'
    ```

    Explanation of these parameters can be mostly found [here](https://docs.ros.org/en/ros2_packages/rolling/api/ur_robot_driver/usage.html). Make sure to edit the config file as per your required use.

4. Create a launch file in your package's `launch` directory. An example of the launch file is shown [here](../ur_demo/launch/robotiq_85_demo.launch.py). Modify the file with your relevant parameters.