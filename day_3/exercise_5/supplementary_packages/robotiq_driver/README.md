# robotiq

## Setup

  * Robotiq 85 Gripper with UR5

### Testing the Robotiq gripper

Source the appropriate workspace
```{bash}
source ~/${WORKSPACE}/install/setup.bash
```

Terminal 1

Try running the `robotiq_urcap_control` node:
```{bash}
ros2 run robotiq_urcap_control robotiq_urcap_driver
```

Terminal 2

Send a request from an example client:
```{bash}
ros2 run robotiq_urcap_control robotiq_example_client
```

