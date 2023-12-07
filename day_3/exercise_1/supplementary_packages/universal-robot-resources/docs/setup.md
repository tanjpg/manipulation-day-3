# Download and setup

There will be three main workspaces to build

| Workspace name | Description |
| ----------- | ---------------- |
| _demo_ws_ | Workspace containing application specific files |
| _moveit2_ws_ | Workspace containing Moveit2 Packages. We will not edit files here | 
| _ur_driver_ws_ | Workspace containing UR Driver Packages. We will not edit files here | 

## Setting Up Workspaces

```bash

mkdir -p ~/workspaces/demo_ws/src

mkdir -p ~/workspaces/ur_driver_ws/src

mkdir -p ~/workspaces/moveit2_ws/src

git clone git@gitlab.com:ROSI-AP/rosi-ap_toolbox/universal-robot.git ~/workspaces/demo_ws/src

git clone git@gitlab.com:ROSI-AP/rosi-ap_toolbox/robotiq-grippers.git ~/workspaces/demo_ws/src
```

## Setting Up Moveit2 Workspace

```bash
source /opt/ros/humble/setup.bash

cd ~/workspaces/moveit2_ws/src

vcs import < ~/workspaces/demo_ws/src/universal-robot/moveit2.repos

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/workspaces/moveit2_ws

colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```
## Setting Up UR Driver Workspace

```bash
cd ~/workspaces/ur_driver_ws/src

vcs import < ~/workspaces/demo_ws/src/universal-robot/ur_driver.repos

rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y

cd ~/workspaces/ur_driver_ws

source ~/workspaces/moveit2_ws/install/setup.bash

colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Setting Up Demo Workspace
```bash

cd ~/workspaces/demo_ws

source ~/workspaces/ur_driver_ws/install/setup.bash

source ~/workspaces/moveit2_ws/install/setup.bash

colcon build

```