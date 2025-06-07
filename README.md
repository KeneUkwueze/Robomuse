# New Robomuse6.2 Robot


1. [ROS Noetic + Gazebo Classic 11 (branch ros1)](#noetic--classic-ubuntu-2004)
2. [ROS2 Humble + Gazebo Harmonic (branch ros2)](#humble--harmonic-ubuntu-2204)

Each of the following sections describes depedencies, build and run instructions for each of the above combinations

## Noetic + Classic (Ubuntu 20.04)

### Dependencies

In addition to ROS1 Noetic and Gazebo Classic installations, the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Source Build

```bash
catkin build --packages-select robomuse
```
### Run

To launch the robot in Gazebo,
```bash
roslaunch robomuse gazebo.launch
```
To view in rviz,
```bash
roslaunch robomuse rviz.launch
```


## Humble + Harmonic (Ubuntu 22.04)

### Dependencies

In addition to ROS2 Humble and [Gazebo Harmonic installations](https://gazebosim.org/docs/harmonic/install_ubuntu), we need to manually install interfaces between ROS2 and Gazebo sim as follows,

```bash
sudo apt-get install ros-humble-ros-gzharmonic
```
Remainder of the dependencies can be installed with [rosdep](http://wiki.ros.org/rosdep)

```bash
# From the root directory of the workspace. This will install everything mentioned in package.xml
rosdep install --from-paths src --ignore-src -r -y
```

### Build

```bash
colcon build --packages-select robomuse
```

### Run

To launch the robot in Gazebo,
```bash
ros2 launch robomuse gz.launch.py
```
To view in rviz,
```bash
ros2 launch robomuse rviz.launch.py
```

### Mapping with SLAM Toolbox

SLAM Toolbox is an open-source package designed to map the environment using laser scans and odometry, generating a map for autonomous navigation.


To start mapping:
```bash
ros2 launch robomuse mapping.launch.py
```

Use the teleop twist keyboard to control the robot and map the area:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/robomuse/cmd_vel
```

To save the map:
```bash
cd src/robomuse/config
ros2 run nav2_map_server map_saver_cli -f bcr_map
```

### Using Nav2 with robomuse

Nav2 is an open-source navigation package that enables a robot to navigate through an environment easily. It takes laser scan and odometry data, along with the map of the environment, as inputs.


To run Nav2 on robomuse:
```bash
ros2 launch robomuse nav2.launch.py
```