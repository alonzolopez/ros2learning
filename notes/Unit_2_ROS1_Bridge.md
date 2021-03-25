# Unit 2: ROS1 Bridge
Migration to ROS2 is still lagging, especially in the case of Gazebo simulations. The ROS1 Bridge helps us bridge this gap, duh.

The ROS1-Bridge connects messages from ROS1 to ROS2 via mappings defined at compile-time through yaml files exported in the **package.xml**. For more info on this mapping topic, click [here](https://github.com/ros2/ros1_bridge/blob/master/doc/index.rst)

## ROS1-Bridge Sourcing
First, you need to create a custom bridge bashrc config to source both the ROS1 and ROS2 systems that ROS1-Bridge needs.

**ROS1-bridge** needs to source the paths for all of the messages it has access to. That means reaching the ROS1 and ROS2 messages in melodic and foxy AND catkin_ws and ros2_ws
```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/local_setup.bash
source /home/user/catkin_ws/devel/setup.bash
source /home/user/ros2_ws/install/local_setup.bash
```

## Basic Examples
*.bashrc_bridge* will contain something like
```
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/local_setup.bash
source /home/user/catkin_ws/devel/setup.bash
source /home/user/ros2_ws/install/local_setup.bash
```

*.bashrc_ros1* will contain something like
```
export ROS_DISTRO=noetic
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/user/catkin_ws/devel/setup.bash
```
And *.bashrc_ros2* will contain something like
```
export ROS_DISTRO=foxy
source /opt/ros/$ROS_DISTRO/setup.bash
source /home/user/ros2_ws/install/local_setup.bash
```

The ROS1-Bridge is launched in shell #1, the ROS1 node is launched in shell #2, and the ROS2 node is launched in shell #3.

The basic examples go thorugh a talker and listener in ROS1 and ROS2 as well as the other way around, a cross-ROS1/2 service, and then more interestingly the simulation examples: publishing an image in ROS1 and viewing in ROS2; commanding motion in a ROS1 Gazebo simulation using a ROS2 node; calling the ROS1 Gazebo reset service from ROS2. 

