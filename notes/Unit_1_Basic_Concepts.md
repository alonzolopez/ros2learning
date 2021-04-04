# Unit 1: Basic Concepts
Still using the same concept of packages with a launch folder, src folder, CmakeLists.txt, and package.xml.

How to run a ros2 executable:
```
ros2 run <package_name> <executable_file>
```

How to run a ros2 launch file:
```
ros2 launch <package_name> <launch_file>
```

A package contains:
- **launch** folder
- **src** folder
- **CMakeLists.txt**: list of cmake rules for compilation
- **package.xml**: package info and dependencies

## Create a package
First, in order to use the ROS2 command line tools, 
```
source /opt/ros/foxy/setup.bash
```
Then create a package:
```
ros2 pkg create <package_name> --build-type ament_cmake --dependencies <package_dependencies>
```
*package_dependencies* are other ROS packages that this new package depends on.

To see all packages and optionally filter search for one,
```
ros2 pkg list
ros2 pkg list | grep my_package
```

## Compile a Package
To compile ROS packages:
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To compile select packages,
```
colcon build --packages-select my_package
```

## Launch Files
Launch files are different in ROS2!!!

You can generate them from python. For example,
```python
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(package='teleop_twist_keyboard', executable='teleop_twist_keyboard', output='screen')
    ])
```
Three key params are required:
- **package**: name of the package containing the ROS program to execute
- **executable**: name of the cpp executable file to execute
- **output**: where you want to print the output of the program

## My First ROS Program
When making a new launch file,
```
cd ~/ros2_ws/src/my_package/launch
touch my_package_launch_file.launch.py
chmod +x my_package_launch_file.launch.py
```

And edit CMakeLists.txt before building to build the executable by adding the following lines before ament_package()
```
add_executable(simple_node src/simple.cpp)
ament_target_dependencies(simple_node rclcpp)

install(TARGETS
    simple_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)
```

then build with colcon build and source install/setup.bash

## Modifying the CMakeLists.txt file
To create the binary executables for your program, we modify CMakeLists to indicate that we want to create the executables.

Taking apart the previous CMakeLists.txt file we made,
```txt
// generates an executable called simple_node from simple.cpp
add_executable(simple_node src/simple.cpp)

// adds all the ament target dependencies of the executable
ament_target_dependencies(simple_node rclcpp)

// installs the node (places the executable) in the package directory of the 
// install space (i.e. ~/ros2_ws/install/<package_name>/lib)
install(TARGETS
    simple_node
    DESTINATION lib/${PROJECT_NAME}
)

// installs launch files to the install space so they can be executed with ros2 launch
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/
)
```

## ROS Nodes
Node = program.

To see active nodes and get some info on it
```
ros2 node list
ros2 node info <node_name>
```

To program a node to run until shutdown with Ctrl + C
```cpp
int main(int argc, char * argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("ObiWan");

    rclcpp::WallRate loop_rate(2); // create the loop rate, Hz

    while (rclcpp::ok())    // while loop to keep it spinning
    {
        RCLCPP_INFO(node->get_logger(), "Help me Obi-Wan Kenobi, you're my only hope");
        rclcpp::spin_some(node);    // add this
        loop_rate.sleep();          // add this
    }

    rclcpp::shutdown();
    return 0;
}
```

## Client Libraries
ROS client libraries allow nodes written in different programming languages to communicate. There is a core ROS Client Library (RCL) that implements the common functionality needed for ROS APIs across different languages. The following client libs are maintained by the ROS2 team:
- **rclcpp** := C++ client library
- **rclpy** := Python client library

## Environment Variables
ROS uses a set of Linux system environment variables to run. You can check these by typing:
```
export | grep ROS
```
The most important env variables are **ROS_MASTER_URI** and **ROS_DISTRO**. Since ROS2 doesn't have a roscore, ROS_MASTER_URI points to the roscore running in ROS1 when communicating with the ROS1 bridge.

