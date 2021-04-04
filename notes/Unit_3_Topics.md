# Unit 3: Topics (Publishers)
## Basics
A topic is a channel that acts as a pipe where other ROS nodes can either publish or read information. 

A publisher is a node that keeps publishing a message into a topic.

To list ros2 topics:
```
ros2 topic list
```
To get info on the topics:
```
ros2 topic info <topic>
```
To echo the topic:
```
ros2 topic echo <topic>
```
To get help on the ros2 topic command:
```
ros2 topic -h
```

## Node Composition
The old ROS1 method of creating nodes is being deprecated/abandoned for [Node Composition](https://docs.ros.org/en/foxy/Tutorials/Composition.html). Node composition is a more object-oriented way of writing ROS programs, and we saw a little bit of it in some of the nicer ROS1 code. For more about the concept of node composition, click [here](https://docs.ros.org/en/foxy/Concepts/About-Composition.html). [Efficient intra-process communication](https://docs.ros.org/en/foxy/Tutorials/Intra-Process-Communication.html) might also be a good read.

To compare the two methods, this is the old way:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_publisher");
  auto publisher = node->create_publisher<std_msgs::msg::Int32>("counter", 10);
  auto message = std::make_shared<std_msgs::msg::Int32>();
  message->data = 0;
  rclcpp::WallRate loop_rate(2);

  while (rclcpp::ok()) {
    
    publisher->publish(*message);
    message->data++;
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
```


And this is the new "node composition" way taking advantage of classes:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <chrono>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SimplePublisher : public rclcpp::Node
{
public:
  SimplePublisher()
  : Node("simple_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&SimplePublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimplePublisher>());
  rclcpp::shutdown();
  return 0;
}
```

Let's break down this code. First, we define our class which inherits from rclcpp::Node
```cpp
class SimplePublisher : public rclcpp::Node
```
Next, we have the constructor of our class
```cpp
SimplePublisher()
```
this constructor calls the constructor of the superclass **Node**
```cpp
: Node("simple_publisher"), count_(0)
```
Also within the constructor, we define our **publisher_** and **timer_** objects. These objects are declared in the private section of our class as **SharedPtr**s to the objects. Also note that the `timer_` object is bound to a function named **timer_callback**.
```cpp
publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);
timer_ = this->create_wall_timer(
  500ms, std::bind(&SimplePublisher::timer_callback, this));
```
In the definition of the **timer_callback** function, we create a message of type **Int32**, set the data field of the message to the **count_** variable, and increment the **count_** variable. Finally, we publish the message. This function is called every 500ms as a callback to the timer. 
```cpp
void timer_callback()
  {
    auto message = std_msgs::msg::Int32();
    message.data = count_;
    count_++;
    publisher_->publish(message);
  }
```
Also in the private section, we declare the shared pointers to the **timer_** and **publisher_** objects defined above as well as the **count_** variable.
```cpp
rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
size_t count_;
```
Finally, the main just initializes rclcpp and creates a SimplePublisher object and spins until it's shutdown with CTRL+C.


## Messages (interfaces)
ROS1's messages are known as **interfaces** in ROS2. Interfaces are defined in **.msg** files, which are located inside of the **msg** directory of a package. 

To get info about a message:
```
ros2 interface show <message>
```
For example, 
```
ros2 interface show std_msgs/msg/Int32
```
output:
```
# This was originally provided as an example message.
# It is deprecated as of Foxy
# It is recommended to create your own semantically meaningful message.
# However if you would like to continue using this please use the equivalent in example_msgs.

int32 data
```

# Unit 3: Topics (Subscribers and Messages)
## Topic Subscribers
Create a simple subscriber package dependent on rclcpp and std_msgs:
```
ros2 pkg create topic_subscriber_pkg --build-type ament_cmake --dependencies rclcpp std_msgs
```

The subscriber's simple_topic_subscriber.cpp looks like:
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
using std::placeholders::_1;

class SimpleSubscriber : public rclcpp::Node {
public:
  SimpleSubscriber() : Node("simple_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "counter", 10, std::bind(&SimpleSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "I heard: '%d'", msg->data);
  }
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

After writing the node executable and launch file and launching the subscriber node, publish data in the terminal using:
```
ros2 topic pub /counter std_msgs/msg/Int32 data:\ 6\
```

## How to Create a Custom Interface
In order to create a new message, follow these steps:

1. Create a directory named "msg" inside your package
    ```
    cd ~/ros2_ws/src/<package-name>
    mkdir msg
    ```
2. Inside `msg/`, create a file named `Name_of_your_message.msg`. For `Age.msg`, populate it with
    ```
    float32 years
    float32 months
    float32 days
    ```
3. Modify two functions inside CMakeLists.txt: find_package() and rosidl_generate_interfaces()
  * find_package() is where all the packages required to COMPILE the messages for topics, services, and actions go. You have to later state them as **build_depend**s and **exec_depend**s in **package.xml**
    ```
    find_package(ament_cmake REQUIRED)
    find_package(rclcpp REQUIRED)
    find_package(std_msgs REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    ```
  * The **ros_idl_generate_interfaces()** function includes all of the messages of this package (in the msg folder) to be compiled. It should look like this:
  ```
  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Age.msg"
  )
  ```
4. Modify package.xml by adding the following depends:
  ```xml
  <build_depend>rosidl_default_generators</build_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  ```
5. Compile and source. In the below example, the new package we made for our custom message is `new_msg`
  ```
  cd ~/ros2_ws
  colcon build --packages-select new_msg
  source install/setup.bash
  ```
**Hint:** verify that your message has been created with
  ```
  ros2 interface show topic_subscriber_pkg/msg/Age
  ```
6. Use the msg in code. If the message is created as part of one package, for example, the `new_msg` package, you'll need to add the following to your CMakeLists.txt in other packages that use this msg. For example, add the following to the CMakeLists.txt in `topic_subscriber_pkg`
  ```
  # this is the pkg that contains the custom interface
  find_package(new_msg REQUIRED) 

  add_executable(age_publisher_node src/publish_age.cpp)

  # add the pkg that contains the custom interface as a 
  # dependency of this pkg that uses the interface
  ament_target_dependencies(age_publisher_node rclcpp std_msgs new_msg)

  install(TARGETS
    age_publisher_node
    DESTINATION lib/${PROJECT_NAME}
  )
  ```
And add the following to **package.xml**:
  ```
  <depend>new_msg</depend>
  ```

