---
sidebar_position: 2
---

# Chapter 2: ROS 2 Nodes and Topics

In the previous chapter, we introduced the high-level architecture of ROS 2. Now, we'll get our hands dirty with the two most fundamental building blocks of any ROS 2 application: **Nodes** and **Topics**. Mastering these is the first step toward building complex robotic systems.

## Nodes: The Computational Units of ROS 2

A **node** is best thought of as a single, independent program responsible for one specific task. In a complex system like a humanoid robot, you don't write one giant monolithic program. Instead, you break the problem down into many small, manageable nodes.

For our humanoid, we might have:
-   A `/camera_driver` node that captures images from the head camera.
-   An `/object_detector` node that processes these images to find objects.
-   A `/leg_controller` node for each leg to manage joint movements.
-   A `/path_planner` node that determines how to walk from point A to B.
-   A `/battery_monitor` node that tracks the robot's power level.

This modularity is a core tenet of ROS. It makes the system easier to develop, debug, test, and reuse. If the object detector crashes, the legs can still keep the robot balanced. If you want to upgrade to a better camera, you only need to change the `/camera_driver` node; the rest of the system remains untouched.

### Creating a ROS 2 Node

Let's look at the structure of a minimal node in both Python and C++.

#### Python (`rclpy`)

```python
# simple_node.py
import rclpy
from rclpy.node import Node

class MySimpleNode(Node):
    def __init__(self):
        # Initialize the Node with a name
        super().__init__('my_simple_node')
        self.get_logger().info('My Simple Node has started!')

def main(args=None):
    # 1. Initialize the rclpy library
    rclpy.init(args=args)

    # 2. Create an instance of our node
    node = MySimpleNode()

    # 3. "Spin" the node to process callbacks
    rclpy.spin(node)

    # 4. Shutdown the rclpy library
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### C++ (`rclcpp`)
```cpp
// simple_node.cpp
#include "rclcpp/rclcpp.hpp"

class MySimpleNode : public rclcpp::Node
{
public:
    MySimpleNode() : Node("my_simple_node")
    {
        RCLCPP_INFO(this->get_logger(), "My Simple Node has started!");
    }
};

int main(int argc, char **argv)
{
    // 1. Initialize the rclcpp library
    rclcpp::init(argc, argv);

    // 2. Create an instance of our node
    auto node = std::make_shared<MySimpleNode>();

    // 3. "Spin" the node to process callbacks
    rclcpp::spin(node);

    // 4. Shutdown the rclcpp library
    rclcpp::shutdown();
    return 0;
}
```

### What Does "Spinning" Mean?

The `rclpy.spin(node)` or `rclcpp::spin(node)` call is crucial. It starts an event loop that waits for incoming messages, service requests, or other events and dispatches them to the appropriate **callbacks** (functions you write to handle these events). Without `spin()`, your node would be created and then immediately exit, doing nothing.

By default, this happens in a single thread. This means if you have a callback that takes a long time to execute, it will block all other callbacks in that node. ROS 2 provides more advanced **executors** (like multi-threaded executors) to handle multiple callbacks concurrently, a topic we'll revisit when we discuss more complex systems.

## Topics: The Data Buses of ROS 2

If nodes are the workers, **topics** are the conveyor belts that move data between them. A topic is simply a named bus. Nodes can publish data (messages) to a topic, and any other nodes can subscribe to that topic to receive the data.

This **publish-subscribe** model is powerful because it decouples the nodes. The camera node doesn't know or care who is using its images. It simply publishes them to the `/image_raw` topic. The object detector, a data logger, and a remote monitor can all subscribe to `/image_raw` simultaneously without the camera node's knowledge.

### Messages: The Language of Topics

Data is sent over topics in the form of **messages**. A message is a simple data structure, defined in a language-agnostic `.msg` file. For example, the standard `String` message is defined as:

```
# in std_msgs/msg/String.msg
string data
```

ROS 2 then automatically generates the corresponding code for Python (`std_msgs.msg.String`), C++ (`std_msgs::msg::String`), and other supported languages. This ensures that nodes written in different languages can communicate seamlessly.

ROS provides a vast library of standard message types:
-   `std_msgs`: Basic data types (string, int32, bool, etc.).
-   `sensor_msgs`: For common sensors (Image, LaserScan, Imu, PointCloud2).
-   `geometry_msgs`: For representing poses, transforms, and velocities (Pose, Twist, TransformStamped).

### A Practical Example: Robot Battery Monitor

Let's build a system with two nodes: one that simulates publishing a robot's battery level and another that monitors it.

#### The Publisher (`battery_publisher.py`)

This node will publish the battery percentage on the `/battery_level` topic every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class BatteryPublisher(Node):
    def __init__(self):
        super().__init__('battery_publisher')
        # Create a publisher for the /battery_level topic with a message type of Float32
        self.publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.battery_level = 100.0
        self.get_logger().info('Battery publisher started.')

    def timer_callback(self):
        msg = Float32()
        # Simulate battery drain
        self.battery_level -= random.uniform(0.1, 0.5)
        if self.battery_level < 0:
            self.battery_level = 100.0 # Recharge

        msg.data = self.battery_level
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery level: {msg.data:.2f}%')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### The Subscriber (`battery_monitor.py`)

This node subscribes to the `/battery_level` topic and prints a warning if the level gets too low.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitor(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        # Create a subscriber to the /battery_level topic
        self.subscription = self.create_subscription(
            Float32,
            'battery_level',
            self.listener_callback,
            10)
        self.get_logger().info('Battery monitor started.')

    def listener_callback(self, msg):
        battery_level = msg.data
        self.get_logger().info(f'Received battery level: {battery_level:.2f}%')
        if battery_level < 20.0:
            self.get_logger().warn('Low battery! Please charge the robot.')

def main(args=None):
    rclpy.init(args=args)
    node = BatteryMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Quality of Service (QoS): Controlling Data Delivery

What if a message is really important and can't be lost? What if you're sending high-frequency data and it's okay to drop a few messages? ROS 2 provides **Quality of Service (QoS)** policies to give you fine-grained control over how topics behave.

When you create a publisher or subscriber, you can specify a QoS profile. The most important policies are:

| Policy        | Options               | Description                                                                                             | Use Case Example                                       |
|---------------|-----------------------|---------------------------------------------------------------------------------------------------------|--------------------------------------------------------|
| **Reliability** | `RELIABLE`            | Guarantees delivery. Retries sending if a message is lost.                                              | Sending a command to a robot arm (`/move_arm`).        |
|               | `BEST_EFFORT`         | Attempts to deliver but may drop messages on congested or unreliable networks. Lower latency.           | High-frequency sensor data like a camera feed (`/image_raw`). |
| **Durability**  | `VOLATILE`            | Subscribers only receive messages published after they have connected.                                  | Most standard topic communication.                     |
|               | `TRANSIENT_LOCAL`     | "Late-joining" subscribers receive the last N published messages.                                       | A configuration topic where new nodes need the latest value (`/robot_description`). |
| **History**     | `KEEP_LAST`           | Store up to N messages in a queue (specified by `depth`).                                               | The standard, most common setting.                     |
|               | `KEEP_ALL`            | Store all messages, up to the resource limits of the system.                                            | Use with caution; can consume a lot of memory.         |

The publisher and subscriber must have compatible QoS settings to establish a connection. Understanding QoS is critical for building robust robotic systems.

## Introspection with Command-Line Tools

ROS 2 provides powerful command-line tools to inspect and interact with your running system. Assuming you have saved the code above in a package named `my_robot_tutorials`, you would run them in separate terminals:

```bash
# Terminal 1: Run the publisher
ros2 run my_robot_tutorials battery_publisher

# Terminal 2: Run the subscriber
ros2 run my_robot_tutorials battery_monitor
```

Now, in a third terminal, you can use these tools:

-   **List all running nodes:**
    ```bash
    $ ros2 node list
    /battery_monitor
    /battery_publisher
    ```

-   **List all active topics:**
    ```bash
    $ ros2 topic list
    /battery_level
    /parameter_events
    /rosout
    ```

-   **"Eavesdrop" on a topic to see the data being published:**
    ```bash
    $ ros2 topic echo /battery_level
    data: 98.67
    ---
    data: 98.23
    ---
    ```

-   **Get detailed information about a topic, including its QoS settings:**
    ```bash
    $ ros2 topic info /battery_level
    Type: std_msgs/msg/Float32
    Publisher count: 1
    Subscription count: 1
    ```

-   **Publish a single message manually from the command line:**
    ```bash
    $ ros2 topic pub /battery_level std_msgs/msg/Float32 "{data: 5.0}"
    ```

-   **Show the definition of a message type:**
    ```bash
    $ ros2 interface show std_msgs/msg/Float32
    # A single-precision floating-point number.
    float32 data
    ```

By mastering nodes, topics, and these essential command-line tools, you now have the foundational skills to build and debug a wide range of ROS 2 applications. In the next chapter, we will explore synchronous communication patterns: Services and Actions.