---
sidebar_position: 3
---

# Chapter 3: Services, Actions, and Parameters

While topics are the lifeblood of continuous data streams in ROS 2, not all communication fits the publish-subscribe model. Sometimes you need a direct request-response interaction, or you need to manage a long-running task with feedback. For these, ROS 2 provides **Services** and **Actions**. We will also dive deep into **Parameters**, a crucial tool for making your nodes flexible and configurable without changing a single line of code.

## Services: The Synchronous Request-Response Model

A **service** provides a strict request-response interaction. A **client** node sends a single request to a **server** node and waits (blocks) until it receives a single response. This is a synchronous, one-to-one communication pattern.

Services are ideal for quick, atomic tasks, such as:
-   Querying the state of a node: "get_robot_status"
-   Triggering a discrete action: "take_picture"
-   Performing a calculation for another node: "calculate_inverse_kinematics"

### Defining a Service

A service is defined in a `.srv` file, which specifies the request and response message types, separated by `---`.

Let's create a service to add two integers. Create a file named `AddTwoInts.srv` in a folder called `srv` inside your package.

**`my_robot_tutorials/srv/AddTwoInts.srv`**
```
# Request
int64 a
int64 b
---
# Response
int64 sum
```

After creating this file, you need to rebuild your package so that ROS 2 can generate the necessary source code for Python and C++.

### Implementing a Service

#### Service Server (`add_two_ints_server.py`)
The server waits for requests, performs the calculation, and sends back the response.

```python
import rclpy
from rclpy.node import Node
from my_robot_tutorials.srv import AddTwoInts

class AddTwoIntsServer(Node):
    def __init__(self):
        super().__init__('add_two_ints_server')
        # Create the service, specifying the type, name, and callback
        self.srv = self.create_service(
            AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Add Two Ints Server has started.')

    def add_two_ints_callback(self, request, response):
        # The request and response are passed as arguments
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request: a={request.a}, b={request.b}. Returning sum={response.sum}')
        # Return the response
        return response

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Service Client (`add_two_ints_client.py`)
The client sends a request and waits for the server's response.

```python
import rclpy
from rclpy.node import Node
from my_robot_tutorials.srv import AddTwoInts

class AddTwoIntsClient(Node):
    def __init__(self):
        super().__init__('add_two_ints_client')
        # Create a client for the 'add_two_ints' service
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        # Call the service asynchronously
        self.future = self.client.call_async(self.request)
        self.get_logger().info(f'Sending request: a={a}, b={b}')

def main(args=None):
    rclpy.init(args=args)
    node = AddTwoIntsClient()
    node.send_request(5, 10)

    # Spin until the future is complete
    while rclpy.ok():
        rclpy.spin_once(node)
        if node.future.done():
            try:
                response = node.future.result()
                node.get_logger().info(f'Result of add_two_ints: {response.sum}')
            except Exception as e:
                node.get_logger().error(f'Service call failed {e!r}')
            break

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
### Using Services from the Command Line

You can also call services directly from the terminal, which is incredibly useful for debugging.

```bash
# List all available services
$ ros2 service list
/add_two_ints
...

# See the type of a service
$ ros2 service type /add_two_ints
my_robot_tutorials/srv/AddTwoInts

# Call the service with arguments in YAML format
$ ros2 service call /add_two_ints my_robot_tutorials/srv/AddTwoInts "{a: 2, b: 3}"
requester: making request: my_robot_tutorials.srv.AddTwoInts_Request(a=2, b=3)
---
responder:
my_robot_tutorials.srv.AddTwoInts_Response(sum=5)
```

## Actions: For Long-Running, Asynchronous Tasks

What if a task takes a long time? A service client would be blocked, unable to do anything else. This is where **Actions** come in. Actions are for asynchronous tasks that provide feedback during execution.

The action pattern is perfect for tasks like:
-   Navigating to a coordinate: The goal is the destination, feedback is the current distance to the goal, and the result is whether navigation succeeded.
-   Executing a robot arm trajectory.
-   Processing a large dataset.

### Defining an Action
An action is defined in a `.action` file, with three parts separated by `---`: the goal, the result, and the feedback.

Let's define a "countdown" action.
**`my_robot_tutorials/action/Countdown.action`**
```
# Goal definition
int32 start_number
---
# Result definition
bool success
---
# Feedback definition
int32 remaining
```

### The Action Lifecycle
1.  **Goal:** An action client sends a goal to an action server.
2.  **Accept/Reject:** The server accepts or rejects the goal.
3.  **Execute & Feedback:** If accepted, the server starts executing the task. During execution, it can send periodic feedback to the client. The client can also request to cancel the goal at any time.
4.  **Result:** When the task finishes, the server sends a final result.

### Implementing an Action (Python Example)

Implementing a full action server and client is more involved, but it follows this clear pattern. The "fibonacci" or "countdown" examples from the official ROS 2 documentation are excellent starting points that we encourage you to explore as you build your own packages. For brevity, we will focus on the command-line interaction here.

### Using Actions from the Command Line

The `ros2 action` command set is your primary tool for testing and debugging actions.

```bash
# List all available actions
$ ros2 action list

# Get info about an action
$ ros2 action info /countdown

# Send a goal and see the feedback and result
$ ros2 action send_goal /countdown my_robot_tutorials/action/Countdown "{start_number: 10}" --feedback
```
This command will send the goal, print feedback messages as the server sends them, and finally print the result when the action completes.

## Parameters: Making Your Nodes Configurable

Hard-coding values like timer rates, topic names, or controller gains into your code is bad practice. It makes your nodes rigid and hard to reuse. **Parameters** solve this by externalizing configuration.

A node can declare parameters, and their values can be set from a launch file, a YAML file, or the command line, without ever touching the source code.

### Declaring and Using Parameters

```python
import rclpy
from rclpy.node import Node

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare a parameter with a name and default value
        self.declare_parameter('robot_name', 'default_robot')
        self.declare_parameter('max_speed', 1.2)

        # Get the parameter's value
        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        max_speed = self.get_parameter('max_speed').get_parameter_value().double_value

        self.get_logger().info(f'Robot name is: {robot_name}')
        self.get_logger().info(f'Max speed is: {max_speed} m/s')

# ... main function ...
```
### Managing Parameters from the Command Line

```bash
# Run the node
$ ros2 run my_robot_tutorials parameter_node

# List the parameters for a node
$ ros2 param list /parameter_node
max_speed
robot_name
...

# Get the value of a parameter
$ ros2 param get /parameter_node max_speed
Double value is: 1.2

# Set the value of a parameter at runtime
$ ros2 param set /parameter_node robot_name "humanoid_v1"
Set parameter successful

# Get the new value
$ ros2 param get /parameter_node robot_name
String value is: humanoid_v1
```

### The Power of YAML and Launch Files

The most common way to manage parameters is with YAML files.

**`my_robot_tutorials/config/params.yaml`**
```yaml
parameter_node:
  ros__parameters:
    robot_name: "humanoid_from_yaml"
    max_speed: 0.8
```

You can then load this file in a launch file to set the parameters when the node starts. This is the standard workflow for configuring complex robots.

**`my_robot_tutorials/launch/start_robot.launch.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the YAML file
    config = os.path.join(
        get_package_share_directory('my_robot_tutorials'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='parameter_node',
            name='parameter_node',
            parameters=[config] # Load parameters from the file
        )
    ])
```

By mastering services, actions, and parameters, you have unlocked the full suite of ROS 2's communication and configuration tools. You can now build systems that are not only modular and scalable but also robust and highly configurable. In the next chapter, we will define the physical structure of our robot using URDF.