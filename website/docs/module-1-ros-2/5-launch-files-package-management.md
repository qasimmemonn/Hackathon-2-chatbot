---
sidebar_position: 5
---

# Chapter 5: Launch Files and Package Management

So far, we've written nodes and run them one by one from the terminal. This is fine for simple examples, but a real robot is a complex orchestra of dozens of nodes, each needing specific configurations. Starting them manually is not feasible. This is where **launch files** come in—they are the conductors of your ROS 2 orchestra.

We will also cover **packages**, the fundamental building blocks for organizing, building, and sharing your ROS 2 code.

## The ROS 2 Workspace and Packages

All your code in ROS 2 should be organized into **packages**. A package is simply a directory with a specific structure and a `package.xml` manifest file. Multiple packages live inside a **workspace**. A typical workspace has the following structure:

-   `src/`: This is where you put your source code, organized into packages.
-   `build/`: `colcon` (the ROS 2 build tool) uses this directory for intermediate build files. You rarely need to touch this.
-   `install/`: After a successful build, the compiled programs, Python modules, launch files, and other assets are installed here.
-   `log/`: Log files from the build process are stored here.

### The `package.xml` Manifest
This file is the heart of your package. It defines the package's name, version, maintainer, license, and—most importantly—its dependencies.

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_robot_tutorials</name>
  <version>0.0.1</version>
  <description>Tutorials for the Physical AI book.</description>
  <maintainer email="student@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <!-- Dependencies needed to build the code -->
  <buildtool_depend>ament_python</buildtool_depend>

  <!-- Dependencies needed at build time -->
  <build_depend>rclpy</build_depend>
  <build_depend>std_msgs</build_depend>
  <build_depend>my_robot_interfaces</build_depend>

  <!-- Dependencies needed at execution time -->
  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>my_robot_interfaces</exec_depend>

  <!-- Dependencies needed only for running tests -->
  <test_depend>pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

### The `setup.py` File (for Python Packages)
For Python packages (`ament_python`), the `setup.py` file tells `colcon` how to install your package and, crucially, how to create executable scripts from your Python files. The `entry_points` dictionary is where you map the desired executable name to your Python module and main function.

```python
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_tutorials'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files from the 'launch' directory
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Include all config files from the 'config' directory
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='student@example.com',
    description='Tutorials for the Physical AI book.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'executable_name = package_name.python_file:main_function'
            'battery_publisher = my_robot_tutorials.battery_publisher:main',
            'battery_monitor = my_robot_tutorials.battery_monitor:main',
        ],
    },
)
```

## Launch Files: Scripting Your ROS 2 System

In ROS 2, launch files are powerful Python scripts that give you full programmatic control over starting your system.

### A Basic Launch File

Let's create a launch file to start our battery publisher and monitor nodes from Chapter 2.

**`my_robot_tutorials/launch/battery_app.launch.py`**
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='battery_publisher',
            name='battery_publisher_node' # You can optionally rename the node
        ),
        Node(
            package='my_robot_tutorials',
            executable='battery_monitor',
            name='battery_monitor_node'
        ),
    ])
```
To run this, you use the `ros2 launch` command, which automatically finds the file in your package's `install/share` directory (after you've built it).

```bash
# Build your workspace
colcon build --packages-select my_robot_tutorials

# Source the setup file to make the executables and launch files available
source install/setup.bash

# Run the launch file
ros2 launch my_robot_tutorials battery_app.launch.py
```

### Advanced Launch File Features

#### Launch Arguments
You often want to pass arguments to your launch file from the command line. `DeclareLaunchArgument` and `LaunchConfiguration` are used for this.

```python
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare an argument named 'use_low_battery_warning' with a default value of 'true'
    use_low_battery_warning_arg = DeclareLaunchArgument(
        'use_low_battery_warning',
        default_value='true',
        description='Whether to start the battery monitor node.'
    )

    return LaunchDescription([
        use_low_battery_warning_arg,
        Node(...),
        # Conditionally launch the monitor node based on the argument
        Node(
            package='my_robot_tutorials',
            executable='battery_monitor',
            condition=IfCondition(LaunchConfiguration('use_low_battery_warning'))
        ),
    ])
```
Now you can run:
`ros2 launch my_robot_tutorials battery_app.launch.py use_low_battery_warning:=false`

#### Loading Parameter Files
This is the standard way to configure your nodes. We build on the `ParameterNode` example from Chapter 3.

```python
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the YAML file inside your package
    config_file = os.path.join(
        get_package_share_directory('my_robot_tutorials'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='parameter_node',
            name='my_parameter_node',
            # Load parameters from the YAML file
            parameters=[config_file]
        )
    ])
```

#### Remapping Topics
What if a node you want to use publishes to `/camera/image` but your system expects `/head_camera/image_raw`? You can remap topics in the launch file without changing any code.

```python
Node(
    package='camera_driver',
    executable='camera_node',
    remappings=[
        ('/camera/image', '/head_camera/image_raw'),
        ('/camera/camera_info', '/head_camera/camera_info'),
    ]
)
```
#### Including Other Launch Files
Complex systems are built from smaller, reusable launch files.

```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Get the path to the launch file in another package
    arm_control_launch_file = os.path.join(
        get_package_share_directory('my_robot_arm_control'),
        'launch',
        'arm_control.launch.py'
    )

    return LaunchDescription([
        # Include the entire arm control system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(arm_control_launch_file),
            launch_arguments={'use_fake_hardware': 'true'}.items()
        ),
        # Add other nodes for your application
        Node(...)
    ])
```

## The Complete Workflow
1.  **Create:** Create a new package:
    `ros2 pkg create --build-type ament_python my_new_package --dependencies rclpy`
2.  **Code:** Write your Python nodes inside `my_new_package/my_new_package/`.
3.  **Configure `setup.py`:** Add your launch files, config files, and the `entry_points` for your executables.
4.  **Configure `package.xml`:** Add all necessary dependencies.
5.  **Build:** From the root of your workspace, run `colcon build`.
6.  **Source:** In your terminal, run `source install/setup.bash`. You must do this in every new terminal you open.
7.  **Launch:** Run your application: `ros2 launch my_new_package my_app.launch.py`.

This concludes Module 1. You have journeyed from the fundamental concepts of ROS 2 to the practical skills needed to build, organize, and launch complex, multi-node robotics applications. You are now equipped with the foundational software engineering skills required to tackle the next phase of our project: creating a digital twin of our humanoid robot in a realistic physics simulator.