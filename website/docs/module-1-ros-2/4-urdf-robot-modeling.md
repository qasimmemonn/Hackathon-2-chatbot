---
sidebar_position: 4
---

# Chapter 4: URDF Robot Modeling

To simulate, control, or visualize a robot, ROS needs a comprehensive description of its physical structure. The standard for this is the **Unified Robot Description Format (URDF)**. URDF is an XML-based format that creates a model of your robot by defining its links, joints, sensors, and appearance. This model is the cornerstone of almost every robotics application.

A well-defined URDF is essential for:
-   **Simulation:** Physics engines like Gazebo use the URDF to create a virtual, interactive model of your robot.
-   **Visualization:** Tools like RViz2 use the URDF to display the robot's 3D model, sensor data, and coordinate frames.
-   **Kinematics and Dynamics:** Libraries use the URDF to calculate the robot's motion (forward/inverse kinematics) and how it responds to forces (dynamics).

## The Core Components of URDF: Links and Joints

A URDF model describes a robot as a tree of **links** connected by **joints**. There is one root link, and every other link has exactly one parent.

### `<link>`: The Rigid Body
A link represents a rigid part of the robot's body, like a forearm, a wheel, or a chassis. Each `<link>` element has three key aspects you must define: `<visual>`, `<collision>`, and `<inertial>`.

```xml
<link name="my_link">
  <inertial>
    <!-- ... for physics ... -->
  </inertial>
  <visual>
    <!-- ... for rendering ... -->
  </visual>
  <collision>
    <!-- ... for collision detection ... -->
  </collision>
</link>
```

1.  **`<visual>`:** This defines what the link looks like in visualization tools like RViz2. It's all about appearance.
    -   **`<geometry>`:** Can be a simple shape (`<box>`, `<cylinder>`, `<sphere>`) or a 3D mesh file (`<mesh filename="package://my_package/meshes/my_link.stl"/>`).
    -   **`<origin>`:** The pose (position and orientation) of the visual geometry relative to the link's own coordinate frame.
    -   **`<material>`:** Defines the color and texture of the link.

2.  **`<collision>`:** This defines the geometry of the link for the physics engine's collision detection.
    -   For performance, the collision geometry is often a simplified version of the visual geometry. For example, a complex visual mesh might be represented by a simple box or cylinder for collision purposes.

3.  **`<inertial>`:** This describes the dynamic properties of the link for the physics engine. **This is critical for realistic simulation.**
    -   **`<mass value="..."/>`:** The mass of the link in kilograms.
    -   **`<origin xyz="..." rpy="..."/>`:** The pose of the Center of Mass (CoM) relative to the link's origin.
    -   **`<inertia ixx="..." ixy="..." .../>`:** The 6x6 inertia tensor matrix, which describes how the link's mass is distributed around its center of mass. Incorrect inertia values will lead to unrealistic simulation behavior. CAD software can often calculate these values for you.

### `<joint>`: The Connection Between Links
A joint connects a **parent** link to a **child** link, defining their relationship and the motion allowed between them.

| Joint Type      | Description                                            | Example Use Case        |
|-----------------|--------------------------------------------------------|-------------------------|
| **`revolute`**  | A hinge joint that rotates around a single axis with limits. | An elbow or knee joint. |
| **`continuous`**| Rotates around an axis with no limits.                 | A wheel or a spinning lidar. |
| **`prismatic`** | A sliding joint that moves along an axis with limits.  | A linear actuator or a piston. |
| **`fixed`**     | Allows no motion between two links.                    | Mounting a sensor to the chassis. |
| **`floating`**  | Allows motion in all 6 degrees of freedom.             | Connecting a free-flying robot to the world. |
| **`planar`**    | Allows motion in a 2D plane.                           | A robot moving on a flat surface. |

A `<joint>` tag has several important sub-tags:
-   **`<parent link="..."/>`** and **`<child link="..."/>`**: Defines the two links being connected.
-   **`<origin xyz="..." rpy="..."/>`**: The pose of the joint (and thus the child link's frame) relative to the parent link's frame. This is one of the most important tags for defining your robot's structure.
-   **`<axis xyz="..."/>`**: The axis of rotation (for revolute/continuous joints) or translation (for prismatic joints) in the joint's own coordinate frame.
-   **`<limit lower="..." upper="..." velocity="..." effort="..."/>`**: For revolute and prismatic joints, defines the motion limits, maximum velocity, and maximum force/torque.

## URDF and the Transformation (`tf2`) Tree

A URDF defines the *static* geometry of a robot. But how does ROS know the current pose of each link as the joints move? This is handled by the `tf2` transformation library.

A central node called `robot_state_publisher` subscribes to a topic named `/joint_states` (which contains the current angle/position of each non-fixed joint) and, using the URDF, calculates and broadcasts the complete tree of transformations between all links.

**Workflow:** `URDF Model` + `sensor_msgs/JointState` messages -> `robot_state_publisher` -> `tf2` transforms.

![Robot State Publisher Diagram](https://i.imgur.com/uRj0mY4.png)

This means you don't have to calculate the kinematics yourself! You can simply ask `tf2` for the transform between any two links (e.g., "what is the pose of the `right_hand` link relative to the `head_camera` link?"), and it will provide the answer based on the current joint states.

## Beyond URDF: XACRO for Clean, Modular Models

Writing a complex robot like a humanoid in raw URDF would be incredibly repetitive and error-prone. To solve this, ROS uses **XACRO (XML Macros)**. XACRO is a macro language that enhances URDF with variables, math, and reusable blocks. You write `.xacro` files and then convert them to a final `.urdf` file at runtime.

### Key XACRO Features

1.  **Properties (Variables):** Define constants to avoid "magic numbers".
    ```xml
    <xacro:property name="wheel_radius" value="0.1"/>
    <xacro:property name="wheel_mass" value="0.5"/>
    ```

2.  **Macros (Functions):** Create reusable templates for components like links, joints, or entire sensor assemblies.

    ```xml
    <xacro:macro name="default_inertial" params="mass">
      <inertial>
        <mass value="${mass}" />
        <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
      </inertial>
    </xacro:macro>

    <!-- Use the macro -->
    <xacro:default_inertial mass="10"/>
    ```

### Example: A Parametric Two-Link Arm in XACRO

Let's refactor the simple arm from the previous version of this chapter into a cleaner XACRO file.

**`simple_arm.xacro`**
```xml
<?xml version="1.0"?>
<robot name="simple_arm" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Constants -->
  <xacro:property name="base_radius" value="0.2" />
  <xacro:property name="base_length" value="0.6" />
  <xacro:property name="arm_length" value="0.5" />

  <!-- A macro for creating a link with a simple box visual -->
  <xacro:macro name="box_link" params="name mass">
    <link name="${name}">
      <visual>
        <geometry><box size="${arm_length} 0.1 0.1"/></geometry>
        <material name="white"><color rgba="1 1 1 1"/></material>
      </visual>
      <collision><geometry><box size="${arm_length} 0.1 0.1"/></geometry></collision>
      <inertial>
        <mass value="${mass}"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>
      </inertial>
    </link>
  </xacro:macro>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry><cylinder radius="${base_radius}" length="${base_length}"/></geometry>
      <material name="blue"><color rgba="0 0 0.8 1"/></material>
    </visual>
    <collision><geometry><cylinder radius="${base_radius}" length="${base_length}"/></geometry></collision>
    <xacro:default_inertial mass="1"/>
  </link>

  <!-- Arm Link created using our macro -->
  <xacro:box_link name="arm_link" mass="0.5"/>

  <!-- Joint -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <axis xyz="0 0 1"/>
    <origin xyz="0 0 ${base_length / 2}"/>
    <limit effort="1000.0" lower="0.0" upper="1.57" velocity="0.5"/>
  </joint>

</robot>
```
This version is far more readable and maintainable. If we want to change the arm length, we only have to modify it in one place.

## Visualizing and Testing Your URDF

How do you know if your URDF is correct? You visualize it! The following launch file provides a complete setup for viewing your robot model in RViz2 and controlling its joints with a GUI.

**`display.launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the path to the XACRO file
    xacro_file = os.path.join(
        get_package_share_directory('my_robot_tutorials'),
        'urdf',
        'simple_arm.xacro')

    # Get the path to the RViz configuration file
    rviz_config_file = os.path.join(
        get_package_share_directory('my_robot_tutorials'),
        'rviz',
        'display.rviz')

    return LaunchDescription([
        # This node converts the XACRO file to a URDF string and publishes it as a parameter
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(xacro_file).read()}]
        ),

        # This node provides a GUI to control the robot's joints
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        # This node starts RViz2 with the specified configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])
```
Running this launch file will open RViz2 and a small GUI window. By moving the sliders in the GUI, you can change the joint angles, and you will see your robot model articulate in real-time in RViz2. This is the fundamental workflow for developing and verifying any robot model in ROS.

In the next chapter, we'll learn how to properly structure and manage our code within ROS 2 packages and use launch files to run our complete application.