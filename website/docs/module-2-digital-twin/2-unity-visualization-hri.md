---
sidebar_position: 2
---

# Chapter 2: Unity Visualization and Human-Robot Interaction

While Gazebo is our workhorse for accurate physics simulation, the **Unity** game engine offers a world-class platform for high-fidelity graphics, immersive virtual reality (VR), and rich Human-Robot Interaction (HRI). In this chapter, we'll explore how to use Unity as a powerful visualization and interaction front-end for our ROS 2-driven digital twin.

## Unity vs. Gazebo: The Right Tool for the Job

It's not about which tool is better, but what you need to accomplish. A powerful digital twin strategy often involves using both.

-   **Use Gazebo when:** Your primary concern is the accuracy of physics. You are developing and testing low-level controllers, validating dynamics, or training reinforcement learning policies that rely on realistic physical interactions.
-   **Use Unity when:** Your primary concern is visualization or interaction. You need photorealistic graphics for generating synthetic sensor data, you want to build intuitive user interfaces, or you need to simulate complex interactions between humans and robots in VR/AR.

| Feature                      | Gazebo                                      | Unity (with Robotics Hub)                     |
| ---------------------------- | ------------------------------------------- | --------------------------------------------- |
| **Primary Strength**         | High-fidelity, tunable physics simulation   | Photorealistic graphics and interaction design|
| **Physics Engine**           | Multiple options (ODE, Bullet, DART)        | NVIDIA PhysX (highly optimized for games)     |
| **Rendering**                | Functional, OGRE-based engine               | State-of-the-art, scriptable render pipeline  |
| **Ecosystem**                | Gazebo Models database                      | Unity Asset Store (a massive marketplace)     |
| **ROS Integration**          | Deep, using `ros_gz` bridges                | Via the **Unity Robotics Hub** (TCP-based)    |

## The Unity Robotics Hub and its Architecture

Unity communicates with ROS 2 via the **Unity Robotics Hub**, a set of open-source packages. The core of this connection is different from Gazebo's bridge. It uses a client-server model over a standard network connection.

**Architecture:**
1.  **ROS 2 Side:** You run a special `ros_tcp_endpoint` node. This node acts as a server, listening on a specific IP address and port. It converts incoming TCP packets into ROS 2 messages and vice-versa.
2.  **Unity Side:** The `ROS-TCP-Connector` package in your Unity project acts as a client. When you press "Play" in the Unity Editor, it connects to the `ros_tcp_endpoint` server.
3.  **Communication:** Once connected, your C# scripts in Unity can publish and subscribe to ROS 2 topics just as if they were standard ROS 2 nodes.

![Unity ROS Connector Diagram](https://i.imgur.com/rL4e1rZ.png)

This architecture means your Unity simulation doesn't even have to run on the same machine as your ROS 2 system, as long as they are on the same network.

## Setting Up Your First Unity Robotics Project

1.  **Install Unity:** Download and install the Unity Hub. Use it to install a recent, stable version of the Unity Editor (e.g., 2022.3.x LTS).
2.  **Create a Project:** In Unity Hub, create a new 3D project.
3.  **Install Robotics Packages:**
    -   Go to `Window > Package Manager`.
    -   Click the `+` icon and choose `Add package from git URL...`.
    -   Add the official Unity Robotics package: `com.unity.robotics.ros-tcp-connector`. This will automatically install other necessary dependencies.
4.  **Configure ROS Connection:**
    -   In the Unity Editor, go to `Robotics > ROS Settings`.
    -   Set the `ROS IP Address` to the IP of the machine running ROS 2 (or `127.0.0.1` if it's the same machine).
    -   Leave the `ROS Port` as the default (10000).

## Working with a Robot in Unity

The Robotics Hub provides a tool to import your URDF directly into Unity.

1.  **Import URDF:** Go to `Robotics > URDF Importer` and select your robot's `.urdf` or `.xacro` file.
2.  **Configure Articulations:** The importer will generate a `GameObject` hierarchy that mirrors your URDF's link structure. It will automatically add `ArticulationBody` components to the `GameObjects`, which are Unity's equivalent of physics joints. You can configure their properties (drive stiffness, damping, friction) directly in the Unity Inspector.
3.  **Controlling Joints with C#:** To make the robot move, you create C# scripts and attach them to your robot `GameObject`. This script can subscribe to a ROS 2 topic and use the received data to command the `ArticulationBody` joints.

**Example C# Script (`JointController.cs`):**
This script subscribes to a `/joint_command` topic and sets the target position of a single joint.

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std; // Assumes we are using a std_msgs/Float32 message

public class JointController : MonoBehaviour
{
    // The joint we want to control
    public ArticulationBody joint;

    void Start()
    {
        // Get the ROSConnection instance
        ROSConnection.GetOrCreateInstance().Subscribe<Float32Msg>("/joint_command", OnCommandReceived);
    }

    // Callback function for when a message is received
    void OnCommandReceived(Float32Msg message)
    {
        // Get the current drive state of the joint
        var drive = joint.xDrive;
        // Set the target position based on the received message data
        drive.target = message.data;
        // Apply the new drive state
        joint.xDrive = drive;
    }
}
```
You would then attach this script to a `GameObject` in your scene and drag the desired `ArticulationBody` (e.g., the "base_to_arm" joint) into the `joint` field in the Inspector.

## Building an Interactive HRI Scene

This is where Unity excels. Let's create a simple "click-to-command" interface. The goal is to allow a user to click anywhere on a ground plane, and have Unity publish that location as a goal for our robot.

1.  **Create a Publisher Script (`ClickGoalPublisher.cs`):**

    ```csharp
    using UnityEngine;
    using Unity.Robotics.ROSTCPConnector;
    using RosMessageTypes.Geometry; // For PoseStamped

    public class ClickGoalPublisher : MonoBehaviour
    {
        // The ROS topic to publish to
        public string topicName = "/nav_goal";
        private ROSConnection ros;

        void Start()
        {
            ros = ROSConnection.GetOrCreateInstance();
            // Register the publisher
            ros.RegisterPublisher<PoseStampedMsg>(topicName);
        }

        void Update()
        {
            // Check for a left mouse button click
            if (Input.GetMouseButtonDown(0))
            {
                // Create a ray from the camera to the mouse position
                Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
                RaycastHit hit;

                // Perform the raycast
                if (Physics.Raycast(ray, out hit))
                {
                    // Create a new PoseStamped message
                    var goal = new PoseStampedMsg();
                    goal.header.frame_id = "world";
                    goal.pose.position = hit.point; // Use the point where the ray hit
                    goal.pose.orientation.w = 1.0; // Default orientation

                    // Publish the message
                    ros.Publish(topicName, goal);

                    Debug.Log($"Published goal to {topicName}: {hit.point}");
                }
            }
        }
    }
    ```

2.  **Set up the Scene:**
    -   Create a large plane `GameObject` to serve as the ground.
    -   Create an empty `GameObject` and attach the `ClickGoalPublisher.cs` script to it.
    -   Ensure your scene has a camera.

## Running the Full Digital Twin System

1.  **ROS 2 Side:** In a terminal, run the `ros_tcp_endpoint`.
    ```bash
    # (Assuming you've installed the endpoint package)
    ros2 run ros_tcp_endpoint default_server_endpoint
    ```
    You would also run any other ROS 2 nodes you need, like a navigation system that subscribes to `/nav_goal`.

2.  **Unity Side:**
    -   Open your Unity project.
    -   Press the **Play** button at the top of the editor.

The Unity simulation will start, the `ROS-TCP-Connector` will connect to the ROS 2 endpoint, and your ROS-enabled scripts will become active. Now, when you click on the ground plane in the Unity "Game" view, the `ClickGoalPublisher` script will send a goal message that your ROS 2 navigation stack can receive and act upon.

By combining Gazebo's physics with Unity's visualization and interaction capabilities, you can create a truly comprehensive digital twin, enabling rapid development and testing of sophisticated AI and HRI applications. In the next chapter, we'll dive deeper into a critical aspect of simulation: modeling sensors.