---
sidebar_position: 6
title: Assessments & Exercises
---

# Assessments & Exercises

This section provides questions and exercises to test your understanding of the concepts covered in each module.

---

## Module 1: ROS 2

### Conceptual Questions

1.  **What is the primary role of the DDS (Data Distribution Service) in ROS 2?**
    a) To provide a standard way to build user interfaces.
    b) To manage the underlying operating system.
    c) To handle real-time, decentralized communication between nodes.
    d) To compile C++ and Python code.

2.  **You want to send a command to a robot arm to move to a specific position and need to wait for a confirmation that the action is complete. Which ROS 2 communication mechanism is most appropriate?**
    a) Topic (Publish/Subscribe)
    b) Service (Request/Response)
    c) Action (Asynchronous Goal-Based Tasks)
    d) Parameter

3.  **What is the purpose of the `robot_state_publisher` node?**
    a) To publish the robot's battery level.
    b) To take `/joint_states` messages and a URDF model and publish the `tf2` transforms for the robot's links.
    c) To control the motors of the robot directly.
    d) To generate random motion commands for testing.

4.  **In a Python launch file, what is the `Node` action used for?**
    a) To define the physical structure of a robot.
    b) To specify the dependencies of a package.
    c) To start a single ROS 2 executable (a node).
    d) To write a message to the `/rosout` topic.

### Practical Exercises

1.  **Create a ROS 2 Package:**
    Create a new ROS 2 Python package named `my_checker`. Add a node that creates a publisher on a topic named `/check_topic` and a subscriber on the same topic. The publisher should send a `std_msgs/String` message with the content "Health check OK" once per second. The subscriber should log the received message to the console. Create a launch file that starts your node.

2.  **Custom Service:**
    Define a custom service named `SetBool.srv` that takes a boolean `data` in the request and returns a string `message` in the response. Create a service server that implements this service. If the request is `true`, the server should return the message "State set to TRUE". If `false`, it should return "State set to FALSE". Call this service from the command line.

3.  **XACRO and RViz:**
    Create a XACRO file for a simple mobile robot with a chassis, four wheels, and a "head" link on top with a camera. The wheels should be attached with `continuous` joints. Create a launch file that starts `robot_state_publisher`, `joint_state_publisher_gui`, and RViz2 to visualize and control the robot model.

---

## Module 2: Digital Twin

### Conceptual Questions

1.  **What is the primary advantage of using Gazebo for robotics simulation compared to a game engine like Unity?**
    a) Better graphics and visual effects.
    b) A larger asset store for 3D models.
    c) More accurate and tunable physics simulation.
    d) Easier to use for beginners.

2.  **In a URDF file, what is the difference between the `<visual>` and `<collision>` tags for a link?**
    a) There is no difference; they are interchangeable.
    b) `<visual>` is for how the link looks in RViz, while `<collision>` defines its geometry for the physics engine.
    c) `<visual>` is for 2D models, while `<collision>` is for 3D models.
    d) `<visual>` defines the color, while `<collision>` defines the mass.

3.  **What is the role of the `ros_gz_bridge`?**
    a) It builds a physical bridge in the Gazebo world.
    b) It translates messages between the ROS 2 network and the Gazebo transport network.
    c) It compiles C++ code for Gazebo plugins.
    d) It is a tool for debugging ROS 2 nodes.

4.  **What is the main purpose of using the Unity Robotics Hub?**
    a) To provide physics simulation for robotics.
    b) To allow Unity projects to communicate with a ROS 2 system over a TCP connection.
    c) To automatically generate C# code from Python.
    d) To replace the need for a ROS 2 installation.

### Practical Exercises

1.  **Simulated Sensor:**
    Add a simulated camera to the robot model you created in the Module 1 exercise. Use a `<gazebo>` tag and the appropriate plugin to make the camera publish `sensor_msgs/Image` messages to a ROS 2 topic. Verify that you can see the simulated camera feed in RViz2.

2.  **`ros2_control` Setup:**
    For the wheeled robot model, add the `gz_ros2_control` plugin to your URDF. Create a `controllers.yaml` file that defines a `joint_state_broadcaster` and a `diff_drive_controller`. Write a launch file that spawns the robot in Gazebo and starts the `ros2_control` nodes. Test your setup by publishing a `geometry_msgs/Twist` message to the controller's command topic and verify that the robot moves in Gazebo.

---

## Module 3: NVIDIA Isaac Sim

### Conceptual Questions

1.  **What is the most significant advantage of Isaac Sim for AI and Reinforcement Learning?**
    a) It is the easiest simulator to learn.
    b) It has the most accurate physics engine for all scenarios.
    c) It can run thousands of parallel simulations on a single GPU.
    d) It is free and open source.

2.  **What is Domain Randomization and why is it a critical technique for sim-to-real transfer?**
    a) It is the process of making the simulation look as visually pleasing as possible.
    b) It involves varying the simulation's physical and visual properties during training to force a policy to be more robust.
    c) It is a way to organize files in a project.
    d) It is a technique for identifying the physical parameters of a real robot.

3.  **What is the primary role of the Isaac ROS `isaac_ros_dnn_inference` node?**
    a) To train a new deep learning model from scratch.
    b) To run a pre-trained and optimized TensorRT model on a GPU for tasks like object detection.
    c) To generate synthetic training data.
    d) To provide a bridge between ROS 1 and ROS 2.

4.  **In the context of `isaac_ros_visual_slam`, what is the difference between VIO (Visual-Inertial Odometry) and full SLAM?**
    a) VIO uses LiDAR, while SLAM uses cameras.
    b) VIO is for ground robots, while SLAM is for drones.
    c) VIO provides a real-time motion estimate that drifts over time, while full SLAM adds loop closure to correct for drift and create a globally consistent map.
    d) There is no difference.

### Practical Exercises

1.  **Synthetic Data:**
    In Isaac Sim, create a simple Python script that loads a single object (e.g., a cube or sphere). In a loop, randomize the color of the object and the position of a light source in the scene. In each iteration, save a rendered image to disk. This is a basic form of generating a synthetic dataset.

2.  **RL Reward Function:**
    Write a conceptual reward function (as a Python function) for an RL agent learning to balance a humanoid robot on one leg. The function should take the robot's state as input. Include at least three terms in your reward function (e.g., a reward for keeping the torso upright, a penalty for large joint velocities, and a penalty for falling). Justify each term.

---

## Module 4: VLA & Humanoids

### Conceptual Questions

1.  **What is the difference between Forward Kinematics (FK) and Inverse Kinematics (IK)?**
    a) FK is for arms, IK is for legs.
    b) FK calculates the end-effector pose from joint angles; IK calculates the joint angles required to achieve a desired end-effector pose.
    c) FK is used in simulation, IK is used on the real robot.
    d) FK is a learning-based approach, IK is a model-based approach.

2.  **In the context of LLM Function Calling for robotics, what is the "toolbox"?**
    a) A set of physical tools the robot can use.
    b) A YAML file containing hyperparameters.
    c) A list of Python functions (with descriptions and parameters) that the robot can execute, which is passed to the LLM.
    d) A graphical user interface for controlling the robot.

3.  **Why is a classical, model-based walking controller often considered "brittle" compared to a modern, learning-based one?**
    a) It uses too much electricity.
    b) It is written in C++ instead of Python.
    c) It relies on a simplified model of the robot and environment, and can fail when unmodeled effects (like a nudge or a slippery floor) occur.
    d) It cannot walk backwards.

4.  **In our proposed "Tidy Up the Room" capstone project, what is the role of the VLA/LLM?**
    a) To directly control the motors of the robot.
    b) To act as a cognitive planner: decomposing a high-level user command into a sequence of executable actions from its "toolbox."
    c) To generate the 3D models for the objects in the room.
    d) To turn the robot on and off.

### Practical Exercises

1.  **Grasping Pipeline Logic:**
    Write a short Python script that outlines the *logic* of a classical grasping pipeline. The script should have functions with clear names (e.g., `get_object_pose`, `plan_grasp`, `plan_arm_motion`, `execute_motion`). The functions don't need to do anything, but they should be called in the correct order. The script should take a target object name as input and print out the steps it is performing.

2.  **LLM Prompt Engineering:**
    You have a robot in a living room. It can see a "blue cup" on a "coffee table" and a "book" on a "bookshelf." The robot can navigate and grasp. A user says, "Get me the thing on the table." Write the full prompt that the `robot_brain` node would construct to send to the LLM. Be sure to include a system preamble, the robot's capabilities, the world state, and the user's utterance.
