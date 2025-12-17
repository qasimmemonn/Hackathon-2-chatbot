---
sidebar_position: 99
title: Glossary
---

# Glossary

This glossary defines key terms and acronyms used throughout the Physical AI & Humanoid Robotics textbook.

---

### A

**Action (ROS 2)**
: A communication pattern for long-running, asynchronous tasks that provide feedback. An Action consists of a goal, feedback, and a result, making it ideal for tasks like navigation or manipulation.

**ament**
: The build system used by ROS 2. `ament_python` is used for Python packages and `ament_cmake` is used for C++ packages.

**ArticulationBody**
: The component used in Unity to define the physics properties of a robot's joints and links, enabling simulation of complex, articulated bodies like a humanoid.

### C

**Center of Mass (CoM)**
: The average location of the mass of an object (or robot). In bipedal locomotion, controlling the position of the CoM relative to the support polygon is the primary goal for maintaining balance.

**colcon**
: The command-line tool used to build and install ROS 2 packages in a workspace.

### D

**DDS (Data Distribution Service)**
: The industry-standard, publish-subscribe middleware that provides the underlying communication layer for ROS 2, enabling real-time, scalable, and secure data exchange.

**Digital Twin**
: A high-fidelity virtual model of a physical object or system. A digital twin is used to simulate, test, and validate a system's behavior before and during its operational life.

**Domain Randomization (DR)**
: A sim-to-real transfer technique where physical and visual properties of the simulation (e.g., friction, mass, lighting, textures) are randomized during AI training. This forces the learned policy to be more robust and generalize better to real-world conditions.

**Dynamics**
: The study of motion in relation to the forces, torques, and masses that cause it. Dynamics is essential for calculating motor commands and simulating realistic robot behavior.

### E

**End-Effector**
: The device at the end of a robotic arm, such as a gripper, hand, or tool.

### F

**Forward Kinematics (FK)**
: The process of calculating the position and orientation of a robot's end-effector from its joint angles.

**Function Calling (LLM)**
: A feature of modern Large Language Models that allows them to request the execution of a specific function (a "tool") with certain parameters, based on a user's prompt. This is the key to converting natural language commands into robot actions.

### G

**Gazebo**
: A popular, open-source 3D robotics simulator with a focus on high-fidelity physics simulation and tight integration with ROS.

### I

**Inverse Kinematics (IK)**
: The process of calculating the required robot joint angles to place an end-effector at a desired position and orientation.

**Isaac Sim**
: A photorealistic, physically-accurate robotics simulation platform from NVIDIA, built on the Omniverse platform. It uses GPU acceleration to enable large-scale parallel simulation for AI and reinforcement learning tasks.

### J

**Jacobian**
: A matrix of partial derivatives that relates the velocities of a robot's joints to the linear and angular velocity of its end-effector. It is a key mathematical tool used in numerical Inverse Kinematics solvers and dynamics.

**Joint**
: A component that connects two robot links and defines the motion between them (e.g., revolute, prismatic, fixed).

### K

**Kinematics**
: The study of motion without considering the forces and masses that cause it. It is the geometry of motion.

### L

**Launch File**
: In ROS 2, a Python script used to start and configure a collection of nodes, their parameters, and their communication settings.

**Link**
: A rigid body part of a robot that is connected to other links by joints.

**LLM (Large Language Model)**
: A large-scale neural network (e.g., GPT-4) trained on vast amounts of text data, capable of understanding and generating human-like language. In robotics, it is used as a cognitive planner and conversational brain.

### M

**Manipulation**
: The act of a robot interacting with objects in its environment, such as picking, placing, pushing, or using tools.

**Message (ROS 2)**
: A simple data structure (e.g., `String`, `PoseStamped`) used for sending data over ROS 2 topics.

**MoveIt2**
: The standard, state-of-the-art software for motion planning, manipulation, and inverse kinematics in ROS 2.

### N

**Nav2**
: The standard, state-of-the-art software for autonomous navigation and localization in ROS 2.

**Node (ROS 2)**
: The fundamental unit of execution in ROS 2. A node is a single process that performs a specific task.

**NVIDIA Omniverse**
: A scalable, multi-GPU platform for 3D simulation and design collaboration, built around the USD file format. Isaac Sim is an application built on Omniverse.

### P

**Package (ROS 2)**
: The fundamental unit of software organization in ROS 2. A package is a directory containing nodes, launch files, models, and a `package.xml` manifest.

**Parameter (ROS 2)**
: A configurable value (e.g., a robot's speed, a topic name) that can be set at runtime to change a node's behavior without recompiling code.

**PhysX**
: NVIDIA's high-performance, GPU-accelerated physics engine, used by Isaac Sim to simulate dynamics and contact physics.

**Policy (RL)**
: In reinforcement learning, the policy is the "brain" of the agent. It is typically a neural network that maps observations of the environment to actions.

**Prompt Engineering**
: The art of carefully constructing the input text (the prompt) given to a Large Language Model to elicit a desired, accurate, and relevant response.

### Q

**Quality of Service (QoS)**
: In ROS 2, QoS settings give fine-grained control over the behavior of communication, such as reliability (guaranteed delivery vs. best-effort) and durability (how messages are handled for late-joining nodes).

### R

**Reinforcement Learning (RL)**
: A type of machine learning where an agent learns to make decisions by performing actions in an environment to maximize a cumulative reward signal.

**Reward Function**
: In reinforcement learning, a function that provides a scalar feedback signal (the "reward" or "penalty") to the agent based on its actions and the resulting state. The agent's goal is to learn a policy that maximizes the total reward.

**ROS (Robot Operating System)**
: A flexible framework for writing robot software. It is a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior.

**`ros2_control`**
: A standard framework in ROS 2 that provides a hardware abstraction layer for controllers. It allows the same high-level controllers to run on both simulated and real robots by simply swapping out the low-level hardware interface.

**RViz**
: The primary 3D visualization tool for ROS. It is used to display sensor data, robot models, and algorithm outputs.

### S

**SDF (Simulation Description Format)**
: The native XML-based format used by Gazebo to describe robots and simulation environments. It is a superset of URDF.

**Service (ROS 2)**
: A communication pattern for synchronous, request-response interactions between two nodes.

**Sim-to-Real Transfer**
: The process of taking a policy or model trained in a simulated environment and deploying it on a physical robot, and the set of techniques used to bridge the "reality gap."

**SLAM (Simultaneous Localization and Mapping)**
: The problem of a robot building a map of an unknown environment while simultaneously tracking its own position within that map.

**Support Polygon**
: The area on the ground formed by the convex hull of a robot's contact points (e.g., its feet). For a bipedal robot to be stable, its Center of Mass must be kept above this area.

**Synthetic Data**
: Artificial data generated in a simulation. In robotics, this often refers to photorealistic images, depth maps, and segmentation masks generated in Isaac Sim for the purpose of training AI perception models.

**System Identification**
: The process of building a mathematical model of a physical system by measuring its properties from experimental data. This is used to make a simulation more accurately reflect a real robot.

### T

**TF2**
: The transformation library in ROS 2. It manages the tree of coordinate frames, allowing you to ask for the transform between any two frames at any time.

**Topic (ROS 2)**
: A named bus over which nodes exchange messages in a publish-subscribe fashion. It is the primary, one-to-many communication method in ROS.

### U

**URDF (Unified Robot Description Format)**
: An XML-based file format used in ROS to describe the kinematic and dynamic properties of a robot, as well as its visual appearance.

**USD (Universal Scene Description)**
: A framework for interchanging 3D computer graphics data, developed by Pixar. It is the core file format for the NVIDIA Omniverse platform and Isaac Sim.

### V

**VLA (Vision-Language-Action) Model**
: A type of large, multi-modal AI model that takes vision (images) and language (text) as input and produces an action as output. These are at the forefront of modern robotics research.

**VIO (Visual-Inertial Odometry)**
: A state estimation technique that fuses data from a camera and an IMU to provide a real-time estimate of a robot's motion. It provides the `odom` -> `base_link` transform but is subject to drift.

### W

**Workspace (ROS 2)**
: A directory containing ROS 2 packages, along with `src`, `build`, `install`, and `log` sub-directories.

### X

**XACRO (XML Macros)**
: A macro language used to make URDF files more readable and modular by using variables, math, and macros.

### Z

**ZMP (Zero-Moment Point)**
: In bipedal locomotion, the ZMP is the point on the ground where the net moment from inertial and gravitational forces is zero. In classical control, keeping the ZMP inside the support polygon ensures the robot will not tip over.