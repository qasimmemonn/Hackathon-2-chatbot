---
sidebar_position: 2
---

# Guide: Physical AI Edge Kit

Simulation is where we iterate, but the real world is where our robots must prove their intelligence. This guide specifies the components and setup for a **Physical AI Edge Kit**—a reference hardware platform for building and deploying real-world embodied AI agents.

This kit is designed as a versatile, cost-effective starting point for a variety of mobile robots, including a small-scale humanoid.

## Reference Hardware Components

This is a reference design. We encourage you to substitute components based on availability, budget, and your project's specific goals. The principles of a distributed compute architecture (high-level compute + real-time control) remain the same.

| Component              | Example                                       | Rationale                                                                                             |
| ---------------------- | --------------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| **Compute Board**      | NVIDIA Jetson Orin Nano Developer Kit         | A powerhouse for its size. Provides a strong GPU for running AI models for perception and navigation directly on the robot.       |
| **Microcontroller**    | Teensy 4.1 or Raspberry Pi Pico W             | Offloads hard real-time tasks like motor control and sensor readings from the main computer, ensuring stability and low-latency response. The Teensy 4.1 is particularly powerful for its size.          |
| **Primary Vision**     | OAK-D Pro W                                   | An all-in-one stereo camera with on-chip depth processing and AI capabilities. Reduces the computational load on the Jetson, freeing it up for higher-level tasks. |
| **360° LiDAR**         | RPLIDAR A1 / SLAMTEC Mapper                   | Provides a 360-degree map of the environment, essential for reliable SLAM (Simultaneous Localization and Mapping) and autonomous navigation.      |
| **IMU**                | BNO085 9-DOF IMU                              | Critical for estimating the robot's orientation (roll, pitch, yaw), which is fundamental for balance, stabilization, and dead reckoning.  |
| **Servo Motors**       | 24x Dynamixel XL-330 (for a small humanoid)   | High-quality, daisy-chainable servos that provide position, velocity, and temperature feedback. The daisy-chaining feature simplifies wiring significantly.       |
| **Power System**       | 12V 5A LiPo Battery + Battery Management System | A properly sized battery and a safety-critical BMS are non-negotiable for mobile robots.            |
| **Frame**              | 3D Printed (e.g., Poppy Humanoid) or Custom   | The robot's skeleton. Many excellent open-source designs are available on platforms like Printables.com. 3D printing allows for rapid prototyping and customization. |

---

## Software Setup for the NVIDIA Jetson

The NVIDIA Jetson will serve as the robot's brain. The setup process involves flashing it with NVIDIA's software stack and then installing ROS 2.

### Step 1: Install NVIDIA JetPack

JetPack is the essential software development kit for the Jetson platform. It includes the Ubuntu-based OS, NVIDIA drivers, and libraries for AI and computer vision (CUDA, cuDNN, TensorRT).

1.  **Get a good microSD card**: You will need a high-quality, U3-rated microSD card (64GB or larger).
2.  **Follow the official guide**: NVIDIA provides an excellent, step-by-step guide for getting started. Follow the official instructions to [flash your Jetson Orin with the latest JetPack SDK](https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit).
3.  **Complete the initial setup**: Once flashed, connect the Jetson to a monitor and keyboard to complete the Ubuntu system setup.

### Step 2: Install ROS 2 Humble

With the Jetson running, we will install ROS 2 Humble. The process is similar to the workstation setup, but we use the `ros-base` package to save space.

#### **1. Set Up Repositories**

First, authorize the ROS GPG key and add the official repository.

```bash
sudo apt-get update
sudo apt-get install -y software-properties-common
sudo add-apt-repository universe
sudo apt-get update && sudo apt-get install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### **2. Install ROS 2 Packages**

Now, install `ros-humble-ros-base`, which is a more minimal installation than `desktop-full`.

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base
```

#### **3. Source the Setup File**

Add the ROS 2 setup script to your `.bashrc` file.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### **4. Verify the Installation**

Run the `talker`/`listener` demo to confirm that ROS 2 is working correctly on your Jetson.

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker

# Terminal 2
ros2 run demo_nodes_py listener
```

### Step 3: Configure micro-ROS

The microcontroller (e.g., a Teensy) will act as a real-time peripheral, communicating with the Jetson via **micro-ROS**. This architecture is critical for robust robotics.

1.  **Set up the micro-ROS build system** on your **Digital Twin Workstation** (not the Jetson) to cross-compile firmware for your microcontroller.
2.  **Install the micro-ROS Agent** on the **Jetson**. This agent acts as the bridge between the microcontroller and the main ROS 2 network.
3.  **Develop and flash the firmware** for the microcontroller. This firmware will be responsible for low-level tasks like reading from the IMU and commanding the servo motors.

This distributed architecture ensures that high-level planning on the Jetson is decoupled from the critical, real-time operations of the robot's body, creating a more stable and reliable system.

For a detailed guide on setting up micro-ROS, refer to the [official micro-ROS documentation](https://micro.ros.org/).

## Power and Safety

**Working with LiPo batteries requires extreme care.** They are powerful but can be dangerous if mishandled.

-   **Always use a Battery Management System (BMS):** A BMS protects the battery from overcharging, over-discharging, and short circuits. **Do not use a LiPo battery without a BMS.**
-   **Use a proper charger:** Only use a charger specifically designed for LiPo batteries.
-   **Never puncture or damage a LiPo battery:** A damaged battery can catch fire.
-   **Store batteries in a fire-safe bag:** When not in use, store your batteries in a LiPo-safe bag.

## Next Steps

Your Physical AI Edge Kit is now ready for the exciting work of bringing your code to life.

-   **Assemble your robot's frame.**
-   **Wire the components:** Connect the motors, sensors, and microcontroller to the Jetson.
-   **Write the microcontroller firmware:** Develop the micro-ROS firmware to control the motors and read the sensors.
-   **Develop the high-level ROS 2 nodes:** Write the ROS 2 nodes on the Jetson to handle perception, navigation, and decision-making.

This is where the journey truly begins!
