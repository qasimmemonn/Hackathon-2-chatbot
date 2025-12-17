---
sidebar_position: 1
---

# Guide: Digital Twin Workstation Setup

A powerful local workstation is the heart of modern robotics R&D. This guide details the hardware and software required to build a **Digital Twin Workstation**, a machine capable of running the entire robotics stack: from real-time physics simulation and high-fidelity rendering to accelerated AI training.

## Hardware Recommendations

Developing for humanoid robots is computationally demanding. While it's possible to get started with less, this recommended setup will ensure a smooth and productive development experience.

| Component          | Recommended Specification                                  | Why It Matters                                                                     |
| ------------------ | ---------------------------------------------------------- | ---------------------------------------------------------------------------------- |
| **Operating System** | Ubuntu 22.04 LTS (Jammy Jellyfish)                         | The primary supported OS for ROS 2 Humble and the NVIDIA robotics stack.             |
| **CPU**            | 12-core/24-thread (e.g., Intel i7-13700K, AMD Ryzen 9 7900X) | Crucial for multi-threaded applications like physics simulation and code compilation. A high core count will significantly speed up your workflows. |
| **GPU**            | NVIDIA RTX 4070 or higher (12GB+ VRAM)                      | **The single most important component.** Required for Isaac Sim's real-time ray tracing and for accelerating AI/ML model training with CUDA. More VRAM allows for more complex scenes and larger models. |
| **RAM**            | 64 GB DDR5                                                 | Large, complex simulations, 3D assets, and large datasets can consume significant memory. 64GB provides a comfortable buffer for demanding tasks.            |
| **Storage**        | 2 TB NVMe SSD                                              | Fast storage is critical for loading large simulation environments and datasets quickly. An NVMe SSD will dramatically reduce load times compared to a traditional HDD or SATA SSD.   |

---

## Software Installation: A Step-by-Step Guide

Follow these steps carefully to configure your workstation.

### Step 1: Install Ubuntu 22.04 LTS

If you don't have it already, a clean installation of Ubuntu 22.04 LTS is the most reliable starting point.

1.  **Download the ISO**: Get the official Ubuntu 22.04 Desktop ISO from the [Ubuntu website](https://ubuntu.com/download/desktop).
2.  **Create a Bootable USB**: Use a tool like [BalenaEtcher](https://www.balena.io/etcher/) to flash the ISO to a USB drive (8GB or larger).
3.  **Install Ubuntu**: Boot from the USB drive and follow the on-screen instructions. We recommend the "Minimal installation" option to keep your OS clean.

### Step 2: Install NVIDIA Drivers

A correct NVIDIA driver setup is **absolutely critical** for robotics development.

#### **1. Purge Old Drivers**

First, remove any existing NVIDIA drivers to prevent conflicts.

```bash
sudo apt-get purge '^nvidia-.*'
sudo apt-get autoremove
sudo apt-get autoclean
```

#### **2. Install the Recommended Driver**

We recommend using the official graphics-drivers PPA for the latest stable drivers.

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt-get update
```

Now, install the recommended `nvidia-driver-535`.

```bash
sudo apt-get install nvidia-driver-535
```

#### **3. Reboot and Verify**

After the installation is complete, reboot your system.

```bash
sudo reboot
```

Once rebooted, open a terminal and run `nvidia-smi` (NVIDIA System Management Interface). If the driver is installed correctly, you will see a table detailing your GPU and the driver version.

```bash
nvidia-smi
```

### Step 3: Install ROS 2 Humble

We will use the `Humble Hawksbill` distribution of ROS 2, the long-term support (LTS) version designed for Ubuntu 22.04.

#### **1. Set Up Repositories**

First, authorize the ROS GPG key and add the official repository to your system.

```bash
sudo apt-get update && sudo apt-get install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### **2. Install ROS 2 Packages**

Now, update your package index and install the `ros-humble-desktop-full` package, which includes ROS, RViz, simulators, and navigation tools.

```bash
sudo apt-get update
sudo apt-get install ros-humble-desktop-full
```

#### **3. Source the Setup File**

Add the ROS 2 setup script to your `.bashrc` file to automatically source it in new terminal sessions.

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### **4. Verify the Installation**

Test your installation by running the `talker`/`listener` demo.

Open two terminals. In the first terminal, run the C++ talker node:

```bash
# Terminal 1
ros2 run demo_nodes_cpp talker
```

In the second terminal, run the Python listener node:

```bash
# Terminal 2
ros2 run demo_nodes_py listener
```

If the installation is successful, you will see the listener terminal printing messages received from the talker.

### Step 4: Install Gazebo Simulator

Gazebo is the most widely-used open-source physics simulator in robotics.

```bash
sudo apt-get update
sudo apt-get install gazebo
```

Verify the installation by launching the simulator.

```bash
gazebo
```

### Step 5: Install NVIDIA Isaac Sim

NVIDIA Isaac Sim is a state-of-the-art, photorealistic robotics simulator built on the Omniverse platform.

1.  **Install Omniverse Launcher**: Download and install the Omniverse Launcher from the [NVIDIA website](https://www.nvidia.com/en-us/omniverse/download/).
2.  **Install Isaac Sim**:
    *   Open the Omniverse Launcher.
    *   Navigate to the **Exchange** tab and search for "Isaac Sim".
    *   Click on Isaac Sim and select the latest recommended version to install.
3.  **Run Post-Installation Setup**: After installation, Isaac Sim will have post-installation steps to install required dependencies. Follow the on-screen prompts carefully.

## Troubleshooting

-   **`nvidia-smi` command not found:** This means the NVIDIA driver installation failed or is not in your `PATH`. Double-check the installation steps and make sure you have rebooted.
-   **ROS 2 command not found:** This means you haven't sourced the `setup.bash` file correctly. Verify that the `source /opt/ros/humble/setup.bash` line is in your `~/.bashrc` file.
-   **Isaac Sim fails to start:** This is often a driver issue. Ensure you have the correct NVIDIA driver installed and that your GPU is supported.

## Next Steps

Your Digital Twin Workstation is now fully configured. Here's what you can do next:

-   **Explore the ROS 2 tutorials:** Get familiar with the core concepts of ROS 2.
-   **Create a simple robot model:** Learn how to create a URDF file to describe your robot.
-   **Simulate your robot in Gazebo:** Spawn your robot in Gazebo and learn how to control it.
-   **Dive into Isaac Sim:** Explore the powerful features of Isaac Sim and learn how to build realistic simulations.

You are now ready to build and simulate advanced humanoid robots.
