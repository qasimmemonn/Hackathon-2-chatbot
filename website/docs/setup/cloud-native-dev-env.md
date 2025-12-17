---
sidebar_position: 3
---

# Guide: Cloud-Native Development Environment

A cloud-native workflow is a powerful strategy for modern robotics development. It provides a consistent, scalable, and collaborative environment, freeing you from machine-specific setup issues. This guide details how to create a professional development environment using Docker and GitHub Codespaces.

This approach ensures that every developer—and every CI/CD pipeline—uses the exact same toolchain, eliminating the classic "it works on my machine" problem.

## Why Go Cloud-Native?

-   **Consistency & Reproducibility**: Define your entire toolchain as code. Every team member gets an identical environment with the click of a button.
-   **Scalability on Demand**: Tap into powerful cloud servers for computationally intensive tasks like compiling large codebases or running complex simulations, without needing expensive local hardware.
-   **Seamless Collaboration**: Share your exact development session with a teammate via a simple URL for easier debugging and pair programming.
-   **CI/CD Integration**: The same container definition used for development can be used in your automated testing and deployment pipelines, ensuring consistency from development to production.

## Core Technologies

-   **Docker**: The cornerstone of our environment. Docker packages our entire toolchain—OS, compilers, libraries, and dependencies—into a portable, isolated container.
-   **Visual Studio Code**: The de-facto standard for remote development, providing a rich, local-like experience while connected to a remote server or container.
-   **GitHub Codespaces**: A service that hosts Docker-based development environments directly within GitHub, fully managed and integrated with your repositories.

## Setting Up Your Cloud Environment

### Step 1: The Devcontainer Configuration

The magic begins with a `.devcontainer` directory in your repository. This directory tells VS Code and GitHub Codespaces everything they need to know to build your environment.

First, create a `.devcontainer` folder in the root of your project. Inside this folder, create a file named `devcontainer.json`.

####  `.devcontainer/devcontainer.json`

This file is the blueprint for your codespace. It defines the Docker image to use, specifies which VS Code extensions to install, and sets up required runtime arguments.

```json title=".devcontainer/devcontainer.json"
{
  "name": "Physical AI & Robotics Dev Environment",
  "build": {
    "dockerfile": "Dockerfile"
  },
  "customizations": {
    "vscode": {
      "settings": {
        "python.defaultInterpreterPath": "/usr/bin/python3"
      },
      "extensions": [
        "ms-vscode.cpptools-extension-pack",
        "ms-python.python",
        "ms-iot.vscode-ros",
        "dotjoshjohnson.xml",
        "redhat.vscode-yaml"
      ]
    }
  },
  "runArgs": [
    "--gpus=all",
    "--network=host"
  ],
  "remoteUser": "dev"
}
```

-   **`"runArgs"`**: This is critical.
    -   `"--gpus=all"` passes GPU hardware from the host machine into the container, which is essential for accelerating AI workloads and simulations like NVIDIA Isaac Sim.
    -   `"--network=host"` allows services inside the container (like a ROS 2 node) to be accessed directly from your local machine, simplifying network configuration.

### Step 2: The Dockerfile

Next, create the `Dockerfile` that `devcontainer.json` references. This file defines the step-by-step process for building your development environment from a base image.

####  `.devcontainer/Dockerfile`

This Dockerfile sets up a complete ROS 2, and basic development environment on an Ubuntu base.

```dockerfile title=".devcontainer/Dockerfile"
# Start from the official NVIDIA CUDA base image to ensure GPU compatibility
FROM nvidia/cuda:12.1.1-devel-ubuntu22.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install essential development tools and utilities
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    curl \
    wget \
    lsb-release \
    && rm -rf /var/lib/apt/lists/*

# Install ROS 2 Humble from the official ROS repositories
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get install -y \
    ros-humble-desktop-full \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Source ROS 2 automatically in new shell sessions
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc

# Set up a non-root user for better security
ARG USERNAME=dev
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=(root) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non-root user
USER $USERNAME

# Set the working directory for the project
WORKDIR /workspace
```

### Step 3: Launch Your Environment

With the `.devcontainer/devcontainer.json` and `.devcontainer/Dockerfile` committed to your repository, you are ready to launch.

1.  **Navigate to your repository on GitHub.com.**
2.  Click the green **`< > Code`** button.
3.  Select the **Codespaces** tab.
4.  Click **Create codespace on main**.

GitHub will now build your container and connect you to a full-featured VS Code instance in your browser. Your repository code will be mounted in the `/workspace` directory, and the terminal will be connected to your new, fully configured cloud environment.

## Local Development with Devcontainers

You can use the same devcontainer configuration for local development. This gives you the same consistent environment without relying on a cloud service.

### Prerequisites

-   [Visual Studio Code](https://code.visualstudio.com/)
-   [Docker Desktop](https://www.docker.com/products/docker-desktop/) or Docker Engine
-   The [Dev Containers extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) for VS Code

### Steps

1.  **Clone your repository** to your local machine.
2.  **Open the repository folder** in VS Code.
3.  **VS Code will detect the `.devcontainer` folder** and show a notification in the bottom-right corner: "Reopen in Container".
4.  **Click "Reopen in Container"**. VS Code will build the Docker image and start the container.

You are now running the exact same development environment on your local machine.

## Next Steps

Your cloud-native robotics lab is now open! Here are a few things you can do to get started:

-   **Open a terminal** in VS Code (Ctrl+\`).
-   **Verify your ROS 2 installation** by running `ros2 --help`.
-   **Create your first ROS 2 workspace** and package.
-   **Start building your robot!**
