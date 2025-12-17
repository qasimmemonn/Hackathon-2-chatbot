---
sidebar_position: 2
---

# Chapter 2: Bipedal Locomotion and Balance Control

Walking on two legs is the defining capability of a humanoid robot. It is also, by far, its greatest challenge. Bipedal locomotion is not a state of stability, but rather a dynamic process of continuously catching oneself from a fall. Mastering this "controlled falling" is the key to creating a robot that can navigate human-centric environments.

This chapter explores two primary approaches to this problem: the classic, model-based engineering approach, and the modern, AI-driven learning approach.

## The Core Problem: Controlled Falling

A robot is stable as long as its **Center of Mass (CoM)** is projected vertically within its **Support Polygon**—the area on the ground formed by its feet. For a robot standing still, this is easy. But the moment it lifts a foot to take a step, the support polygon shrinks to the size of a single foot, and the robot immediately begins to fall.

The entire goal of a locomotion controller is to command the robot's joints to:
1.  Swing the free leg to a new position to "catch" the fall.
2.  Simultaneously shift the robot's upper body to move the Center of Mass over the new support polygon.
3.  Repeat this process for every step.

## Method 1: The Classical "Model-Based" Approach

The traditional method for achieving bipedal locomotion relies on creating a simplified but accurate physical model of the robot and using it to plan a stable trajectory.

### The Zero-Moment Point (ZMP)

The key concept in this approach is the **Zero-Moment Point (ZMP)**. This is the point on the ground where the total "tipping over" moment, created by gravity and the robot's own acceleration, is zero. If the ZMP stays within the support polygon, the robot will not fall over.

The classical control strategy is therefore: **plan a motion for the robot's body such that the resulting ZMP is always in a stable location.**

### The Inverted Pendulum Model

To make this planning tractable in real-time, the complex dynamics of the full humanoid are simplified. The most common simplification is the **Linear Inverted Pendulum Model (LIPM)**, which models the robot as a single point mass (the CoM) at a fixed height, connected to the ground by a massless leg. This model allows engineers to mathematically derive a stable trajectory for the CoM that guarantees the ZMP will remain within the support polygon.

### The Classical Pipeline
This leads to a multi-stage pipeline for walking:

![Classical Control Pipeline](https://i.imgur.com/kS9DqM4.png)

1.  **Gait Planning (Offline):** A trajectory planner generates a sequence of desired footstep locations and a corresponding CoM trajectory based on the LIPM.
2.  **Inverse Kinematics (Online):** An IK solver calculates the joint angles needed for the legs and body to achieve the planned footsteps and CoM trajectory.
3.  **Whole-Body Control (Online):** A low-level controller calculates the motor torques required to track the desired joint angles, often using feedback from an IMU and foot force sensors to make minor corrections and reject small disturbances.

**Pros:** This method is well-understood, mathematically sound, and can produce stable walking gaits.
**Cons:** It is brittle. Since it relies on a simplified model, it struggles with unmodeled effects like uneven ground, external pushes, or inaccuracies in the robot's physical properties. The resulting motion often appears stiff and "robotic."

## Method 2: The Modern "Learning-Based" Approach

The modern paradigm, enabled by GPU-accelerated simulators like Isaac Sim, throws away the simplified models. Instead, it uses **Reinforcement Learning (RL)** to allow the robot to *discover* a walking strategy on its own.

The goal is no longer to track a pre-computed trajectory, but to train a neural network **policy** that directly maps the robot's sensor observations to motor commands in order to achieve a goal.

### RL Formulation for Bipedal Locomotion

As we saw with the "Ant" robot in Module 3, we must define the learning problem:

-   **Goal:** Walk forward at a target velocity without falling.
-   **Observation Space (What the robot senses):**
    -   Positions and velocities of all joints.
    -   Orientation and angular velocity of the torso (from an IMU).
    -   A high-level command, e.g., target velocity (vx, vy, vω).
    -   The actions taken in the previous timestep.
    -   Phase information about the gait cycle (e.g., which leg is swinging).
-   **Action Space (What the robot does):**
    -   The target position for each of the leg and torso joints.
-   **Reward Function (The "score" to be maximized):**
    -   `+ Large positive reward` for moving at the target velocity.
    -   `+ Small positive reward` for being "alive" (not falling).
    -   `- Penalty` for excessive joint torque (to encourage energy efficiency).
    -   `- Penalty` for lateral deviation from the target direction.
    -   `- Penalty` for foot impacts that are too hard or feet scuffing the ground.
    -   `- Large negative penalty` for falling (terminating the episode).

### The Power of Domain-Randomized Training

This policy is trained for billions of steps in a **domain-randomized** Isaac Sim environment. In each of the thousands of parallel simulations, the physics of the world and the robot are slightly different:
-   The robot's mass and center of mass are varied.
-   The ground friction is changed.
-   The motor strength and joint friction are randomized.
-   Random forces ("pushes") are applied to the robot's torso.

A policy that learns to walk successfully across all these variations becomes incredibly robust. It doesn't rely on a single, perfect model of physics. Instead, it learns a general strategy that is resilient to the inevitable gap between simulation and reality.

**Pros:** Produces highly robust, dynamic, and natural-looking gaits. Can learn to handle varied terrain and external disturbances without being explicitly programmed to do so.
**Cons:** Requires massive computational resources for training (a high-end GPU and a simulator like Isaac Sim are essential). The resulting "black box" policy can be difficult to analyze and debug.

## The Future: Hybrid Approaches

The state-of-the-art is often a hybrid of these two methods. For instance, an RL policy might be trained to generate desired footstep locations and CoM goals in a reactive and dynamic way, while a classical whole-body controller uses its model-based knowledge to execute those goals while rigorously respecting joint limits and other physical constraints.

With a robust locomotion controller, our humanoid can now move purposefully through its environment. The next step is to give it hands and the ability to interact with objects, which is the focus of the following chapter on manipulation.