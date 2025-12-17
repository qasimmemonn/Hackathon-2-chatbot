---
sidebar_position: 4
---

# Chapter 4: Digital Twin Integration and Testing

Throughout this module, we've built the individual components of our digital twin: a robot model enhanced for simulation, a physics environment in Gazebo, a visualization front-end in Unity, and a suite of simulated sensors. This final chapter is where we bring it all together. Our goal is to create a single, cohesive system that starts our entire simulated world and robot with one command.

This integrated digital twin is the foundation for everything that follows. It's the virtual sandbox where we will develop and test the advanced AI, control, and perception algorithms in the rest of this book.

## The Goal: A Single Launch Command

A well-structured ROS 2 project should be easy to run. We will create a master launch file that encapsulates all the complexity of starting our simulation. The final result will be a single command:

`ros2 launch my_robot_bringup start_sim.launch.py`

This command will:
1.  Start the Gazebo simulator with a specified world.
2.  Spawn our humanoid robot model into the simulation.
3.  Start all necessary ROS 2 bridges and controllers.
4.  Launch visualization tools like RViz2.

To achieve this, we'll follow a best practice: creating a dedicated "bringup" package.

## The Bringup Package
This package (e.g., `my_robot_bringup`) typically doesn't contain source code. Its purpose is to hold the top-level launch files and configuration files needed to start the robot in its various modes (simulation, physical hardware, etc.). This separates the robot's description (`my_robot_description`) and control configuration (`my_robot_control`) from the high-level orchestration.

## The Master Simulation Launch File

Let's build the `start_sim.launch.py` file step-by-step. It will live in the `launch/` directory of our `my_robot_bringup` package.

**`my_robot_bringup/launch/start_sim.launch.py`**
```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    # 1. DEFINE LAUNCH ARGUMENTS
    # ============================
    # Allows us to change the world file or RViz config from the command line
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([
            get_package_share_directory('my_robot_worlds'), 'worlds', 'empty.world'
        ]),
        description='Path to the Gazebo world file'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Whether to start RViz2'
    )

    # 2. GET FILE PATHS
    # =================
    # Path to this package
    pkg_bringup = get_package_share_directory('my_robot_bringup')
    # Path to the robot description package
    pkg_description = get_package_share_directory('my_robot_description')
    # Path to the RViz configuration
    rviz_config_path = PathJoinSubstitution([pkg_bringup, 'rviz', 'sim.rviz'])
    # Path to the robot's URDF file (XACRO)
    urdf_path = PathJoinSubstitution([pkg_description, 'urdf', 'humanoid.urdf.xacro'])


    # 3. LOAD THE ROBOT DESCRIPTION
    # =============================
    robot_description_content = LaunchConfiguration(
        'robot_description',
        default=urdf_path
    )
    robot_description = {'robot_description': robot_description_content}


    # 4. START GAZEBO SIMULATOR
    # ===========================
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', LaunchConfiguration('world')]}.items()
    )


    # 5. SPAWN THE ROBOT IN GAZEBO
    # ============================
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'humanoid',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )


    # 6. START ROS-GAZEBO BRIDGES
    # ===========================
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # Clock (Gazebo -> ROS 2)
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # Joint States (Gazebo -> ROS 2)
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Camera (Gazebo -> ROS 2)
            '/head_camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/head_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            # LiDAR (Gazebo -> ROS 2)
            '/torso_lidar/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
        ],
        remappings=[
            ('/joint_states', '/joint_states_gz'), # Remap to avoid conflict with ros2_control
        ],
        output='screen'
    )


    # 7. START `robot_state_publisher`
    # ===============================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}]
    )


    # 8. START `ros2_control`
    # =======================
    # Starts the controller_manager and spawns the controllers
    ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'ros2_control.launch.py')
        ),
        launch_arguments={
            'robot_description': robot_description_content
        }.items()
    )


    # 9. START RVIZ2
    # ==============
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz')),
        output='screen'
    )


    # LAUNCH DESCRIPTION
    # ==================
    return LaunchDescription([
        world_arg,
        rviz_arg,
        gz_sim,
        spawn_robot,
        bridge,
        robot_state_publisher,
        ros2_control,
        rviz
    ])
```

## Testing and Validating the Integrated Twin

Once you run the launch file, your entire digital twin should be running. Now, you must validate that all the components are working together correctly. Open new terminals (and `source install/setup.bash` in each one) to perform these checks.

### Check 1: Visualization and TF Tree
-   **Gazebo:** Do you see your robot in the Gazebo world?
-   **RViz2:** Does the robot model appear? Is it in the correct pose?
-   **TF Tree:** The `robot_state_publisher` and `joint_state_broadcaster` should be publishing the transform tree.
    ```bash
    # Check if the transform from the world to the robot's foot is available
    ros2 run tf2_ros tf2_echo odom base_link
    ```
    If this command prints a continuous stream of transforms, your TF tree is alive.

### Check 2: Sensor Data
Verify that sensor data is flowing from Gazebo, through the `ros_gz_bridge`, to the ROS 2 ecosystem.
```bash
# Check for camera images
ros2 topic hz /head_camera/image_raw

# Check for LiDAR point clouds
ros2 topic hz /torso_lidar/points
```
If you see a steady stream of messages, your sensors are working. You should also visualize this data in RViz2 to confirm it looks correct.

### Check 3: Joint Control
This is the final and most important test: can you command the robot's joints? We can use the `ros2 action` command to send a goal to the `joint_trajectory_controller`.

```bash
# Send a goal to move the 'arm_elbow_joint' to 1.0 radian over 2 seconds
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory trajectory_msgs/action/FollowJointTrajectory '{
  "trajectory": {
    "joint_names": ["arm_elbow_joint"],
    "points": [
      {
        "positions": [1.0],
        "time_from_start": {"sec": 2, "nanosec": 0}
      }
    ]
  }
}'
```
Watch the robot in both Gazebo and RViz2. The arm should smoothly move to the commanded position. If it does, your digital twin is fully integrated and ready for development.

This concludes Module 2. You have successfully built and validated a complex, multi-component digital twin of a humanoid robot—an essential tool that will serve as your development platform for the rest of this book. You are now prepared to tackle Module 3, where we will give our robot the ability to see and understand its world using AI-based perception.