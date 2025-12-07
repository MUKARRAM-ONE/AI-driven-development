# Module 3 Code Examples

This directory contains Python code examples for Module 3: The AI-Robot Brain (NVIDIA Isaac).

## Files

-   `isaac_sim_hello_world.py`: A basic script to launch Isaac Sim and load a simple scene/robot. (This is a conceptual Python script as direct Isaac Sim scripting can be more complex).
-   `isaac_ros_vslam_example.py`: A conceptual ROS 2 node that would interact with an Isaac ROS vSLAM pipeline. (Actual implementation would involve running Isaac ROS containers).
-   `nav2_goal_publisher.py`: A ROS 2 node to publish a navigation goal for Nav2.

## How to Run

1.  Ensure you have a ROS 2 workspace (`ros2_ws`) and your `src/module3` examples are part of a ROS 2 package (e.g., `my_robot_isaac_pkg`).
2.  Build your workspace: `colcon build`.
3.  Source your workspace: `source install/setup.bash`.
4.  Ensure Isaac Sim is running (for `isaac_sim_hello_world.py`).
5.  Ensure Isaac ROS containers are running (for `isaac_ros_vslam_example.py`).
6.  Ensure Nav2 stack is configured and running (for `nav2_goal_publisher.py`).

### Launching Isaac Sim (Conceptual)

```bash
# This is a conceptual example. Actual Isaac Sim scripting
# involves specific Omniverse Kit environments.
python3 isaac_sim_hello_world.py
```

### Interacting with Isaac ROS vSLAM (Conceptual)

```bash
ros2 run my_robot_isaac_pkg isaac_ros_vslam_example.py
# This script would typically subscribe to /tf or /visual_slam/tracking/odometry
```

### Publishing Nav2 Goal

```bash
ros2 run my_robot_isaac_pkg nav2_goal_publisher.py
```
