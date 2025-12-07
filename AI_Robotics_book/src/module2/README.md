# Module 2 Code Examples

This directory contains Python code examples for Module 2: The Digital Twin (Gazebo & Unity).

## Files

-   `spawn_simple_model.py`: A ROS 2 node to spawn a simple box model into Gazebo.
-   `joint_controller.py`: A ROS 2 node to control the joints of a simulated robot in Gazebo.
-   `ros_unity_bridge.py`: A simple Python script to illustrate streaming ROS 2 data (e.g., joint states) to Unity (conceptual, as Unity integration is primarily handled via ROS-TCP-Connector).

## How to Run

1.  Ensure you have a ROS 2 workspace (`ros2_ws`) and your `src/module2` examples are part of a ROS 2 package (e.g., `my_robot_simulation_pkg`).
2.  Build your workspace: `colcon build`.
3.  Source your workspace: `source install/setup.bash`.
4.  Ensure Gazebo is running with a suitable world.

### Spawning a Simple Model

```bash
ros2 run my_robot_simulation_pkg spawn_simple_model.py
```

### Controlling Robot Joints (Requires a robot spawned in Gazebo)

```bash
ros2 run my_robot_simulation_pkg joint_controller.py
```

### ROS-Unity Bridge (Conceptual)

This example illustrates the data flow. Actual Unity integration uses the Unity ROS-TCP-Connector.
```bash
ros2 run my_robot_simulation_pkg ros_unity_bridge.py
```
*(This script would typically publish data that Unity's ROS-TCP-Connector subscribes to.)*
