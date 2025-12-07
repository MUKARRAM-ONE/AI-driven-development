# Capstone Project: Autonomous Humanoid Robot

This directory contains the code and assets for the Capstone Project, which integrates all concepts from the Physical AI & Humanoid Robotics Textbook Module.

## Objective

To create an autonomous humanoid robot in a simulated environment that can:
1.  Receive a voice command.
2.  Plan a path and navigate obstacles.
3.  Identify an object.
4.  Manipulate the object.

## Files

-   `capstone_main.py`: The main orchestrator node for the capstone project, integrating VLA, navigation, perception, and manipulation.
-   `humanoid_robot.urdf`: A placeholder URDF for the humanoid robot used in the capstone simulation.
-   `capstone_launch.launch.py`: A launch file to bring up all capstone components in simulation.

## Conceptual Flow

1.  **Voice Input**: `voice_command_transcriber.py` (from Module 4)
2.  **Cognitive Planning**: `cognitive_planner_node.py` (from Module 4) generates action sequences.
3.  **Action Execution**: `action_executor_node.py` (from Module 4) interprets and executes action sequences.
4.  **Navigation**: Interacts with Nav2 stack (from Module 3) using goals from the action executor.
5.  **Perception**: Uses Isaac ROS (from Module 3) for object detection.
6.  **Manipulation**: Direct joint control (from Module 1) or a simple manipulation controller.
7.  **Simulation**: Isaac Sim (from Module 3) provides the environment and robot model.

## How to Run

1.  Ensure all previous module examples and dependencies are correctly set up and built in your ROS 2 workspace.
2.  Build your workspace: `colcon build`.
3.  Source your workspace: `source install/setup.bash`.
4.  Launch Isaac Sim with your humanoid robot in a suitable environment.
5.  Launch the capstone project: `ros2 launch my_capstone_pkg capstone_launch.launch.py` (assuming `my_capstone_pkg` is your package name).
6.  Provide voice commands to your microphone and observe the robot's behavior in simulation.
