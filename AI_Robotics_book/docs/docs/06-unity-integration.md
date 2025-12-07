---
id: 06-unity-integration
title: "Chapter 6: Unity Integration for Robotics"
sidebar_label: "6. Unity Integration for Robotics"
---

## Chapter 6: Unity Integration for Robotics

**Objective**: Use Unity as a high-fidelity visualizer for ROS 2 and Gazebo simulations.

### 6.1 Why Unity for Visualization?

While Gazebo excels at physics simulation and ROS 2 integration, its graphical rendering capabilities are functional but not highly realistic. **Unity**, a powerful real-time 3D development platform, provides superior graphical fidelity, advanced rendering pipelines, and extensive tools for creating rich, interactive environments.

By integrating Unity with Gazebo and ROS 2, we can:
-   **Enhance Visualization**: Create visually stunning representations of our robots and environments, crucial for human-robot interaction and presentation.
-   **Decouple Simulation and Visualization**: Leverage Gazebo's robust physics while using Unity for its strengths in rendering.
-   **Interactive Environments**: Build richer human-robot interaction scenarios that might be more challenging to create directly in Gazebo.

**Important Note**: As clarified in the specification, Unity's role in this curriculum is **exclusively for high-fidelity visualization**. All robot physics, control logic, and primary sensor simulation will remain within Gazebo and ROS 2. Students will learn to stream the state from their Gazebo simulation into Unity, not to replace Gazebo's physics engine.

### 6.2 Setting up Unity Robotics Hub

The **Unity Robotics Hub** is a collection of resources, tools, and tutorials provided by Unity to facilitate robotics development. The core component for ROS 2 integration is the **ROS-TCP-Connector**.

1.  **Install Unity**: Download and install Unity Hub and a recent LTS (Long Term Support) version of Unity (e.g., 2021.3 LTS).
2.  **Create a New Project**: Open Unity Hub and create a new 3D (URP - Universal Render Pipeline or HDRP - High Definition Render Pipeline) project.
3.  **Install ROS-TCP-Connector**:
    *   Open your Unity project.
    *   Go to `Window > Package Manager`.
    *   Click the '+' icon in the top-left and select `Add package from git URL...`.
    *   Enter `https://github.com/Unity-Technologies/Unity-Robotics-Hub.git?path=/com.unity.robotics.ros-tcp-connector#main` (or the appropriate branch/version).
    *   This will install the ROS-TCP-Connector package and its dependencies.

### 6.3 Streaming Simulation State from Gazebo to Unity

The core idea is to publish robot state data (e.g., joint positions, transforms) from ROS 2 (which is receiving it from Gazebo) to Unity.

1.  **ROS 2 Side (Gazebo/Python Node)**:
    *   Ensure your Gazebo simulation is publishing the robot's joint states to a ROS 2 topic (e.g., `/joint_states`, message type `sensor_msgs/JointState`). This is typically done via Gazebo-ROS plugins as discussed in Chapter 5.
    *   Additionally, ensure the `robot_state_publisher` node is running to publish the robot's TF (transform frames) based on the URDF and joint states. This is crucial for Unity to accurately position links.

2.  **Unity Side (ROS-TCP-Connector)**:
    *   **ROS Connection**: In Unity, add a `ROS Connection` component to a GameObject in your scene. Configure its `ROS IP Address` (the IP of your ROS 2 machine/WSL2 instance) and `Port`.
    *   **Subscribe to Joint States**: Create a C# script that uses `RosConnection.Get  Instance().Subscribe<Sensor_Msgs_JointState>("/joint_states", ReceiveJointStates)`.
    *   **Update Robot Model**: In the `ReceiveJointStates` callback, parse the incoming `JointState` message. Iterate through the robot's corresponding `ArticulationBody` components (or similar joint representations) in your Unity scene and update their `jointPosition` properties.
    *   **Transform Updates**: Similarly, you can subscribe to `/tf` (or specific TF frames) to get the position and orientation of robot links that are not directly controlled by joint states, such as the base link.

### 6.4 Building a Visualization Scene

Once you are streaming data, you can build a rich visualization in Unity:
-   **Import Robot Model**: Import your robot's URDF/SDF model into Unity using tools like the URDF Importer from the Unity Robotics Hub. This will create a Unity representation of your robot with `ArticulationBody` components for its joints.
-   **Environment Design**: Design a detailed 3D environment in Unity that mirrors or enhances your Gazebo world.
-   **Cameras and Lighting**: Configure Unity cameras and advanced lighting to create photorealistic renders.
-   **UI Overlays**: Add UI elements to display information from ROS 2 topics, enhancing the human-robot interaction experience.

This chapter demonstrates how to effectively combine the strengths of Gazebo for physics simulation and Unity for high-fidelity visualization, creating a powerful digital twin for your robotics projects.
