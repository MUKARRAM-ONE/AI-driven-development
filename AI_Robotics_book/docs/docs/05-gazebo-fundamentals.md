---
id: 05-gazebo-fundamentals
title: "Chapter 5: Gazebo Simulation Fundamentals"
sidebar_label: "5. Gazebo Simulation Fundamentals"
---

## Chapter 5: Gazebo Simulation Fundamentals

**Objective**: Simulate a robot in a virtual environment with realistic physics.

### 5.1 Introduction to Gazebo

**Gazebo** is a powerful 3D robot simulator that is widely used in robotics research and development. It provides the ability to accurately and efficiently simulate populations of robots in complex indoor and outdoor environments. Gazebo uses physics engines (like ODE, Bullet, DART, or Simbody) to provide realistic interactions between objects and robots.

**Why Gazebo?**
-   **Realistic Physics**: Simulate gravity, inertia, friction, and collisions.
-   **Comprehensive Sensor Models**: Simulate various sensors like cameras, LiDAR, IMUs, and force/torque sensors.
-   **Powerful API**: Allows programmatic control of the simulation and interaction with ROS 2.
-   **Large Community**: Extensive documentation, tutorials, and a vibrant user community.

![Digital Twin Architecture](/img/module2-architecture.svg)

### 5.2 Gazebo World Files

A Gazebo simulation is defined by a **world file** (typically with a `.world` extension). This XML-based file describes everything in your simulation environment:
-   **Physics Engine**: Which physics engine to use and its parameters (e.g., gravity, time step).
-   **Lights**: Ambient, directional, and point lights.
-   **Models**: Static objects (e.g., walls, tables) and dynamic robots.
-   **Ground Plane**: A floor for your simulation.

#### Example `simple_world.world`
```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- Light Source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- Ground Plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Custom Box Model -->
    <model name="my_box">
      <pose>0 0 0.5 0 0 0</pose> <!-- x y z roll pitch yaw -->
      <link name="box_link">
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.8 0.1 0.1 1</ambient>
            <diffuse>0.8 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <inertial>
          <mass>10.0</mass>
          <inertia>
            <ixx>1.0</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>1.0</iyy><iyz>0.0</iyz><izz>1.0</izz>
          </inertia>
        </inertial>
      </link>
    </model>
  </world>
</sdf>
```

You can launch this world using: `gazebo -s libgazebo_ros_factory.so simple_world.world`

### 5.3 Spawning Robots

Robots are typically spawned into a Gazebo world using their **URDF** or **SDF** (Simulation Description Format) descriptions. While URDF is great for kinematic chains and visuals, SDF is Gazebo's native format and provides more comprehensive features for simulation, such as sensor definitions and physics properties.

#### Spawning a URDF model
You can spawn a URDF model using the `spawn_entity.py` script from `ros2_gazebo_utils`:

```bash
ros2 run gazebo_ros spawn_entity.py -entity my_robot -file $(ros2 pkg prefix my_robot_pkg)/share/my_robot_pkg/urdf/my_robot.urdf -x 0 -y 0 -z 1
```

### 5.4 ROS 2 Integration

Gazebo provides plugins that bridge the simulation with ROS 2. These **Gazebo-ROS plugins** allow you to:
-   Publish robot joint states to ROS 2 topics.
-   Subscribe to ROS 2 topics to control robot joints (e.g., `/cmd_vel`).
-   Publish sensor data (camera images, LiDAR scans, IMU data) to ROS 2 topics.

These plugins are typically added directly within the robot's URDF/SDF description.

#### Example Gazebo-ROS Joint State Publisher Plugin
```xml
<gazebo>
  <plugin name="joint_state_broadcaster" filename="libgazebo_ros_joint_state_broadcaster.so">
    <ros>
      <namespace>/my_robot</namespace>
    </ros>
    <update_rate>50</update_rate>
    <joint_name>shoulder_pan_joint</joint_name>
    <joint_name>shoulder_lift_joint</joint_name>
    <!-- Add all joints you want to publish -->
  </plugin>
</gazebo>
```

This chapter has provided a foundational understanding of Gazebo, its world files, and how to integrate robots and simple models. In the next chapter, we will see how to leverage Unity for high-fidelity visualization.
