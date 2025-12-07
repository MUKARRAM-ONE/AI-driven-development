---
id: 04-urdf-for-humanoids
title: "Chapter 4: URDF for Humanoids"
sidebar_label: "4. URDF for Humanoids"
---

## Chapter 4: URDF for Humanoids

**Objective**: Learn to describe a robot's physical structure for simulation and visualization.

### 4.1 Introduction to URDF

The **Unified Robot Description Format (URDF)** is an XML format for representing a robot model. In ROS, URDF is the standard for describing the physical properties of a robot, including its links, joints, sensors, and visual appearance. A URDF file is not just for looks; it's a critical component for simulation, planning, and control.

A URDF file allows you to define:
-   The robot's **kinematic chain**: the arrangement of links and joints.
-   The **visual appearance** of each part of the robot.
-   The **collision properties** of each part for physics simulation.
-   The **inertial properties** (mass, center of gravity, etc.) of each part.

### 4.2 Core Components: `<robot>`, `<link>`, and `<joint>`

A URDF file is structured around three main tags:

-   **`<robot name="my_robot">`**: The root element of the file. All other elements are contained within this tag.

-   **`<link name="link_name">`**: A link represents a rigid body part of the robot. Each link has a name and can contain three sub-tags:
    -   **`<visual>`**: Defines the visual appearance of the link (e.g., a 3D mesh file).
    -   **`<collision>`**: Defines the collision geometry of the link, which may be simpler than the visual geometry for performance reasons.
    -   **`<inertial>`**: Defines the dynamic properties of the link, such as its mass and inertia tensor.

-   **`<joint name="joint_name" type="joint_type">`**: A joint connects two links together and defines how they can move relative to each other.
    -   The `type` attribute can be:
        -   `revolute`: A hinge joint that rotates around a single axis (e.g., an elbow).
        -   `continuous`: Similar to a revolute joint, but without angle limits (e.g., a wheel).
        -   `prismatic`: A sliding joint that moves along a single axis.
        -   `fixed`: A rigid connection between two links.
    -   A joint must specify a **`<parent link="..."/>`** and a **`<child link="..."/>`**.
    -   It also defines the **`<origin xyz="..." rpy="..."/>`** (position and orientation) of the child link relative to the parent link.
    -   For non-fixed joints, an **`<axis xyz="..."/>`** tag specifies the axis of rotation or translation.

### 4.3 Describing a Humanoid: A Simple Example

Let's create a very simple humanoid URDF with a torso, head, and one arm.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <!-- Base Link (Torso) -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.3 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Head Link -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Neck Joint -->
  <joint name="neck_joint" type="fixed">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
  </joint>

  <!-- Arm Link -->
  <link name="upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.4" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>

  <!-- Shoulder Joint -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="upper_arm"/>
    <origin xyz="0 0.2 0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

</robot>
```

### 4.4 Integrating Sensors

You can also attach sensors to your robot model by extending the URDF with sensor tags, often used by Gazebo plugins.

```xml
<!-- In the head link -->
<link name="head">
  ...
  <sensor name="camera" type="camera">
    <camera>
      <horizontal_fov>1.396</horizontal_fov>
      <image>
        <width>800</width>
        <height>800</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>300</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/demo</namespace>
        <remapping>image_raw:=image_demo</remapping>
      </ros>
    </plugin>
  </sensor>
</link>
```

### 4.5 Visualization in RViz2

**RViz2** is the standard visualization tool for ROS 2. You can use it to view your URDF model and other ROS data.

To view your URDF in RViz2, you need a `robot_state_publisher` node. This node reads the URDF file and publishes the robot's transformations (`tf2` frames) based on the joint states.

1.  **Create a launch file** to start the `robot_state_publisher`.
2.  **Run the launch file**.
3.  **Open RViz2**: `ros2 run rviz2 rviz2`
4.  **Add a "RobotModel" display** and set the "Description Topic" to `/robot_description`.

You should now see your humanoid model in the RViz2 window. This is a crucial step for debugging your robot model before using it in a full simulation.
