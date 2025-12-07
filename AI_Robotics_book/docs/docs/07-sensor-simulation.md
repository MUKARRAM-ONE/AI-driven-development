---
id: 07-sensor-simulation
title: "Chapter 7: Sensor Simulation"
sidebar_label: "7. Sensor Simulation"
---

## Chapter 7: Sensor Simulation

**Objective**: Simulate common robot sensors in Gazebo to generate realistic data.

### 7.1 Simulating a LiDAR

A **LiDAR (Light Detection and Ranging)** sensor measures distances to objects by emitting pulsed laser light and measuring the time it takes for the reflected light to return. In robotics, LiDAR is crucial for mapping, localization, and obstacle avoidance.

Gazebo provides a `gazebo_ros_ray_sensor` plugin that allows you to simulate a LiDAR. You integrate this plugin directly into your robot's URDF/SDF model.

#### Example URDF Integration (within a `<link>` tag)
```xml
<link name="hokuyo_link">
  <inertial>
    <mass value="0.1"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/hokuyo.dae"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>
</link>

<joint name="hokuyo_joint" type="fixed">
  <axis xyz="0 0 1"/>
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="hokuyo_link"/>
</joint>

<gazebo reference="hokuyo_link">
  <sensor type="ray" name="head_hokuyo_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>40</update_rate>
    <ray>
      <scan>
        <horizontal>
          <samples>720</samples>
          <resolution>1</resolution>
          <min_angle>-1.570796</min_angle>
          <max_angle>1.570796</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.1</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
    </ray>
    <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <argument>~/out:=scan</argument>
        <argument>~/out_depth:=depth</argument>
        <namespace>hokuyo</namespace>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>hokuyo_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

The simulated LiDAR will publish messages of type `sensor_msgs/LaserScan` to the `/hokuyo/scan` topic. You can visualize this data in RViz2 by adding a `LaserScan` display and setting its topic.

### 7.2 Simulating a Depth Camera

**Depth cameras** (like Intel RealSense or Microsoft Kinect) provide both an RGB image and a depth map, which indicates the distance of each pixel from the camera. This is invaluable for 3D perception, object recognition, and navigation.

Gazebo's `gazebo_ros_camera` plugin (or `libgazebo_ros_openni_kinect.so` for kinect-like sensors) can simulate these.

#### Example URDF Integration (within a `<link>` tag)
```xml
<link name="camera_link">
  <inertial>
    <mass value="0.01"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.03 0.03 0.03"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.03 0.03 0.03"/>
    </geometry>
  </collision>
</link>

<joint name="camera_joint" type="fixed">
  <origin xyz="0.05 0 0.05" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="camera_link"/>
</joint>

<gazebo reference="camera_link">
  <sensor type="depth_camera" name="depth_camera_sensor">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.1</near>
        <far>10</far>
      </clip>
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_depth_camera.so">
      <ros>
        <namespace>camera</namespace>
        <argument>image_raw:=image</argument>
        <argument>depth_image_raw:=depth</argument>
        <argument>camera_info:=camera_info</argument>
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link_optical</frame_name>
    </plugin>
  </sensor>
</gazebo>
```
The simulated depth camera will publish `sensor_msgs/Image` (RGB), `sensor_msgs/Image` (depth), and `sensor_msgs/CameraInfo` messages.

### 7.3 Simulating an IMU

An **IMU (Inertial Measurement Unit)** sensor typically provides linear acceleration and angular velocity, and sometimes orientation (roll, pitch, yaw). IMUs are fundamental for robot localization, balance, and motion tracking.

Gazebo's `libgazebo_ros_imu_sensor.so` plugin can simulate an IMU.

#### Example URDF Integration (within a `<link>` tag)
```xml
<link name="imu_link">
  <inertial>
    <mass value="0.001"/>
    <origin xyz="0 0 0"/>
    <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.01 0.01 0.01"/>
    </geometry>
  </collision>
</link>

<joint name="imu_joint" type="fixed">
  <origin xyz="0 0 0.01" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="imu_link"/>
</joint>

<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/imu</namespace>
        <argument>~/out:=data</argument>
      </ros>
      <topicName>data</topicName>
      <frameName>imu_link</frameName>
      <gaussianNoise>0.0001</gaussianNoise>
    </plugin>
  </sensor>
</gazebo>
```
The simulated IMU will publish `sensor_msgs/Imu` messages to the `/imu/data` topic.

### 7.4 Sensor Noise: Making Simulations Realistic

Real-world sensors are imperfect and introduce **noise** into their measurements. To make simulations more realistic and robust, it's important to model sensor noise. Gazebo allows you to configure various types of noise (e.g., Gaussian noise) for many sensor plugins.

For example, in the IMU plugin above, the `<gaussianNoise>0.0001</gaussianNoise>` tag adds Gaussian noise to the sensor readings. Similarly, ray sensors (LiDAR) and camera sensors often have parameters for adding noise. Modeling noise helps in developing algorithms that are less sensitive to real-world imperfections and in preparing for sim-to-real transfer.
