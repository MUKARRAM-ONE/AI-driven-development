# ROS 2 Interfaces Contract

This document defines the key ROS 2 interfaces that students will implement and interact with throughout the module. These interfaces establish the communication protocols between different robot components and systems.

## 1. Standard Topics

### `/cmd_vel` (Command Velocity)
- **Description**: Publishes desired linear and angular velocities for robot movement.
- **Message Type**: `geometry_msgs/Twist`
- **Publisher(s)**: Student-developed controller nodes, navigation stack.
- **Subscriber(s)**: Robot base controller (simulated or physical).
- **Key Fields**:
    - `linear.x` (float): Forward velocity.
    - `angular.z` (float): Rotational velocity around Z-axis.

### `/joint_states` (Robot Joint States)
- **Description**: Publishes the current position, velocity, and effort of robot joints.
- **Message Type**: `sensor_msgs/JointState`
- **Publisher(s)**: Robot (simulated or physical) joint encoders.
- **Subscriber(s)**: Student-developed perception nodes, visualization tools (e.g., RViz, Unity).
- **Key Fields**:
    - `name[]` (string): Array of joint names.
    - `position[]` (float): Array of joint positions.
    - `velocity[]` (float): Array of joint velocities.
    - `effort[]` (float): Array of joint efforts.

### `/scan` (LiDAR Scan Data)
- **Description**: Publishes range measurements from a LiDAR sensor.
- **Message Type**: `sensor_msgs/LaserScan`
- **Publisher(s)**: Simulated LiDAR sensor, physical LiDAR.
- **Subscriber(s)**: Student-developed navigation, obstacle avoidance, mapping nodes.
- **Key Fields**:
    - `angle_min`, `angle_max` (float): Start and end angles of the scan.
    - `angle_increment` (float): Angular distance between measurements.
    - `ranges[]` (float): Array of range measurements.

## 2. Standard Services

### `/set_joint_angle`
- **Description**: Sets a specific joint to a desired angle (position).
- **Service Type**: Custom (`robot_control/srv/SetJointAngle`)
- **Client(s)**: Student-developed high-level control nodes.
- **Server(s)**: Low-level joint controller node.
- **Request Fields**:
    - `joint_name` (string): Name of the joint to control.
    - `angle` (float): Desired angle in radians.
- **Response Fields**:
    - `success` (bool): True if command was successful, false otherwise.
    - `message` (string): Optional status message.

## 3. Standard Actions

### `/navigate_to_pose`
- **Description**: Commands the robot to navigate to a specified pose.
- **Action Type**: `nav2_msgs/NavigateToPose` (or similar custom action for humanoid bipedal locomotion)
- **Client(s)**: Student-developed task planners, VLA system.
- **Server(s)**: Navigation stack (e.g., Nav2).
- **Goal Fields**:
    - `pose` (geometry_msgs/PoseStamped): Desired target pose.
- **Result Fields**:
    - `succeeded` (bool): True if navigation succeeded, false otherwise.
- **Feedback Fields**:
    - `current_pose` (geometry_msgs/PoseStamped): Current robot pose.
    - `distance_remaining` (float): Distance to target.
    - `estimated_time_remaining` (duration): Estimated time to reach target.
