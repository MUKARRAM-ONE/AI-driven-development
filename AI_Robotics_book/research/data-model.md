# Data Model: Physical AI & Humanoid Robotics Textbook Module

This document outlines the key entities and their relationships within the "Physical AI & Humanoid Robotics Textbook Module" curriculum. While not a traditional software data model, it defines the conceptual components and data flows that students will learn to implement and interact with.

## Entities

### 1. Robot
- **Description**: A simulated or physical humanoid robot, representing the primary actor for embodied intelligence.
- **Key Attributes**:
    - **Joints**: Articulated components allowing movement (e.g., `joint_name: float (angle/position)`).
    - **Links**: Rigid body components connecting joints (e.g., `link_name`).
    - **Sensors**: Devices providing environmental perception.
        - `LiDAR`: Range data (e.g., `[float, ...]`).
        - `Depth Camera`: RGB image, depth map (e.g., `(image_data, depth_map)`).
        - `IMU`: Orientation, acceleration, angular velocity (e.g., `(quaternion, linear_acceleration, angular_velocity)`).
    - **Actuators**: Mechanisms for physical movement (e.g., `motor_command: float`).
- **Relationships**:
    - Interacts with `ROS 2 Components` (receives commands, publishes sensor data).
    - Operates within `Simulation Elements`.

### 2. ROS 2 Components
- **Description**: The middleware framework for robot control and communication.
- **Key Attributes**:
    - **Nodes**: Executable processes (e.g., `perception_node`, `controller_node`).
    - **Topics**: Data streams for asynchronous communication (e.g., `/cmd_vel`, `/joint_states`, `/scan`).
    - **Services**: Request-response communication (e.g., `/set_joint_angle`).
    - **Actions**: Long-running, preemptable tasks (e.g., `/navigate_to_pose`).
    - **Parameters**: Configuration values for nodes.
    - **Packages**: Logical units organizing nodes, messages, services, etc.
- **Relationships**:
    - `Nodes` communicate via `Topics`, `Services`, `Actions`.
    - `Nodes` control `Robot` components.
    - Integrates with `Simulation Elements`.

### 3. Simulation Elements
- **Description**: The virtual environment and physics engine where robots operate.
- **Key Attributes**:
    - **World**: Virtual scene including obstacles, terrain, light.
    - **Models**: Representation of robots, objects, and environment elements.
    - **Physics Engine**: Governs interactions (gravity, collisions).
- **Relationships**:
    - Hosts `Robot` entities.
    - Provides simulated sensor data to `ROS 2 Components`.
    - Visualized by `Unity Visualization`.

### 4. VLA System (Vision-Language-Action System)
- **Description**: The system translating natural language commands into robot actions.
- **Key Attributes**:
    - **Voice Command**: Raw audio input.
    - **Natural Language Plan**: Textual representation of the desired task (e.g., "Clean the room").
    - **LLM**: Processes natural language into action sequences.
    - **ROS 2 Actions**: Sequences of robot tasks derived from the LLM's plan.
- **Relationships**:
    - Receives `Voice Command` (e.g., via OpenAI Whisper).
    - Generates `ROS 2 Actions`.
    - Leverages `Robot` capabilities.

### 5. Assessment
- **Description**: A standardized, auto-gradable coding project to evaluate student understanding and application of module concepts.
- **Key Attributes**:
    - **Coding Project**: Student-developed ROS 2 nodes or scripts.
    - **Success Criteria**: Clear, measurable objectives for the project (e.g., "robot reaches target pose").
    - **Auto-grading Script**: Automated system to evaluate student code against success criteria.
- **Relationships**:
    - Evaluates student interaction with `Robot` and `ROS 2 Components`.

### 6. Unity Visualization
- **Description**: A high-fidelity rendering environment used to visualize Gazebo simulation states.
- **Key Attributes**:
    - **Streamed State**: Robot joint positions, sensor data from Gazebo.
    - **High-fidelity Rendering**: Visual representation of the robot and environment.
- **Relationships**:
    - Receives state from `Simulation Elements`.
    - Provides enhanced visual feedback to the student.
    - No direct control over `Robot` physics or control.