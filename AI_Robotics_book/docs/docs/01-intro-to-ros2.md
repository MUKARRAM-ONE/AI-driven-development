---
id: 01-intro-to-ros2
title: "Chapter 1: Introduction to ROS 2"
sidebar_label: "1. Introduction to ROS 2"
---

## Chapter 1: Introduction to ROS 2

**Objective**: Understand the motivation behind ROS 2 and its core architectural principles.

### 1.1 What is ROS and Why Use It?

The Robot Operating System (ROS) is not a traditional operating system in the sense of Windows, macOS, or Linux. Instead, it is a **meta-operating system**: a flexible framework for writing robot software. It provides a collection of tools, libraries, and conventions that aim to simplify the complex task of creating robust and versatile robot behavior across a wide variety of robotic platforms.

Think of ROS as the glue that holds a robot's software together. A modern robot is a complex system of sensors, actuators, and computers. ROS provides a standardized way for these components to communicate with each other, from low-level motor control to high-level planning and perception.

**Real-world applications of ROS**:
- **Logistics and Warehousing**: Autonomous mobile robots (AMRs) use ROS for navigation, obstacle avoidance, and inventory management.
- **Healthcare**: Surgical robots like the da Vinci system use ROS-like frameworks for precise control and feedback.
- **Autonomous Vehicles**: Many self-driving car prototypes and research vehicles use ROS for sensor fusion, perception, and decision making.
- **Academic Research**: ROS is the de facto standard in robotics research, enabling collaboration and reproducible results.

### 1.2 From ROS 1 to ROS 2: The Need for Change

ROS 1, first released in 2007, was a groundbreaking success. However, as robotics evolved, its limitations became apparent, particularly in the areas of:
- **Real-time performance**: ROS 1's communication system was not designed for hard real-time control loops.
- **Multi-robot systems**: ROS 1 had a single point of failure (the ROS Master), making it difficult to create decentralized, multi-robot systems.
- **Security**: ROS 1 had no built-in security mechanisms, a major concern for commercial and connected robots.
- **Resource constraints**: ROS 1 was not well-suited for small, embedded systems.

ROS 2 was designed from the ground up to address these limitations, with a focus on:
- **Improved performance**: Leveraging the Data Distribution Service (DDS) for real-time communication.
- **Enhanced reliability**: A decentralized discovery process eliminates the single point of failure.
- **Robust security**: Built-in security features for authentication, encryption, and access control.
- **Greater flexibility**: Support for a wider range of platforms, from microcontrollers to multi-core desktops.

### 1.3 DDS (Data Distribution Service): The Middleware Powering ROS 2

The most significant architectural change in ROS 2 is the adoption of the **Data Distribution Service (DDS)** as its core communication middleware. DDS is an industry standard for high-performance, real-time, and reliable data exchange.

Instead of a centralized "ROS Master" as in ROS 1, ROS 2 nodes use DDS to automatically discover each other on the network. This publish-subscribe model allows for:
- **Decentralized discovery**: Nodes can join and leave the network dynamically without a central coordinator.
- **Quality of Service (QoS)**: Fine-grained control over the reliability, durability, and timeliness of data exchange. This is crucial for robotics, where a missed message to a motor can have real-world consequences.
- **Interoperability**: DDS is a standard, allowing ROS 2 systems to potentially interoperate with non-ROS systems that also use DDS.

![Module 1 Architecture](/img/module1-architecture.svg)

### 1.4 Installation and Setup

This section provides a brief overview of the installation process. For a detailed, step-by-step guide, please refer to the `quickstart.md` document.

**Supported Platforms**:
- **Primary**: Ubuntu 22.04 LTS
- **Secondary**: Windows 10/11 with WSL2 (Windows Subsystem for Linux 2)

**Installation Steps**:

1.  **Install ROS 2 Humble/Iron**: Follow the official installation guide for your platform. This typically involves adding the ROS 2 apt repository and installing the `ros-humble-desktop` (or `ros-iron-desktop`) package.
2.  **Source the Setup File**: After installation, you need to source the ROS 2 setup file to make the ROS 2 commands available in your terminal. This is usually done by adding `source /opt/ros/humble/setup.bash` to your `~/.bashrc` file.
3.  **Verify Installation with the 'Talker-Listener' Demo**: To confirm that ROS 2 is installed correctly, you can run the `talker-listener` demo.
    -   Open two separate terminals.
    -   In the first terminal, run: `ros2 run demo_nodes_py talker`
    -   In the second terminal, run: `ros2 run demo_nodes_py listener`

    You should see the "talker" node publishing messages and the "listener" node receiving and printing them. This confirms that your ROS 2 communication is working correctly.

With ROS 2 successfully installed, you are now ready to dive into the fundamental concepts of building a robotic nervous system.
