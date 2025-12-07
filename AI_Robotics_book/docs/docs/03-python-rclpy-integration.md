---
id: 03-python-rclpy-integration
title: "Chapter 3: Python Integration with rclpy"
sidebar_label: "3. Python Integration with rclpy"
---

## Chapter 3: Python Integration with rclpy

**Objective**: Master the creation of custom ROS 2 applications using the `rclpy` library.

### 3.1 Creating a ROS 2 Package

A ROS 2 package is a directory with a specific structure that contains your nodes, launch files, custom message definitions, and a `package.xml` manifest file.

To create a new package, you can use the `ros2 pkg create` command:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python --node-name my_first_node my_robot_tutorials
```

This command creates a new directory `my_robot_tutorials` with the following structure:

-   `package.xml`: An XML file containing meta-information about the package, such as its name, version, author, and dependencies.
-   `setup.py`: A Python script for installing the package and its executables.
-   `setup.cfg`: A configuration file for the `setup.py` script.
-   `my_robot_tutorials/`: The Python package directory containing your node scripts.
    -   `my_first_node.py`: A basic, empty node script.
-   `resource/`: A directory to mark the package location.
-   `test/`: A directory for your package's tests.

### 3.2 Writing a Custom Node

A ROS 2 node is typically structured as a Python class that inherits from `rclpy.node.Node`. This provides access to all the necessary ROS 2 functionality.

Let's modify the `my_first_node.py` script to create a node that publishes the robot's simulated battery level.

```python
# In my_robot_tutorials/my_first_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        self.publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_level)
        self.battery_level_ = 100.0
        self.get_logger().info('Battery Monitor node started.')

    def publish_battery_level(self):
        msg = Float32()
        msg.data = self.battery_level_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery level: {self.battery_level_:.2f}%')
        self.battery_level_ -= 0.1 # Simulate battery drain
        if self.battery_level_ < 0:
            self.battery_level_ = 100.0 # Reset
```

### 3.3 Parameter Management

Parameters allow you to configure your nodes externally without changing the code. You can declare, set, and get parameters.

Let's modify our `BatteryMonitorNode` to use a parameter for the battery drain rate.

```python
# In my_robot_tutorials/my_first_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class BatteryMonitorNode(Node):
    def __init__(self):
        super().__init__('battery_monitor')
        # Declare parameter for drain rate
        self.declare_parameter('drain_rate', 0.1)
        self.drain_rate_ = self.get_parameter('drain_rate').get_parameter_value().double_value

        self.publisher_ = self.create_publisher(Float32, 'battery_level', 10)
        self.timer = self.create_timer(1.0, self.publish_battery_level)
        self.battery_level_ = 100.0
        self.get_logger().info(f'Battery Monitor node started with drain rate: {self.drain_rate_}.')

    def publish_battery_level(self):
        msg = Float32()
        msg.data = self.battery_level_
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing battery level: {self.battery_level_:.2f}%')
        self.battery_level_ -= self.drain_rate_
        if self.battery_level_ < 0:
            self.battery_level_ = 100.0
```

You can then run this node and set the parameter from the command line:

```bash
ros2 run my_robot_tutorials my_first_node --ros-args -p drain_rate:=0.5
```

### 3.4 Launch Files

Launch files (`.launch.py`) allow you to start and configure multiple nodes at once. This is essential for managing complex robotic systems.

Here's an example of a launch file that starts our `BatteryMonitorNode` and a simple subscriber node to monitor it.

```python
# In my_robot_tutorials/my_launch.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='my_first_node',
            name='battery_monitor',
            parameters=[{'drain_rate': 0.25}]
        ),
        Node(
            package='my_robot_tutorials',
            executable='my_subscriber_node', # Assuming you have a subscriber node
            name='battery_display'
        )
    ])
```

To run this launch file:

```bash
ros2 launch my_robot_tutorials my_launch.launch.py
```

This chapter has provided the foundational skills for creating custom ROS 2 applications with Python. With this knowledge, you are now ready to build more complex robotic behaviors.
