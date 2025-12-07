# Module 1 Code Examples

This directory contains the Python code examples for Module 1: The Robotic Nervous System (ROS 2).

## Files

-   `simple_publisher.py`: A basic ROS 2 node that publishes a string message to a topic.
-   `simple_subscriber.py`: A basic ROS 2 node that subscribes to a string message from a topic.
-   `simple_service_server.py`: A node that provides a service to add two integers.
-   `simple_service_client.py`: A node that calls the 'add two integers' service.
-   `battery_monitor.py`: A node that demonstrates the use of parameters by simulating a battery drain.
-   `my_launch.launch.py`: A ROS 2 launch file that starts the `battery_monitor` and a subscriber node.

## How to Run

1.  Ensure you have a ROS 2 workspace (`ros2_ws`).
2.  Create a new package (e.g., `my_robot_tutorials`) inside the `src` directory of your workspace.
3.  Copy these example files into your new package's directory.
4.  Update your `setup.py` to include the new nodes as entry points.
5.  Build your workspace: `colcon build`.
6.  Source your workspace: `source install/setup.bash`.
7.  Run the examples using `ros2 run <package_name> <executable_name>` or `ros2 launch <package_name> <launch_file_name>`.
