from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_robot_tutorials',
            executable='battery_monitor',
            name='battery_monitor',
            parameters=[{'drain_rate': 0.25}]
        ),
        Node(
            package='my_robot_tutorials',
            executable='simple_subscriber',
            name='battery_display'
        )
    ])
