from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Define package name (replace with your actual package name)
    capstone_package_name = 'my_capstone_pkg' 

    # Path to the URDF file
    # This assumes humanoid_robot.urdf is in the same package's 'urdf' directory
    # For a simple example, we are using a dummy path.
    # In a real scenario, you'd use ament_index_python.packages to find it.
    humanoid_urdf_path = os.path.join(
        #get_package_share_directory(capstone_package_name),
        #'urdf',
        'src/capstone/humanoid_robot.urdf' # Dummy path for now
    )


    return LaunchDescription([
        # Robot State Publisher to parse URDF and publish TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(humanoid_urdf_path).read()}]
        ),
        
        # Joint State Publisher (for manually controlling joints in Rviz, or from a simulator)
        Node(
            package='joint_state_publisher_gui', # GUI version for testing
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        # Voice Command Transcriber Node (Module 4)
        Node(
            package=capstone_package_name, # Assuming it's in the capstone package
            executable='voice_command_transcriber.py',
            name='voice_command_transcriber',
            output='screen'
        ),

        # Cognitive Planner Node (Module 4)
        Node(
            package=capstone_package_name,
            executable='cognitive_planner_node.py',
            name='cognitive_planner',
            output='screen'
        ),

        # Action Executor Node (Module 4)
        Node(
            package=capstone_package_name,
            executable='action_executor_node.py',
            name='action_executor',
            output='screen'
        ),
        
        # Example: Nav2 Stack (conceptual - would require full Nav2 setup)
        # Node(
        #     package='nav2_bringup',
        #     executable='bringup_node',
        #     name='nav2_bringup',
        #     output='screen',
        #     parameters=[os.path.join(get_package_share_directory('nav2_bringup'), 'config', 'nav2_params.yaml')]
        # ),

        # Rviz2 for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(os.path.dirname(__file__), 'rviz_config.rviz')]
        )
    ])
