import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy.action
import json
import logging
import math

class ActionExecutorNode(Node):
    def __init__(self):
        super().__init__('action_executor')
        self.action_sequence_subscription = self.create_subscription(
            String,
            '/robot_action_sequence',
            self.execute_action_sequence_callback,
            10
        )
        self.get_logger().info('Action Executor node started. Waiting for action sequences.')

        # Initialize ROS 2 clients for various actions
        self.nav_to_pose_client = rclpy.action.ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.joint_trajectory_publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        # Add other clients/publishers as needed for 'pick_up_object', 'find_object', 'say_phrase' etc.

    def execute_action_sequence_callback(self, msg):
        try:
            action_sequence = json.loads(msg.data)
            self.get_logger().info(f"Received action sequence: {action_sequence}")

            for action in action_sequence:
                action_type = action.get('action')
                params = action.get('params', {})

                if action_type == 'navigate_to_pose':
                    self.execute_navigate_to_pose(params)
                elif action_type == 'pick_up_object':
                    self.execute_pick_up_object(params)
                elif action_type == 'find_object':
                    self.execute_find_object(params)
                elif action_type == 'say_phrase':
                    self.execute_say_phrase(params)
                else:
                    self.get_logger().warn(f"Unknown action type: {action_type}")
            
            self.get_logger().info("Action sequence execution finished.")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to parse action sequence JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error executing action sequence: {e}")

    def execute_navigate_to_pose(self, params):
        self.get_logger().info(f"Executing navigate_to_pose: {params}")
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal_msg.pose.pose.position.x = float(params.get('x', 0.0))
        goal_msg.pose.pose.position.y = float(params.get('y', 0.0))
        goal_msg.pose.pose.position.z = 0.0 # Assuming 2D navigation

        yaw_degrees = float(params.get('yaw_degrees', 0.0))
        yaw_radians = math.radians(yaw_degrees)
        q = self.euler_to_quaternion(0, 0, yaw_radians)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f'Sending navigation goal: {params}')
        self.nav_to_pose_client.wait_for_server()
        future = self.nav_to_pose_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected :(')
            return

        self.get_logger().info('Navigation goal accepted :)')
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        self.get_logger().info(f'Navigation result: {result.status}')

    def execute_pick_up_object(self, params):
        self.get_logger().info(f"Executing pick_up_object: {params.get('object_name')}")
        # This would involve publishing joint trajectories or calling a manipulation service
        # For simulation, just log it.
        self.get_logger().info(f"Simulating picking up {params.get('object_name')}")

    def execute_find_object(self, params):
        self.get_logger().info(f"Executing find_object: {params.get('object_name')}")
        # This would involve calling a perception service or subscribing to a perception topic
        # For simulation, just log it.
        self.get_logger().info(f"Simulating finding {params.get('object_name')}")
        
    def execute_say_phrase(self, params):
        self.get_logger().info(f"Executing say_phrase: '{params.get('phrase')}'")
        # This would involve Text-to-Speech (TTS) synthesis
        # For simulation, just log it.
        self.get_logger().info(f"Robot says: '{params.get('phrase')}'")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    action_executor = ActionExecutorNode()
    rclpy.spin(action_executor)
    action_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
