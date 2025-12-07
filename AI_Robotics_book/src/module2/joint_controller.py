import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
from builtin_interfaces.msg import Duration
import time

class JointController(Node):
    def __init__(self):
        super().__init__('joint_controller')
        self.publisher_ = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.get_logger().info('Joint Controller node started.')

        # Example joint names - replace with your robot's actual joint names
        self.joint_names = ['joint1', 'joint2'] 
        self.current_position = [0.0, 0.0]

        # Publish a trajectory every few seconds
        self.timer = self.create_timer(5.0, self.publish_trajectory)
    
    def publish_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        
        # Toggle positions
        if self.current_position[0] == 0.0:
            self.current_position = [1.0, -0.5] # Move to new position
        else:
            self.current_position = [0.0, 0.0] # Return to home position

        point.positions = self.current_position
        point.time_from_start = Duration(sec=2, nanosec=0) # Reach target in 2 seconds
        
        trajectory_msg.points.append(point)
        self.publisher_.publish(trajectory_msg)
        self.get_logger().info(f'Published new joint trajectory to: {self.current_position}')

def main(args=None):
    rclpy.init(args=args)
    joint_controller = JointController()
    rclpy.spin(joint_controller)
    joint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
