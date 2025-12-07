import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import random
import math

class ROSUnityBridge(Node):
    def __init__(self):
        super().__init__('ros_unity_bridge')
        self.joint_state_publisher_ = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster_ = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10 Hz
        self.get_logger().info('ROS-Unity Bridge node started.')

        self.joint_pos1 = 0.0
        self.joint_pos2 = 0.0

    def timer_callback(self):
        # 1. Publish JointStates (simulated from Gazebo)
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = ['joint1', 'joint2']
        
        self.joint_pos1 = math.sin(self.get_clock().now().nanoseconds / 1e9) # Oscillating joint
        self.joint_pos2 = random.uniform(-0.5, 0.5) # Random joint

        joint_state_msg.position = [self.joint_pos1, self.joint_pos2]
        joint_state_msg.velocity = [0.0, 0.0]
        joint_state_msg.effort = [0.0, 0.0]
        self.joint_state_publisher_.publish(joint_state_msg)
        
        # 2. Publish TF (simulated base_link from Gazebo)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom' # Parent frame
        t.child_frame_id = 'base_link' # Child frame (e.g., robot's base)
        
        t.transform.translation.x = math.sin(self.get_clock().now().nanoseconds / 1e9 / 2) * 2
        t.transform.translation.y = math.cos(self.get_clock().now().nanoseconds / 1e9 / 2) * 2
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0 # No rotation for simplicity
        
        self.tf_broadcaster_.sendTransform(t)

        self.get_logger().info(f'Published simulated joint states and TF.')

def main(args=None):
    rclpy.init(args=args)
    ros_unity_bridge = ROSUnityBridge()
    rclpy.spin(ros_unity_bridge)
    ros_unity_bridge.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
