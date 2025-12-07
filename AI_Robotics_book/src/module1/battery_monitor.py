import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import logging

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

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    battery_monitor = BatteryMonitorNode()
    rclpy.spin(battery_monitor)
    battery_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
