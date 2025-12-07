import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import logging

class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__('simple_service_client')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        return self.client.call_async(self.req)

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    service_client = SimpleServiceClient()
    future = service_client.send_request(5, 10)
    rclpy.spin_until_future_complete(service_client, future)
    response = future.result()
    service_client.get_logger().info(f'Result of add_two_ints: {response.sum}')
    service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
