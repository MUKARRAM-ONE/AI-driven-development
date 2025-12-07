--- 
id: 02-ros2-nodes-topics-services
title: "Chapter 2: Nodes, Topics, and Services"
sidebar_label: "2. Nodes, Topics, and Services"
---

## Chapter 2: Nodes, Topics, and Services

**Objective**: Learn the fundamental communication patterns in ROS 2.

### 2.1 ROS 2 Graph Concepts

The "ROS Graph" is the network of ROS 2 elements processing data together. It's a peer-to-peer network of processes (or **nodes**) that are loosely coupled. The graph is composed of:

-   **Nodes**: A node is an executable that uses ROS 2 to communicate with other nodes. Each node in ROS should be responsible for a single, module purpose (e.g., one node for controlling a wheel motor, one node for controlling a laser range-finder).
-   **Topics**: A topic is a named bus over which nodes exchange messages. Topics are intended for unidirectional, streaming data.
-   **Services**: A service is a request/response communication pattern. One node offers a service, and another node can call that service.
-   **Actions**: An action is for long-running tasks. It provides feedback during execution and is preemptable.
-   **Parameters**: A parameter is a configuration value for a node. You can think of parameters as node settings.

![ROS 2 Pub/Sub Architecture](/img/module1-pub-sub-sequence.svg)

### 2.2 The Publisher-Subscriber Pattern (Topics)

The most common communication pattern in ROS is the publisher-subscriber model, which is anonymous and asynchronous. A node that produces data **publishes** it to a named topic. A node that is interested in that data **subscribes** to that topic. There can be multiple publishers and multiple subscribers on a single topic.

#### Creating a Publisher Node in Python

Here is a simple example of a Python node that publishes a "Hello World" message every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('Publisher has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = f"Hello World: {self.get_clock().now()}"
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = SimplePublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Creating a Subscriber Node in Python

This node subscribes to the `chatter` topic and prints the messages it receives.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSubscriber(Node):
    def __init__(self):
        super().__init__('simple_subscriber')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.get_logger().info('Subscriber has been started.')

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = SimpleSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3 The Service (Request-Response) Pattern

Services are a request/response communication pattern, suitable for tasks that require a direct, synchronous exchange, like querying a database or triggering a specific action that completes quickly.

#### Creating a Service Server Node

This node provides a service that adds two integers.

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class SimpleServiceServer(Node):
    def __init__(self):
        super().__init__('simple_service_server')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.get_logger().info('Service server has been started.')

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    simple_service_server = SimpleServiceServer()
    rclpy.spin(simple_service_server)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Creating a Service Client Node

This node calls the `add_two_ints` service.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

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
```

### 2.4 ROS 2 CLI: Introspection and Debugging

ROS 2 provides a powerful set of command-line tools to introspect and debug your ROS graph.

-   **`ros2 node list`**: Lists all active nodes.
-   **`ros2 topic list`**: Lists all active topics.
-   **`ros2 service list`**: Lists all active services.
-   **`ros2 action list`**: Lists all active actions.
-   **`ros2 topic echo <topic_name>`**: Prints messages published to a topic.
-   **`ros2 service call <service_name> <service_type> '<args>'`**: Calls a service from the command line.

```