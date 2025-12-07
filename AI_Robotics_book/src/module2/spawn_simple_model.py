import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity
import os
import xacro

class SpawnSimpleModel(Node):
    def __init__(self):
        super().__init__('spawn_simple_model')
        self.client = self.create_client(SpawnEntity, '/spawn_entity')
        self.get_logger().info("Waiting for service /spawn_entity...")
        self.client.wait_for_service()
        self.get_logger().info("Service /spawn_entity is available.")

    def send_request(self):
        # Path to a simple box URDF/SDF model
        # For simplicity, we create a minimal SDF for a box here.
        # In a real scenario, this would load from a file.
        model_xml = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="simple_box">
    <pose>0 0 1 0 0 0</pose>
    <link name="box_link">
      <visual name="visual">
        <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
        <material><diffuse>1 0 0 1</diffuse></material>
      </visual>
      <collision name="collision">
        <geometry><box><size>0.5 0.5 0.5</size></box></geometry>
      </collision>
      <inertial><mass>1.0</mass><inertia ixx="0.083" ixy="0" ixz="0" iyy="0.083" iyz="0" izz="0.083"/></inertial>
    </link>
  </model>
</sdf>
"""

        request = SpawnEntity.Request()
        request.name = "my_simple_box"
        request.xml = model_xml
        request.robot_namespace = ""
        request.reference_frame = "world"

        self.get_logger().info("Sending spawn request...")
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f"Spawn service response: {future.result().status_message}")
        else:
            self.get_logger().error("Service call failed %r" % (future.exception(),))

def main(args=None):
    rclpy.init(args=args)
    spawner = SpawnSimpleModel()
    spawner.send_request()
    spawner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
