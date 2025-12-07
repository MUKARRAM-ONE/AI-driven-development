import yaml
import json
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Duration
import time
import math

# This is a simplified simulation of an auto-grading script for Nav2 goal publishing.
# In a real-world scenario, this would monitor the student's action client
# interacting with a real (or mocked) Nav2 action server.

class Nav2GraderNode(Node):
    def __init__(self, config):
        super().__init__('module3_grader_node')
        self.config = config
        self.target_x = config['assessment_params']['target_x']
        self.target_y = config['assessment_params']['target_y']
        self.target_yaw_degrees = config['assessment_params']['target_yaw_degrees']

        self.action_server = rclpy.action.ActionServer(
            self,
            NavigateToPose,
            config['interfaces']['action_client']['action_name'],
            self.execute_callback
        )
        self.get_logger().info(f"Grader node started, acting as Nav2 action server for '{config['interfaces']['action_client']['action_name']}'")

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Received goal request: {goal_handle.request}")
        
        # Extract goal from student's request
        req_x = goal_handle.request.pose.pose.position.x
        req_y = goal_handle.request.pose.pose.position.y
        req_orientation = goal_handle.request.pose.pose.orientation
        
        # Convert target yaw to quaternion for comparison
        target_yaw_radians = math.radians(self.target_yaw_degrees)
        target_q = self.euler_to_quaternion(0, 0, target_yaw_radians)

        # Simple comparison with some tolerance
        x_match = abs(req_x - self.target_x) < 0.1
        y_match = abs(req_y - self.target_y) < 0.1
        
        # Simplified orientation check: check z and w components for 2D yaw
        yaw_match = (abs(req_orientation.z - target_q[2]) < 0.1) and (abs(req_orientation.w - target_q[3]) < 0.1)


        if x_match and y_match and yaw_match:
            self.get_logger().info("Goal matches expected. Accepting goal.")
            goal_handle.succeed()
            result = NavigateToPose.Result()
            result.status = 0 # success
            return result
        else:
            self.get_logger().warn("Goal does NOT match expected. Aborting goal.")
            goal_handle.abort()
            result = NavigateToPose.Result()
            result.status = 1 # failed
            return result

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]


def run_grader():
    """
    Executes the grading logic.
    """
    print("--- Starting Module 3 Auto-Grader ---")

    # 1. Load project definition
    with open("project_definition.yaml", "r") as f:
        config = yaml.safe_load(f)
    print("Loaded project definition.")

    total_score = 0
    test_case_results = []
    
    rclpy.init()
    grader_node = Nav2GraderNode(config)
    
    print("Waiting for student's Nav2 Action Client to send a goal...")
    
    start_time = time.time()
    goal_processed = False
    
    while rclpy.ok() and (time.time() - start_time) < config['grading']['timeout']:
        rclpy.spin_once(grader_node, timeout_sec=0.1) # Process callbacks (will execute action server)
        # Check if the goal was processed (simplified: assume it happened within spin_once)
        # In a real scenario, you'd monitor the action client's state or result.
        if grader_node.action_server._goal_handles: # Simplified check if a goal was ever received
            goal_processed = True
            break
        time.sleep(0.1)
    
    # Simulate grading based on the action server's outcome (internal to the grader_node)
    # This part needs to be more robust in a real grader
    if goal_processed and grader_node.action_server._goal_handles[0].status == rclpy.action.server.GoalStatus.SUCCEEDED:
        test_result = {
            "test_name": config['test_cases'][0]['name'],
            "status": "PASS",
            "score": config['test_cases'][0]['score'],
            "feedback": "Navigation goal sent correctly and accepted by simulated Nav2 server."
        }
        total_score = config['grading']['max_score']
    else:
        test_result = {
            "test_name": config['test_cases'][0]['name'],
            "status": "FAIL",
            "score": 0,
            "feedback": "Navigation goal either not sent, incorrect, or rejected by simulated Nav2 server."
        }
        
    test_case_results.append(test_result)

    # 3. Generate grade report
    grade_report = {
        "total_score": total_score,
        "test_case_results": test_case_results
    }

    with open("grade_report.json", "w") as f:
        json.dump(grade_report, f, indent=2)

    print("\n--- Grading Complete ---")
    print(json.dumps(grade_report, indent=2))

    grader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    import random # Not used here, but keeping consistent with other examples
    run_grader()
