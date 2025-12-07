import yaml
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

# This is a simplified simulation of an auto-grading script.
# In a real-world scenario, this would involve launching Gazebo,
# spawning a robot, and launching the student's controller node.

class GraderNode(Node):
    def __init__(self, config):
        super().__init__('module2_grader_node')
        self.config = config
        self.joint_to_control = config['assessment_params']['joint_to_control']
        self.target_position = config['assessment_params']['target_position']
        self.tolerance = config['assessment_params']['tolerance']
        self.time_to_reach_target = config['assessment_params']['time_to_reach_target']

        self.joint_states_received = False
        self.current_joint_position = None

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states', # Assuming this topic publishes robot joint states
            self.joint_states_callback,
            10
        )
        self.get_logger().info(f"Grader node started, monitoring '/joint_states' for joint: {self.joint_to_control}")

    def joint_states_callback(self, msg):
        self.joint_states_received = True
        try:
            joint_index = msg.name.index(self.joint_to_control)
            self.current_joint_position = msg.position[joint_index]
            # self.get_logger().info(f"Current {self.joint_to_control} position: {self.current_joint_position:.2f}")
        except ValueError:
            self.get_logger().warn(f"Joint '{self.joint_to_control}' not found in /joint_states message.")
        except IndexError:
            self.get_logger().warn(f"Joint '{self.joint_to_control}' index out of bounds in /joint_states message.")

    def check_target_reached(self):
        if self.current_joint_position is not None:
            diff = abs(self.target_position - self.current_joint_position)
            return diff <= self.tolerance
        return False

def run_grader():
    """
    Executes the grading logic.
    """
    print("--- Starting Module 2 Auto-Grader ---")

    # 1. Load project definition
    with open("project_definition.yaml", "r") as f:
        config = yaml.safe_load(f)
    print("Loaded project definition.")

    total_score = 0
    test_case_results = []
    
    # Initialize ROS 2
    rclpy.init()
    grader_node = GraderNode(config)
    
    # In a real scenario, launch Gazebo and student's node here
    print("Please ensure Gazebo is running with a robot and the student's controller node is also running.")
    print(f"Monitoring joint '{grader_node.joint_to_control}' for target position '{grader_node.target_position}'...")
    
    start_time = time.time()
    target_reached = False
    
    while rclpy.ok() and (time.time() - start_time) < config['grading']['timeout']:
        rclpy.spin_once(grader_node, timeout_sec=0.1) # Process callbacks
        if grader_node.joint_states_received and grader_node.check_target_reached():
            target_reached = True
            break
        time.sleep(0.1) # Small delay
    
    # 2. Evaluate results
    test_result = {
        "test_name": config['test_cases'][0]['name'],
        "status": "FAIL",
        "score": 0,
        "feedback": f"Joint '{grader_node.joint_to_control}' did not reach target position '{grader_node.target_position}' within {config['grading']['timeout']} seconds."
    }

    if target_reached:
        test_result['status'] = "PASS"
        test_result['score'] = config['test_cases'][0]['score']
        test_result['feedback'] = f"Joint '{grader_node.joint_to_control}' successfully reached target position '{grader_node.target_position}'."
        total_score = config['grading']['max_score'] # Assuming one test case for simplicity

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
    run_grader()
