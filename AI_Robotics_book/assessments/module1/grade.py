import yaml
import json
import subprocess
import time
import rclpy
from std_msgs.msg import String

# This is a simplified simulation of an auto-grading script.
# In a real-world scenario, this would be a more robust ROS 2 node
# that programmatically launches and introspects the student's code.

def run_grader():
    """
    Simulates the grading process for the Module 1 assessment.
    """
    print("--- Starting Module 1 Auto-Grader ---")

    # 1. Load project definition
    with open("project_definition.yaml", "r") as f:
        config = yaml.safe_load(f)
    print("Loaded project definition.")

    total_score = 0
    test_case_results = []

    # In a real grader, we would launch the student's node here
    # For simulation, we assume the student's node is running separately.
    print("Assuming student's 'topic_relay_node' is running...")

    # 2. Initialize a temporary ROS 2 node for grading
    rclpy.init()
    node = rclpy.create_node('grader_node')
    publisher = node.create_publisher(String, config['interfaces']['subscription']['topic_name'], 10)
    
    # Simple subscriber to capture output
    received_messages = []
    def sub_callback(msg):
        print(f"Grader received: {msg.data}")
        received_messages.append(msg.data)

    subscription = node.create_subscription(String, config['interfaces']['publication']['topic_name'], sub_callback, 10)

    # 3. Run test cases
    for i, case in enumerate(config['test_cases']):
        print(f"\n--- Running Test Case: {case['name']} ---")
        msg_to_pub = String()
        msg_to_pub.data = case['input']
        
        # Clear previous messages and publish
        received_messages.clear()
        publisher.publish(msg_to_pub)
        print(f"Grader published: {msg_to_pub.data}")
        
        # Wait for the message to be processed
        time.sleep(2) # Simple delay for simulation
        rclpy.spin_once(node, timeout_sec=1)

        result = {
            "test_name": case['name'],
            "status": "FAIL",
            "score": 0,
            "feedback": "No message received on output topic."
        }

        if received_messages:
            if received_messages[0] == case['expected_output']:
                result['status'] = "PASS"
                result['score'] = case['score']
                result['feedback'] = "Correct message received."
                total_score += case['score']
            else:
                result['feedback'] = f"Incorrect message. Expected: '{case['expected_output']}', Got: '{received_messages[0]}'"
        
        test_case_results.append(result)

    # 4. Generate grade report
    grade_report = {
        "total_score": total_score,
        "test_case_results": test_case_results
    }

    with open("grade_report.json", "w") as f:
        json.dump(grade_report, f, indent=2)

    print("\n--- Grading Complete ---")
    print(json.dumps(grade_report, indent=2))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_grader()
