# Assessment Contract

This document defines the contract for student assessments, focusing on the standardized input, expected student output, and the mechanism for auto-grading each module's coding project. This contract aims for clarity, fairness, and an optimized, cost-effective evaluation process.

## 1. Student Submission Contract

### Input
- **Student Code**: A Python file (or set of files) containing the implementation of a ROS 2 node, script, or other programmatic solution as specified by the module's project prompt.
    - **`project_main.py`**: Entry point for the student's solution.
    - **`package.xml` / `setup.py`**: Standard ROS 2 package definition files (if applicable).
- **Configuration Files**: Any necessary configuration files (e.g., `.yaml`, `.launch.py`) as specified by the project.

### Output (Expected from Student Code)
- **ROS 2 Topic Publications**: The student's node(s) MUST publish messages to specific ROS 2 topics with defined message types and content (e.g., a `/robot_path` topic publishing `geometry_msgs/PoseArray`).
- **ROS 2 Service Responses**: The student's node(s) MUST respond correctly to specific ROS 2 service calls with defined request and response types.
- **Console Output / Logging**: Specific output patterns or log messages (e.g., successful task completion indicators) might be expected for validation.
- **Simulation State**: For simulation-based projects, the student's code MUST achieve a specific robot state or perform a sequence of actions within the simulated environment (e.g., robot reaching a target pose in Gazebo).

## 2. Auto-Grading Contract

### Input
- **`student_submission_dir`**: Path to the directory containing the student's submitted code and files.
- **`project_definition_config.yaml`**: A configuration file detailing:
    - **Expected ROS 2 Interfaces**: List of topics, services, actions, and their expected message/service/action types, and content patterns.
    - **Simulation Environment**: Path to Gazebo world file, robot model (URDF/SDF), and initial simulation state.
    - **Test Cases**: Specific scenarios to run against the student's code (e.g., "command robot to move to X, Y", "send service request Z").
    - **Success Metrics**: Criteria for passing each test case (e.g., "robot arrived at pose within tolerance X", "service response matches Y").
    - **Timeout**: Maximum execution time for the student's code.

### Output (from Auto-Grading Script)
- **`grade_report.json`**: A JSON file summarizing the assessment results.
    - **`total_score`** (float): Overall score (e.g., out of 100).
    - **`test_case_results`** (array of objects):
        - **`test_name`** (string): Name of the test case.
        - **`status`** (string): "PASS", "FAIL", "ERROR", "TIMEOUT".
        - **`score`** (float): Score for this test case.
        - **`feedback`** (string): Detailed feedback for the student (e.g., "Robot did not reach target pose", "Service call returned incorrect value").
    - **`logging_output`** (string, optional): Captured logs/console output from student code during execution.
    - **`error_messages`** (string, optional): Any compilation or runtime errors encountered.

### Auto-Grading Mechanism
- The auto-grading script will be a Python-based ROS 2 package.
- It will launch the student's ROS 2 nodes, the simulation environment (if applicable), and then execute defined test cases.
- It will monitor ROS 2 topics, services, and action states to verify expected behavior and compare against success metrics.
- Execution will be sandboxed to prevent malicious code from affecting the grading environment.
- The auto-grader MUST provide clear, actionable feedback to students based on their submission.