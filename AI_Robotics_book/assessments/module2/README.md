# Module 2 Assessment: Gazebo Joint Control

## Objective

This assessment is designed to test your ability to control a simulated robot's joints in Gazebo using ROS 2.

## Task

Create a ROS 2 Python node that:
1.  Connects to a simulated robot in Gazebo (e.g., via a `JointTrajectoryController`).
2.  Publishes a `JointTrajectory` message to move a specific joint (e.g., `joint1`) to a target position (e.g., 0.5 radians).
3.  Ensures the joint reaches and maintains the target position within a given tolerance.

## Files to Submit

-   A Python file containing your ROS 2 node (e.g., `joint_mover_node.py`).
-   The `package.xml` and `setup.py` files for your ROS 2 package.

## Auto-Grading

Your submission will be evaluated by an auto-grading script (`grade.py`). The script will:
1.  Launch your `joint_mover_node` in a Gazebo simulation with a test robot.
2.  Monitor the robot's joint states (e.g., via `/joint_states` topic).
3.  Verify that the specified joint reaches the target position within the allowed time and tolerance.
4.  Provide a grade report (`grade_report.json`) with your score and feedback.

Refer to the `project_definition.yaml` file for the exact joint names, target positions, and tolerances that the grader will use.
