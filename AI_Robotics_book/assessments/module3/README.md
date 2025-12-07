# Module 3 Assessment: Nav2 Goal Publisher

## Objective

This assessment is designed to test your ability to publish navigation goals to the Nav2 stack in ROS 2.

## Task

Create a ROS 2 Python node that:
1.  Acts as a client to the `navigate_to_pose` action server provided by Nav2.
2.  Publishes a single navigation goal to move the robot to a specified (x, y) coordinate with a specific yaw orientation.
3.  Monitors the feedback from the action server and reports whether the goal was accepted or rejected, and eventually the final result.

## Files to Submit

-   A Python file containing your ROS 2 node (e.g., `nav2_goal_client.py`).
-   The `package.xml` and `setup.py` files for your ROS 2 package.

## Auto-Grading

Your submission will be evaluated by an auto-grading script (`grade.py`). The script will:
1.  Launch your `nav2_goal_client.py` node.
2.  Simulate the Nav2 action server's responses (acceptance, feedback, and final result).
3.  Verify that your node correctly sends the goal and processes the responses.
4.  Provide a grade report (`grade_report.json`) with your score and feedback.

Refer to the `project_definition.yaml` file for the exact goal coordinates and orientation that the grader will use.
