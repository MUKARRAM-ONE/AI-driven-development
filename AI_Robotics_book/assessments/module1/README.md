# Module 1 Assessment: ROS 2 Topic Relay

## Objective

This assessment is designed to test your understanding of basic ROS 2 concepts, including nodes, topics, publishers, and subscribers.

## Task

Create a ROS 2 Python node that:
1.  Subscribes to the `/input_topic` (message type: `std_msgs/String`).
2.  Processes the received message by appending " (processed)" to the string.
3.  Publishes the processed string to the `/output_topic` (message type: `std_msgs/String`).

## Files to Submit

-   A Python file containing your ROS 2 node (e.g., `topic_relay_node.py`).
-   The `package.xml` and `setup.py` files for your ROS 2 package.

## Auto-Grading

Your submission will be evaluated by an auto-grading script (`grade.py`). The script will:
1.  Launch your `topic_relay_node`.
2.  Publish a series of test messages to `/input_topic`.
3.  Subscribe to `/output_topic` and verify that the received messages match the expected processed output.
4.  Provide a grade report (`grade_report.json`) with your score and feedback.

Refer to the `project_definition.yaml` file for the exact topic names and message types that the grader will use.
