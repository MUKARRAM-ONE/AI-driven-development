# VLA (Vision-Language-Action) System Contract

This document defines the interface for the Vision-Language-Action (VLA) system, particularly focusing on the interaction between natural language commands, cognitive planning (LLMs), and robot actions (ROS 2).

## 1. Voice Command Input (via OpenAI Whisper)

### Input Interface
- **Mechanism**: Audio stream from microphone.
- **Processing**: Transcribed by OpenAI Whisper.
- **Output**:
    - **`transcribed_text`** (string): The natural language command as text.

## 2. Cognitive Planning Interface (LLM Integration)

### Input
- **`natural_language_command`** (string): The transcribed text from the voice command (e.g., "Clean the room", "Pick up the red block").
- **`robot_context`** (JSON string/dict, optional): Current robot state (e.g., `current_pose`, `known_objects`, `battery_level`) and environmental context (e.g., `room_layout`, `object_locations`).

### Output
- **`action_sequence`** (JSON array of dicts): A structured sequence of discrete, executable ROS 2 actions.
    - Each action object should specify:
        - `type` (string): Type of ROS 2 interaction (e.g., "topic_publish", "service_call", "action_goal").
        - `name` (string): Name of the topic, service, or action.
        - `parameters` (JSON dict): Key-value pairs representing message/request/goal fields.
        - `expected_feedback` (string, optional): Criteria for success/failure.
- **`plan_status`** (string): "SUCCESS", "FAILURE", "CLARIFICATION_NEEDED".
- **`clarification_query`** (string, optional): If `plan_status` is "CLARIFICATION_NEEDED", a natural language question for the user (e.g., "Which room do you mean?").

### Example `action_sequence`
```json
[
  {
    "type": "action_goal",
    "name": "/navigate_to_pose",
    "parameters": {
      "target_pose": {
        "header": {"frame_id": "map"},
        "pose": {"position": {"x": 2.0, "y": 1.0, "z": 0.0}, "orientation": {"w": 1.0}}
      }
    },
    "expected_feedback": "navigation_completed"
  },
  {
    "type": "service_call",
    "name": "/look_for_object",
    "parameters": {"object_name": "red_block"},
    "expected_feedback": "object_found"
  },
  {
    "type": "action_goal",
    "name": "/pick_up_object",
    "parameters": {"object_id": "red_block_123"},
    "expected_feedback": "object_grasped"
  }
]
```

## 3. Robot Action Execution (via ROS 2)

### Input
- **`action_sequence`** (JSON array of dicts): The output from the Cognitive Planning Interface.

### Output
- **`execution_status`** (string): "COMPLETED", "FAILED", "IN_PROGRESS".
- **`current_action_status`** (JSON dict, optional): Details on the currently executing action's status.
- **`error_message`** (string, optional): Description of any execution error.
- **`robot_state_feedback`** (JSON string/dict, optional): Updated robot sensor data and pose.

## 4. Error Handling and Graceful Degradation

- The VLA system MUST implement basic retry logic for transient communication failures (e.g., with OpenAI API, ROS 2 topics).
- The VLA system SHOULD provide graceful degradation (e.g., vocalizing "I'm having trouble connecting to the map service, please try again") when critical external services are unavailable, rather than crashing silently.
- If the LLM requires clarification, the system MUST be able to query the user for more information before proceeding.