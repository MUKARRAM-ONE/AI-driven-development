import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import openai
import os
import json
import logging

class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner')
        self.subscription = self.create_subscription(
            String,
            '/voice_command/text',
            self.command_callback,
            10
        )
        self.action_sequence_publisher = self.create_publisher(String, '/robot_action_sequence', 10)
        self.get_logger().info('Cognitive Planner node started.')

        openai.api_key = os.getenv("OPENAI_API_KEY", "YOUR_OPENAI_API_KEY") # Replace YOUR_OPENAI_API_KEY if not using env var

        self.robot_context = {
            "current_location": "living_room",
            "available_objects": ["apple", "cup", "book"],
            "known_locations": {"kitchen": (5.0, 2.0, 90.0), "bedroom": (-3.0, 4.0, 0.0), "table": (1.0, 1.0, 0.0)}
        }

    def command_callback(self, msg):
        command_text = msg.data
        self.get_logger().info(f"Received voice command: '{command_text}'")

        if openai.api_key == "YOUR_OPENAI_API_KEY" or not openai.api_key:
             self.get_logger().error("OpenAI API key not set. Please set OPENAI_API_KEY environment variable.")
             return
        
        # Construct prompt for the LLM
        prompt = self.create_llm_prompt(command_text)

        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", # or gpt-4
                messages=[
                    {"role": "system", "content": "You are a robot task planner. Convert commands into ROS 2 actions."},
                    {"role": "user", "content": prompt}
                ],
                max_tokens=200,
                temperature=0.0 # Make it deterministic
            )
            llm_response_content = response.choices[0].message.content.strip()
            self.get_logger().info(f"LLM Raw Response: {llm_response_content}")

            # Robustly parse LLM response for JSON
            action_sequence = self.parse_llm_response(llm_response_content)
            
            if action_sequence:
                # Publish the action sequence for execution
                action_msg = String()
                action_msg.data = json.dumps(action_sequence)
                self.action_sequence_publisher.publish(action_msg)
                self.get_logger().info("Published ROS 2 action sequence.")
            else:
                self.get_logger().warn("LLM did not return a valid action sequence or returned an empty one.")

        except Exception as e:
            self.get_logger().error(f"Error calling LLM API or parsing response: {e}")

    def create_llm_prompt(self, command_text):
        # This is where prompt engineering happens
        # It should include current robot context and available actions
        prompt = f"""You are a robot task planner. Your goal is to convert natural language commands into a sequence of ROS 2 actions.
Available ROS 2 Actions (use the coordinates from known_locations if a location is mentioned):
- navigate_to_pose(x:float, y:float, yaw_degrees:float): Navigate to a 2D pose.
- pick_up_object(object_name:str): Pick up a specified object.
- find_object(object_name:str): Find a specified object using perception.
- say_phrase(phrase:str): Make the robot speak a phrase.

Current robot context:
  Location: {self.robot_context['current_location']}
  Known locations: {json.dumps(self.robot_context['known_locations'])}
  Available objects: {json.dumps(self.robot_context['available_objects'])}

Respond ONLY with a JSON array where each object is an action. Do not include any other text or explanations.
If a location is mentioned, use its coordinates from 'known_locations'.
If you cannot fulfill the command with available actions or need clarification, return an empty array or an action to 'say_phrase' asking for clarification.

User command: "{command_text}"
Response:
"""
        return prompt
    
    def parse_llm_response(self, raw_response):
        """
        Parses the raw LLM response, attempting to extract a JSON array.
        Handles cases where LLM might include conversational text around JSON.
        """
        try:
            # Attempt to find the first and last curly braces to extract potential JSON
            json_start = raw_response.find('[')
            json_end = raw_response.rfind(']')
            if json_start != -1 and json_end != -1:
                json_str = raw_response[json_start : json_end + 1]
                return json.loads(json_str)
            else:
                self.get_logger().warn("LLM response did not contain a valid JSON array string.")
                return []
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Failed to decode JSON from LLM response: {e}. Raw: {raw_response}")
            return []

def main(args=None):
    logging.basicConfig(level=logging.INFO)
    rclpy.init(args=args)
    cognitive_planner_node = CognitivePlannerNode()
    rclpy.spin(cognitive_planner_node)
    cognitive_planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
