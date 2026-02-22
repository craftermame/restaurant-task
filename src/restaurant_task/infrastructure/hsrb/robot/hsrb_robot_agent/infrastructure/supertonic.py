import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sobits_interfaces.action import TextToSpeech

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils

class SupertonicClient(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("supertonic")
        self.action_client = ActionClient(self, TextToSpeech, "/speech_word")

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Cannot connect to supertonic server.")

    def execute(self, text: str):
        goal_msg = TextToSpeech.Goal()
        goal_msg.text = text

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            return None
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

supertonic_client = SupertonicClient()
