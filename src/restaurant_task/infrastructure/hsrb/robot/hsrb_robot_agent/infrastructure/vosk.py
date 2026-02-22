import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sobits_interfaces.action import SpeechRecognition

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils

class VoskClient(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("vosk")
        self.action_client = \
            ActionClient(self, SpeechRecognition, "/speech_recognition")

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Cannot connect to vosk server.")

    def execute(self, timeout_sec: int):
        goal_msg = SpeechRecognition.Goal()
        goal_msg.timeout_sec = timeout_sec
        goal_msg.silent_mode = False
        goal_msg.feedback_rate = 0.5

        # Goal送信
        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        # ゴールの待機
        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            self.get_logger().error('Goal was rejected.')
            return None
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        result = get_result_future.result()

        return result.result.result_text

vosk_client = VoskClient()
