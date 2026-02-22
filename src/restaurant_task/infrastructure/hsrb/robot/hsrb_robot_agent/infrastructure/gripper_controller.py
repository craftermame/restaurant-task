from typing import Literal

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from tmc_control_msgs.action import GripperApplyEffort

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

CommandOption = Literal["open", "close"]

class GripperControllerClient(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("gripper")
        self.action_client = ActionClient(
            self,
            GripperApplyEffort,
            config.msg.GRIPPER
        )

        if not self.action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Cannot connect to gripper controller server")

    def execute(self, command: CommandOption):
        effort = 0.1 if command == "open" else -0.1

        goal_msg = GripperApplyEffort.Goal()
        goal_msg.effort = effort
        goal_msg.do_control_stop = False

        send_goal_future = self.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future, timeout_sec=3.0)

gripper_controller = GripperControllerClient()
