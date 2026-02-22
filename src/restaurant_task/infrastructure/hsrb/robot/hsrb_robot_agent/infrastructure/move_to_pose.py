import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sobits_interfaces.action import MoveToPose

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

class MoveToPoseClient(Node):
    @utils.with_rclpy
    def __init__(self):
        super().__init__("move_to_pose")
        self._action_client = \
            ActionClient(self, MoveToPose, config.msg.MOVE_TO_POSE)

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error("Cannot connect to supertonic server.")

    def execute(self, pose_name: str) -> None:
        goal_msg = MoveToPose.Goal()
        goal_msg.pose_name = pose_name
        goal_msg.time_allowance.sec = 4

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            return None
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

move_to_pose = MoveToPoseClient()
