import yaml
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import utils
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.config import config

class NavigateToPoseClient(Node):
    @utils.with_rclpy
    def __init__(self) -> None:
        super().__init__("navigate_to_pose")
        self._action_client = \
            ActionClient(self, NavigateToPose, config.msg.NAVIGATE_TO_POSE)

        if not self._action_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error(
                "Cannot connect to navigate to pose server."
            )

        self._locations = _Nav2Loc("./e301.yaml")

    def execute(self, pose_name: str) -> bool:
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = "map"
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = self._locations.loc2pose(pose_name)

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        # ゴールの待機
        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            self.get_logger().error('Goal was rejected.')
            return False
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        result = get_result_future.result()

        # 結果
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            return True
        return False

class _Nav2Loc():
    """
    地点名を入力することでナビゲーションします．
    """
    def __init__(self, poses_file_path: str) -> None:
        with open(poses_file_path, 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
            self.locations = data["location_pose"]

    def loc2pose(self, location_name: str) -> Pose:
        """地点名を座標に変換します．"""

        # 辞書型情報を座標情報に変換
        loc = self.locations[location_name]
        pose = Pose()
        pose.position = Point(
            x=loc["translation"]["x"],
            y=loc["translation"]["y"],
            z=loc["translation"]["z"]
        )
        pose.orientation = Quaternion(
            x=loc["rotation"]["x"],
            y=loc["rotation"]["y"],
            z=loc["rotation"]["z"],
            w=loc["rotation"]["w"]
        )

        return pose

navigate_to_pose = NavigateToPoseClient()
