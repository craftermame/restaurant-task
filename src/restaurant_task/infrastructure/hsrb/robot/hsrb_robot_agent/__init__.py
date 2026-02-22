from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.supertonic import supertonic_client
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.vosk import vosk_client
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.cmd_vel import cmd_vel
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.wheel_odom import wheel_odom
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.gripper_controller import gripper_controller
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.laser_scan import laser_scan
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.move_to_pose import move_to_pose
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.infrastructure.navigate_to_pose import navigate_to_pose

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.application.move_service import MoveService
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.application.detect_obstacle_service import DetectObstacleService
from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent.application.item_pose_service import ItemPoseSerivce
from restaurant_task.domain.models.item.i_item_repository import IItemRepository
from restaurant_task.domain.models.physical_item.i_physical_item_repository import IPhysicalItemRepository

move_service = MoveService(cmd_vel, wheel_odom)
detect_obstacle_service = DetectObstacleService(laser_scan)

class HSRBRobotAgent(IRobotAgent):
    def __init__(
            self,
            item_repository: IItemRepository,
            physical_item_repository: IPhysicalItemRepository
        ):
        self._item_repository = item_repository
        self._physical_item_repository = physical_item_repository

    def speak(self, sentence):
        supertonic_client.execute(sentence)

    def ask(self, question, duration_sec = 4):
        self.speak(question)
        return vosk_client.execute(duration_sec)

    def move_to(self, spot_id):
        return navigate_to_pose.execute(spot_id.value)

    def move(self, x, y):
        move_service.execute(x, y)

    def open_hand(self):
        gripper_controller.execute("open")

    def close_hand(self):
        gripper_controller.execute("close")

    def detect_obstacle(self, max_distance):
        return detect_obstacle_service.execute(max_distance)

    def set_pose(self, pose_name):
        move_to_pose.execute(pose_name)

    def find_item(self, item):
        item_pose_service = ItemPoseSerivce(self._item_repository, self._physical_item_repository)
        return item_pose_service.find_item(item.item_id.value)
