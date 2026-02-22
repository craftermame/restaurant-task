import time

from restaurant_task.config import config

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

class ServeItemService:
    def __init__(self, robot_agent: IRobotAgent) -> None:
        self._robot = robot_agent

    def execute(self) -> None:
        self._robot.speak(config.message.SERVE_ITEM)
        self._robot.set_pose("serve")
        self._robot.speak(config.message.GIVE_ITEM)
        self._robot.open_hand()
        time.sleep(3.0)
        self._robot.close_hand()
        self._robot.set_pose("default")
