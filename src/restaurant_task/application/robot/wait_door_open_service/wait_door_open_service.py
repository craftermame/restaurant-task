import time
from dataclasses import dataclass

from restaurant_task.config import config

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

@dataclass(frozen=True)
class WaitDoorOpenCommand:
    max_distance_meter: float

class WaitDoorOpenService:
    def __init__(self, robot_agent: IRobotAgent) -> None:
        self._robot = robot_agent

    def execute(self, command: WaitDoorOpenCommand) -> None:
        max_distance = command.max_distance_meter

        obstacle = True
        while obstacle:
            time.sleep(2.0)
            obstacle = self._robot.detect_obstacle(max_distance)
            self._robot.speak(config.message.WAIT_DOOR_OPEN)

        self._robot.speak(config.message.DOOR_OPENING_DETECTED)
