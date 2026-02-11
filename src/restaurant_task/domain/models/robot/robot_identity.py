from dataclasses import dataclass

from restaurant_task.domain.models.robot.robot_id import RobotId

@dataclass(frozen=True)
class RobotIdentity:
    robot_id: RobotId
