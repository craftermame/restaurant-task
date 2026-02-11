from restaurant_task.domain.models.robot.robot_id import RobotId
from restaurant_task.domain.models.robot.robot_identity import RobotIdentity
from restaurant_task.domain.models.robot.robot_status import RobotStatus

class Robot:
    def __init__(
            self,
            identity: RobotIdentity,
            status: RobotStatus
        ) -> None:
        self._identity = identity
        self._status = status

    def __eq__(self, value: "Robot") -> bool:
        return self._identity == value._identity

    @property
    def robot_id(self) -> RobotId:
        return self._identity.robot_id

    @property
    def status(self) -> RobotStatus:
        return self._status

    @classmethod
    def create(cls, identity: RobotIdentity, status: RobotStatus) -> "Robot":
        return cls(identity, status)

    @classmethod
    def reconstruct(
            cls,
            identity: RobotIdentity,
            status: RobotStatus
        ) -> "Robot":
        return cls(identity, status)
