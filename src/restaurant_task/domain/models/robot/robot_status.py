from dataclasses import dataclass
from typing import Literal, get_args

RobotStatusType = Literal[
    "READY", "WAITING", "NAVIGATING", "GRASPING", "MOVING",
    "SPEAKING", "LISTENING", "SERVING", "FINDING_ITEM", "FINDING_HAND_RISER"
]

@dataclass(frozen=True)
class RobotStatus:
    value: RobotStatusType

    def __post_init__(self):
        stripped_value = self.value.strip()

        if not stripped_value in get_args(RobotStatusType):
            raise ValueError(f"RobotStatus {stripped_value} は存在しません")

        object.__setattr__(self, "value", stripped_value)
