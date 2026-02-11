from dataclasses import dataclass
from enum import Enum, auto

class TaskStatus(Enum):
    READY = auto()
    MOVINT_TO_SEAT = auto()
    TAKING_ORDER = auto()
    MOVING_TO_KITCHEN = auto()
    FINDING_ITEM = auto()
    PICKING_ITEM = auto()
    BRINGING_ITEM = auto()
    SERVING_ITEM = auto()
    MOVING_TO_ENTRANCE = auto()
    DONE = auto()

@dataclass(frozen=True)
class TaskProgress:
    value: TaskStatus
