from dataclasses import dataclass
from typing import Final

# 発話関連

class MessageConfig:
    START_TASK: Final = "Starts the task."
    DOOR_OPENING_DETECTED: Final = "Door opening detected."
    WAIT_DOOR_OPEN: Final = "Waiting for the door opened."
    ORDER_QUESTION: Final = "What's your order?"
    SERVE_ITEM: Final = "Serving the item."
    GIVE_ITEM: Final = "Please receive the item."

# パラメータ関連

class ParamsConfig:
    ITEM_FIND_TRIALS: Final = 3

# Export

@dataclass(frozen=True)
class Config:
    message = MessageConfig()
    params = ParamsConfig()

config = Config()
