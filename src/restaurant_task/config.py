from dataclasses import dataclass

# 発話関連

class MessageConfig:
    START_TASK = "Starts the task."
    DOOR_OPENING_DETECTED = "Door opening detected."
    WAIT_DOOR_OPEN = "Waiting for the door opened."
    ORDER_QUESTION = "What's your order?"
    SERVE_ITEM = "Serving the item."
    GIVE_ITEM = "Please receive the item."
    TEST = 'hi'

# パラメータ関連

class ParamsConfig:
    ITEM_FIND_TRIALS = 3

# Export

@dataclass(frozen=True)
class Config:
    message = MessageConfig()
    params = ParamsConfig()

config = Config()

print(config.message.TEST)
