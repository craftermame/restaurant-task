from typing import Final

class MsgConfig:
    CMD_VEL: Final = "/omni_base_controller/cmd_vel"
    WHEEL_ODOM: Final = "/omni_base_controller/wheel_odom"
    GRIPPER: Final = "/gripper_controller/grasp"
    LASER_SCAN: Final = "/scan"
    MOVE_TO_POSE: Final = "/hsrb/move_to_pose"
    NAVIGATE_TO_POSE: Final = "/navigate_to_pose"

class Config:
    msg = MsgConfig()

config = Config()
