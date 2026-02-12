from typing import Final

class MsgConfig:
    CMD_VEL: Final = "/omni_base_controller/cmd_vel"
    WHEEL_ODOM: Final = "/omni_base_controller/wheel_odom"
    GRIPPER: Final = "/gripper_controller/grasp"
    LASER_SCAN: Final = "/scan"

class Config:
    msg = MsgConfig()

config = Config()
