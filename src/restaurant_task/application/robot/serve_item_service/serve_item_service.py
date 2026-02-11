from restaurant_task.config import config

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

class ServeItemService:
    def __init__(self, robot_agent: IRobotAgent) -> None:
        self._robot = robot_agent

    async def execute(self) -> None:
        await self._robot.speak(config.message.SERVE_ITEM)
        await self._robot.set_pose("serve")
        await self._robot.speak(config.message.GIVE_ITEM)
        await self._robot.open_hand()
        await self._robot.sleep(3)
        await self._robot.close_hand()
        await self._robot.set_pose("default")
