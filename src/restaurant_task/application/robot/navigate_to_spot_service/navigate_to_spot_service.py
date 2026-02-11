from dataclasses import dataclass

from restaurant_task.domain.models.spot.spot_id import SpotId
from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent

@dataclass(frozen=True)
class NavigateToSpotCommand:
    spot_id: str

class NavigateToSpotService:
    def __init__(
            self,
            robot_agent: IRobotAgent,
        ) -> None:
        self._robot = robot_agent

    async def execute(self, command: NavigateToSpotCommand) -> None:
        spot_id = SpotId(command.spot_id)

        await self._robot.speak(f"I'll go to {spot_id}.")
        await self._robot.move_to(spot_id)
