from dataclasses import dataclass

from restaurant_task.config import config

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent
from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.i_item_repository import IItemRepository
from restaurant_task.domain.models.item.error import ItemNotFoundError

@dataclass(frozen=True)
class GraspItemCommand:
    item_id: str

class GraspItemService:
    def __init__(
            self,
            robot_agent: IRobotAgent,
            item_repository: IItemRepository
        ) -> None:
        self._robot = robot_agent
        self._item_repository = item_repository

    async def execute(self, command: GraspItemCommand) -> None:
        item = await self._item_repository.find_by_id(ItemId(command.item_id))

        if not item:
            raise ItemNotFoundError(f"ItemId: {command.item_id} is not exists.")

        await self._robot.speak(f"I'll grasp the {item.item_id.value}.")
        for _ in range(config.params.ITEM_FIND_TRIALS):
            await self._robot.speak(f"Finding the {item.item_id.value}.")
            physical_item = await self._robot.find_item(item)

            distance_x = physical_item.coordinate.x
            distance_y = physical_item.coordinate.y
            await self._robot.move(distance_x, distance_y)
            # TODO: 手の画像とかつかってもいいかも、前進するみたいな感じで
            # 画像っていうのは、商品に衝突して画像が黒くなることを検知する的な
            # だから、まずは x を決めて前進。を数回繰り返す。手が衝突すると画像が変わらなくなる
            # これを検知して把持判定とか？

        await self._robot.speak(f"Grasp the {item.item_id.value}.")
        await self._robot.open_hand()
        await self._robot.set_pose("grasp")
        await self._robot.close_hand()
        await self._robot.set_pose("default")
