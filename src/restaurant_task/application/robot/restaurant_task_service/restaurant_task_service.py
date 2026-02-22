from dataclasses import dataclass

from restaurant_task.config import config

from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent
from restaurant_task.domain.models.order.i_order_repository import IOrderRepository
from restaurant_task.domain.models.order.order_id import OrderId
from restaurant_task.domain.models.item.i_item_repository import IItemRepository

from restaurant_task.application.robot.wait_door_open_service.wait_door_open_service import WaitDoorOpenCommand, WaitDoorOpenService
from restaurant_task.application.robot.navigate_to_spot_service.navigate_to_spot_service import NavigateToSpotCommand, NavigateToSpotService
from restaurant_task.application.robot.take_order_service.take_order_service import TakeOrderCommand, TakeOrderService
from restaurant_task.application.robot.grasp_item_service.grasp_item_service import GraspItemCommand, GraspItemService
from restaurant_task.application.robot.serve_item_service.serve_item_service import ServeItemService

@dataclass(frozen=True)
class RestaurantTaskCommand:
    seat_number: int

class RestaurantTaskService:
    def __init__(
            self,
            robot_agent: IRobotAgent,
            order_repository: IOrderRepository,
            item_repostiroy: IItemRepository,
            wait_door_open_service: WaitDoorOpenService,
            navigate_to_spot_service: NavigateToSpotService,
            take_order_service: TakeOrderService,
            grasp_item_service: GraspItemService,
            serve_item_service: ServeItemService
        ) -> None:
        self._robot = robot_agent
        self._order_repository = order_repository
        self._item_repository = item_repostiroy
        self._wait_door_open_service = wait_door_open_service
        self._navigate_to_spot_service = navigate_to_spot_service
        self._take_order_service = take_order_service
        self._grasp_item_service = grasp_item_service
        self._serve_item_service = serve_item_service

    def execute(self, command: RestaurantTaskCommand) -> None:
        seat_number = command.seat_number
        seat_spot_id = f"seat_{seat_number}"

        self._robot.speak(config.message.START_TASK)

        self._wait_door_open_service.execute(
            WaitDoorOpenCommand(max_distance_meter=1.0)
        )

        self._navigate_to_spot_service.execute(
            NavigateToSpotCommand(seat_spot_id)
        )

        order_dto = \
            self._take_order_service.execute(
                TakeOrderCommand(seat_number)
            )

        order = self._order_repository.find_by_id(OrderId(order_dto.order_id))
        if not order:
            raise ValueError("Order not found")

        item = self._item_repository.find_by_id(order.item_id)
        kitchen = "kitchen_1" if item.category.value == "snack" else "ketchen_2"

        self._navigate_to_spot_service.execute(
            NavigateToSpotCommand(kitchen)
        )

        order = self._order_repository.find_by_id(
            OrderId(order_dto.order_id)
        )
        if not order:
            raise ValueError(f"Order: {order_dto.order_id}")

        self._grasp_item_service.execute(
            GraspItemCommand(order.item_id.value)
        )

        self._navigate_to_spot_service.execute(
            NavigateToSpotCommand(seat_spot_id)
        )

        self._serve_item_service.execute()

        self._navigate_to_spot_service.execute(
            NavigateToSpotCommand("exit")
        )
