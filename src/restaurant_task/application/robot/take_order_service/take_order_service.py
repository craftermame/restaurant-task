from dataclasses import dataclass

from restaurant_task.config import config

from restaurant_task.domain.models.order.order import Order
from restaurant_task.domain.models.order.order_id import OrderId
from restaurant_task.domain.models.order.order_identity import OrderIdentity
from restaurant_task.domain.models.order.i_order_repository import IOrderRepository
from restaurant_task.domain.models.order.seat_number import SeatNumber
from restaurant_task.domain.models.item.i_item_repository import IItemRepository
from restaurant_task.domain.models.robot.i_robot_agent import IRobotAgent
from restaurant_task.domain.services.item.item_id_extraction_domain_service import ItemIdExtractionDomainService

from restaurant_task.application.robot.take_order_service.take_order_dto import TakeOrderDTO

@dataclass(frozen=True)
class TakeOrderCommand:
    seat_number: int

class TakeOrderService:
    def __init__(
            self,
            robot_agent: IRobotAgent,
            order_repository: IOrderRepository,
            item_repository: IItemRepository
        ) -> None:
        self._robot = robot_agent
        self._order_repository = order_repository
        self._item_repository = item_repository
        self._item_id_extraction_service = \
            ItemIdExtractionDomainService(self._item_repository)

    def execute(self, command: TakeOrderCommand) -> TakeOrderDTO:
        order_id = OrderId.genereate()
        seat_number = SeatNumber(command.seat_number)

        while True:
            try:
                raw_order = \
                    self._robot.ask(config.message.ORDER_QUESTION, 6)
                item_ids = \
                    self._item_id_extraction_service.from_sentence(
                        raw_order
                    )
                for item_id in item_ids:
                    response = self._robot.ask(
                        f"Is your order {item_id.value}? Yes or No."
                    )
                    low_res = response.lower()
                    if "yes" in low_res and "no" not in low_res:
                        break
                else:
                    continue
                break
            except ValueError:
                continue

        order_identity = OrderIdentity(order_id)
        order = Order(
            order_identity,
            item_id,
            seat_number
        )

        self._order_repository.save(order)

        return TakeOrderDTO(order_id.value)
