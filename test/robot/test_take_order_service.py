from restaurant_task.domain.models.order.order_id import OrderId

from restaurant_task.application.robot.take_order_service.take_order_service import TakeOrderCommand, TakeOrderService

from restaurant_task.infrastructure.hsrb.robot.hsrb_robot_agent import HSRBRobotAgent
from restaurant_task.infrastructure.csvdb.order.csvdb_order_repository import CSVDBOrderRepository
from restaurant_task.infrastructure.csvdb.item.csvdb_item_repository import CSVDBItemRepository

hsrb = HSRBRobotAgent()
order_repository = CSVDBOrderRepository()
item_repository = CSVDBItemRepository()

async def test_take_order_service():
    print("Answer 'coffee' for following question.")

    service = \
        TakeOrderService(hsrb, order_repository, item_repository)

    seat_number = 1
    result = await service.execute(TakeOrderCommand(seat_number))
    order_id = OrderId(result.order_id)
    result_order = await order_repository.find_by_id(order_id)

    if not result_order:
        raise RuntimeError("Order not found")

    assert result_order.item_id.value == "coffee"
    assert result_order.seat_number.value == seat_number
