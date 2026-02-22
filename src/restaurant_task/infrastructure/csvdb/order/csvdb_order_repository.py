from restaurant_task.domain.models.order.i_order_repository import IOrderRepository
from restaurant_task.domain.models.order.order import Order
from restaurant_task.domain.models.order.order_id import OrderId
from restaurant_task.domain.models.order.order_identity import OrderIdentity
from restaurant_task.domain.models.order.seat_number import SeatNumber
from restaurant_task.domain.models.item.item_id import ItemId

from restaurant_task.infrastructure.csvdb.csvdb import CSVDB

class CSVDBOrderRepository(IOrderRepository):
    def __init__(self) -> None:
        self._db = CSVDB("orders", ["id", "order_id", "item_id", "seat_number"])

    def save(self, order: Order) -> None:
        order_id = order.order_id.value
        item_id = order.item_id.value
        seat_number = str(order.seat_number.value)

        self._db.set(order_id, {
            "order_id": order_id,
            "item_id": item_id,
            "seat_number": seat_number
        })

    def find_by_id(self, order_id: OrderId) -> Order | None:
        record = self._db.get(order_id.value)

        if not record:
            return None

        identity = OrderIdentity(order_id)
        item_id = ItemId(record["item_id"])
        seat_number = SeatNumber(int(record["seat_number"]))

        return Order.reconstruct(identity, item_id, seat_number)
