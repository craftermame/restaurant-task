from typing import Protocol

from restaurant_task.domain.models.order.order import Order
from restaurant_task.domain.models.order.order_id import OrderId

class IOrderRepository(Protocol):
    def save(self, order: Order) -> None:...
    def find_by_id(self, order_id: OrderId) -> Order | None:...
