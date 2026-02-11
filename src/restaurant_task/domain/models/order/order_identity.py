from dataclasses import dataclass

from restaurant_task.domain.models.order.order_id import OrderId

@dataclass(frozen=True)
class OrderIdentity:
    order_id: OrderId
