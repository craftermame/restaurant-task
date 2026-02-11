from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.order.order_id import OrderId
from restaurant_task.domain.models.order.order_identity import OrderIdentity
from restaurant_task.domain.models.order.seat_number import SeatNumber

class Order:
    def __init__(
            self,
            identity: OrderIdentity,
            item_id: ItemId,
            seat_number: SeatNumber
        ) -> None:
        self._indentity = identity
        self._item_id = item_id
        self._seat_number = seat_number

    def __eq__(self, value: "Order") -> bool:
        return self._indentity == value._indentity

    @property
    def order_id(self) -> OrderId:
        return self._indentity.order_id

    @property
    def item_id(self) -> ItemId:
        return self._item_id

    @property
    def seat_number(self) -> SeatNumber:
        return self._seat_number

    @classmethod
    def create(
            cls,
            identity: OrderIdentity,
            item_id: ItemId,
            seat_number: SeatNumber
        ) -> "Order":
        return cls(identity, item_id, seat_number)

    @classmethod
    def reconstruct(
            cls,
            identity: OrderIdentity,
            item_id: ItemId,
            seat_number: SeatNumber
        ) -> "Order":
        return cls(identity, item_id, seat_number)
