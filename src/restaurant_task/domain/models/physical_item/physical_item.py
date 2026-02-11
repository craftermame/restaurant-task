from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.physical_item.physical_item_identity import PhysicalItemIdentity
from restaurant_task.domain.models.shared.coordinate import Coordinate

class PhysicalItem:
    def __init__(
            self,
            identity: PhysicalItemIdentity,
            coordinate: Coordinate
        ) -> None:
        self._identity = identity
        self._coordinate = coordinate

    def __eq__(self, value: "PhysicalItem") -> bool:
        return self._identity == value._identity

    @property
    def item_id(self) -> ItemId:
        return self._identity.item_id

    @property
    def coordinate(self) -> Coordinate:
        return self.coordinate

    @classmethod
    def create(
            cls,
            identity: PhysicalItemIdentity,
            coordinate: Coordinate
        ) -> "PhysicalItem":
        return cls(identity, coordinate)

    @classmethod
    def reconsturct(
            cls,
            identity: PhysicalItemIdentity,
            coordinate: Coordinate
        ) -> "PhysicalItem":
        return cls(identity, coordinate)
