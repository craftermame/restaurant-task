from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.physical_item.physical_item import PhysicalItem
from restaurant_task.domain.models.physical_item.physical_item_identity import PhysicalItemIdentity
from restaurant_task.domain.models.physical_item.i_physical_item_repository import IPhysicalItemRepository
from restaurant_task.domain.models.shared.coordinate import Coordinate

from restaurant_task.infrastructure.csvdb.csvdb import CSVDB

class CSVDBPhysicalItemRepository(IPhysicalItemRepository):
    def __init__(self) -> None:
        self._db = CSVDB("physical_items", ["id", "item_id", "x", "y", "z"])

    def save(self, physical_item: PhysicalItem) -> None:
        item_id = physical_item.item_id.value
        x = physical_item.coordinate.x
        y = physical_item.coordinate.y
        z = physical_item.coordinate.z

        self._db.set(item_id, {
            "item_id": item_id,
            "x": str(x),
            "y": str(y),
            "z": str(z)
        })

    def find_by_id(self, item_id: ItemId) -> PhysicalItem | None:
        record = self._db.get(item_id.value)

        if not record:
            return None

        identity = PhysicalItemIdentity(item_id)
        coordinate = Coordinate(
            float(record["x"]),
            float(record["y"]),
            float(record["z"])
        )

        return PhysicalItem.reconsturct(identity, coordinate)
