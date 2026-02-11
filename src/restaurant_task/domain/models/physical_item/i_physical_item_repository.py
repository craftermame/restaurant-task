from typing import Protocol

from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.physical_item.physical_item import PhysicalItem

class IPhysicalItemRepository(Protocol):
    async def save(self, physical_item: PhysicalItem) -> None:...
    async def find_by_id(self, item_id: ItemId) -> PhysicalItem | None:...
