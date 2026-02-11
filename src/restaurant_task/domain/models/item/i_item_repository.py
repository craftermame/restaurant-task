from typing import Protocol

from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.item import Item

class IItemRepository(Protocol):
    async def find_by_id(self, item_id: ItemId) -> Item | None:...
    async def get_all_ids(self) -> list[ItemId]:...
