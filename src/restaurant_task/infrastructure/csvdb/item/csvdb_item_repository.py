from restaurant_task.domain.models.item.item import Item
from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.category import Category
from restaurant_task.domain.models.item.item_identity import ItemIdentity
from restaurant_task.domain.models.item.i_item_repository import IItemRepository

from restaurant_task.infrastructure.csvdb.csvdb import CSVDB

class CSVDBItemRepository(IItemRepository):
    def __init__(self) -> None:
        self._db = CSVDB("items", ["id", "item_id", "category"])

    async def find_by_id(self, item_id: ItemId) -> Item | None:
        record = self._db.get(item_id.value)

        if not record:
            return None

        identity = ItemIdentity(item_id)
        category = Category(record["category"])

        return Item.reconstruct(identity, category)

    async def get_all_ids(self) -> list[ItemId]:
        return [ItemId(str_id) for str_id in self._db.get_all_ids()]
