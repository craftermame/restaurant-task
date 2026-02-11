from dataclasses import dataclass

from restaurant_task.domain.models.item.item_id import ItemId

@dataclass(frozen=True)
class ItemIdentity:
    item_id: ItemId
