from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.i_item_repository import IItemRepository

class ItemIdExtractionDomainService:
    def __init__(self, item_repository: IItemRepository) -> None:
        self._item_repository = item_repository

    async def from_sentence(self, sentence: str) -> ItemId:
        str_ids = [
            item_id.value
            for item_id in await self._item_repository.get_all_ids()
        ]

        for str_id in str_ids:
            if str_id in sentence:
                return ItemId(str_id)

        raise ValueError("Could not extract any ItemId.")
