from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.i_item_repository import IItemRepository

item_dict = {
    "caramel_corn": ["caramel corn", "caramel", "corn", "canada", "cat", "call", "phone", "can", "made", "goal", "medical"],
    "potato_chips": ["potato chips", "potato", "chips"],
    "cookie": ["cookie", "key", "could", "cook"],
    "green_tea": ["green tea", "green", "tea", "great"],
    "coffee": ["coffee", "cause", "he"],
    "cola": ["cola", "though", "call", "up"]
}

class ItemIdExtractionDomainService:
    def __init__(self, item_repository: IItemRepository) -> None:
        self._item_repository = item_repository

    def from_sentence(self, sentence: str) -> list[ItemId]:
        str_ids = [
            item_id.value
            for item_id in self._item_repository.get_all_ids()
        ]

        ids = []

        for str_id in str_ids:
            for candidate in item_dict[str_id]:
                if candidate in sentence:
                    ids.append(ItemId(str_id))

        if not ids:
            raise ValueError("Could not extract any ItemId.")

        return ids
