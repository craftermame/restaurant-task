from restaurant_task.domain.models.item.category import Category
from restaurant_task.domain.models.item.item_id import ItemId
from restaurant_task.domain.models.item.item_identity import ItemIdentity

class Item:
    def __init__(
            self,
            identity: ItemIdentity,
            category: Category
        ) -> None:
        self._identity = identity
        self._category = category

    def __eq__(self, value: "Item") -> bool:
        return self._identity == value._identity

    @property
    def item_id(self) -> ItemId:
        return self._identity.item_id

    @property
    def category(self) -> Category:
        return self._category

    @classmethod
    def create(cls, identity: ItemIdentity, category: Category) -> "Item":
        return cls(identity, category)

    @classmethod
    def reconstruct(cls, identity: ItemIdentity, category: Category) -> "Item":
        return cls(identity, category)
