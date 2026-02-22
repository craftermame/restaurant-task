from restaurant_task.domain.models.item.item import Item, ItemId, ItemIdentity, Category

from restaurant_task.infrastructure.csvdb.item.csvdb_item_repository import CSVDBItemRepository

def test_csvdb_item_repository():
    csvdb_item_repository = CSVDBItemRepository()

    item_id = ItemId("coffee")
    item = csvdb_item_repository.find_by_id(item_id)

    expected_item = Item(
        ItemIdentity(ItemId("coffee")),
        Category("drink")
    )

    assert item == expected_item

    ids = csvdb_item_repository.get_all_ids()
    ids_str = ["caramel_corn", "cookie", "potato_chips", "cola", "green_tea", "coffee"]
    excepted_ids = [ItemId(item_id_str) for item_id_str in ids_str]

    assert set(ids) == set(excepted_ids)
