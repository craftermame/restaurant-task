from dataclasses import dataclass

from restaurant_task.config import config

@dataclass(frozen=True)
class Category:
    value: str

    def __post_init__(self):
        stripped_value = self.value.strip()

        if not stripped_value:
            raise ValueError("Category は空にできません")

        if stripped_value not in config.item.categories:
            raise ValueError(f"カテゴリ {self.value} は存在しません")

        object.__setattr__(self, "value", stripped_value)
