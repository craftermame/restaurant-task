from dataclasses import dataclass
import uuid

@dataclass(frozen=True)
class OrderId:
    value: str

    def __post_init__(self):
        if not self.value.strip():
            raise ValueError("OrderId は空にできません")

        if not self.value.startswith("ord_"):
            raise ValueError("OrderId は ord_ で始まる必要があります")

    @classmethod
    def genereate(cls) -> "OrderId":
        return cls("ord_" + str(uuid.uuid1()))
