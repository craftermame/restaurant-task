from dataclasses import dataclass

@dataclass(frozen=True)
class TakeOrderDTO:
    order_id: str
