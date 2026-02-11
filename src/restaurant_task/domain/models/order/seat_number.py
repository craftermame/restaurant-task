from dataclasses import dataclass

@dataclass(frozen=True)
class SeatNumber:
    _MIN_NUMBER = 1
    _MAX_NUMBER = 4

    value: int

    def __post_init__(self):
        if not self._MIN_NUMBER <= self.value <= self._MAX_NUMBER:
            raise ValueError(
                f"SeatNumber は {self._MIN_NUMBER} から {self._MAX_NUMBER} です"
            )
