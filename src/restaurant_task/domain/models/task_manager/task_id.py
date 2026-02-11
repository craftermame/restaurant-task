from dataclasses import dataclass

@dataclass(frozen=True)
class TaskId:
    value: str

    def __post_init__(self):
        stripped_value = self.value.strip()

        if not stripped_value:
            raise ValueError("TaskId は空にできません")

        object.__setattr__(self, "value", stripped_value)
