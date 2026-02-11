from restaurant_task.domain.models.task_manager.task_id import TaskId
from restaurant_task.domain.models.task_manager.task_identity import TaskIdentity
from restaurant_task.domain.models.task_manager.task_progress import TaskProgress

class Task:
    def __init__(self, identity: TaskIdentity, progress: TaskProgress) -> None:
        self._identity = identity
        self._progress = progress

    def __eq__(self, value: "Task") -> bool:
        return self._identity == value._identity

    @property
    def task_id(self) -> TaskId:
        return self._identity.task_id

    @property
    def progress(self) -> TaskProgress:
        return self._progress

    @classmethod
    def create(cls, identity: TaskIdentity, progress: TaskProgress) -> "Task":
        return cls(identity, progress)

    @classmethod
    def reconstruct(
            cls,
            identity: TaskIdentity,
            progress: TaskProgress
        ) -> "Task":
        return cls(identity, progress)
