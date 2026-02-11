from dataclasses import dataclass

from restaurant_task.domain.models.task_manager.task_id import TaskId

@dataclass(frozen=True)
class TaskIdentity:
    task_id: TaskId
