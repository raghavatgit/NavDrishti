import queue
from dataclasses import dataclass, field
from typing import Any

@dataclass(order=True)
class FeedbackEvent:
    priority: int
    message: str = field(compare=False)
    payload: Any = field(compare=False, default=None)

class AudioHapticScheduler:
    def __init__(self):
        self._q = queue.PriorityQueue()

    def post(self, priority: int, message: str):
        self._q.put(FeedbackEvent(priority=priority, message=message))

    def next_event(self) -> FeedbackEvent:
        return self._q.get()
