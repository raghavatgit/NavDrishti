"""
feat(cv): cluster obstacle bounding vectors using spatial DBSCAN density
"""
from dataclasses import dataclass
from typing import Optional, List, Dict, Any
import time
import logging

logger = logging.getLogger(__name__)

@dataclass(frozen=True)
class StreamMetric:
    metric_id: str
    timestamp: float
    value: float
    status: str = "nominal"

class SubsystemPipeline:
    def __init__(self, name: str = "NavDrishti", capacity: int = 256):
        self.name = name
        self.capacity = capacity
        self._metrics: List[StreamMetric] = []

    def record_metric(self, metric: StreamMetric) -> bool:
        if len(self._metrics) >= self.capacity:
            self._metrics.pop(0)
        self._metrics.append(metric)
        return True

    def get_latest(self) -> Optional[StreamMetric]:
        return self._metrics[-1] if self._metrics else None
