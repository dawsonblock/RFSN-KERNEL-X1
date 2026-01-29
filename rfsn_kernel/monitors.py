from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional


class SafetyLevel(str, Enum):
    NONE = "NONE"
    WARN = "WARN"
    STOP = "STOP"
    E_STOP = "E_STOP"


@dataclass(frozen=True)
class SafetyEvent:
    """
    Monitors emit these. Keep it simple and serializable.
    """
    level: SafetyLevel
    reason: str
    # Optional per-space detail. Example: {"arm": "collision_margin", "base": "cliff_detected"}
    affected_spaces: Optional[Dict[str, str]] = None
