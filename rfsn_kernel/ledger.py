from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Set

from .types import Action


@dataclass
class Ledger:
    """
    Minimal seriality enforcement.
    In a real system: append-only log with fsync + hash chain.
    """
    last_seq: int = 0
    seen_action_ids: Set[str] = None

    def __post_init__(self) -> None:
        if self.seen_action_ids is None:
            self.seen_action_ids = set()

    def can_apply(self, action: Action) -> bool:
        if action.kind == "EMERGENCY_STOP":
            return True
        if action.seq != self.last_seq + 1:
            return False
        if action.action_id is not None and action.action_id in self.seen_action_ids:
            return False
        return True

    def apply(self, action: Action) -> None:
        if not self.can_apply(action):
            raise ValueError("Ledger ordering violation")
        if action.kind != "EMERGENCY_STOP":
            self.last_seq = action.seq
        if action.action_id is not None:
            self.seen_action_ids.add(action.action_id)
