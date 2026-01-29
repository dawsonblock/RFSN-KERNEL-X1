from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional

from .monitors import SafetyEvent, SafetyLevel


@dataclass
class MonitorRegistry:
    """
    Merging logic for multiple safety monitors.
    Rule: The highest severity level always wins (Worst-Case Merging).
    """
    active_events: Dict[str, SafetyEvent] = None

    def __post_init__(self):
        if self.active_events is None:
            self.active_events = {}

    def update(self, source_id: str, event: SafetyEvent) -> None:
        self.active_events[source_id] = event

    def aggregate(self) -> SafetyEvent:
        """
        Deterministic O(N) merge.
        Returns a single SafetyEvent representative of the worst system state.
        """
        if not self.active_events:
            return SafetyEvent(SafetyLevel.NONE, "no_monitors")

        worst_level = SafetyLevel.NONE
        worst_reasons = []
        affected_map: Dict[str, str] = {}

        # Priority map for comparison
        severity = {
            SafetyLevel.NONE: 0,
            SafetyLevel.WARN: 1,
            SafetyLevel.STOP: 2,
            SafetyLevel.E_STOP: 3,
        }

        for src, evt in sorted(self.active_events.items()):
            # Update worst level
            if severity[evt.level] > severity[worst_level]:
                worst_level = evt.level
                worst_reasons = [f"{src}:{evt.reason}"]
            elif severity[evt.level] == severity[worst_level] and evt.level != SafetyLevel.NONE:
                worst_reasons.append(f"{src}:{evt.reason}")

            # Merge affected spaces if this event is significant
            if evt.affected_spaces and severity[evt.level] >= severity[SafetyLevel.STOP]:
                for space, reason in evt.affected_spaces.items():
                    # If multiple reasons for one space, concatenate them deterministically
                    if space in affected_map:
                        affected_map[space] += f";{src}:{reason}"
                    else:
                        affected_map[space] = f"{src}:{reason}"

        return SafetyEvent(
            level=worst_level,
            reason=" | ".join(worst_reasons),
            affected_spaces=affected_map if affected_map else None,
        )
