from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .controller_types import CommandKind, ControlSpace


@dataclass(frozen=True)
class HoldPolicy:
    """
    Defines HOLD semantics per space per kind.

    HOLD vector semantics:
      - POSITION hold uses now_q by default
      - VELOCITY/TORQUE hold uses 0.0 by default

    You can override per space:
      - e.g. ARM prefers position hold, LEGS prefers velocity hold
    """
    # Space -> preferred hold kind (used when no command exists for that space)
    preferred_hold_kind: Dict[ControlSpace, CommandKind]

    # Optional per-space overrides for hold value generation
    # If not provided, defaults apply.
    # Example: for VELOCITY, you might want a small damping value; keep default 0.0 here.
    # Keeping this minimal and deterministic.
    _reserved: Optional[dict] = None


def default_hold_policy() -> HoldPolicy:
    return HoldPolicy(
        preferred_hold_kind={
            ControlSpace.ARM: CommandKind.JOINT_POSITION,   # hold arm at current q
            ControlSpace.LEGS: CommandKind.JOINT_VELOCITY,  # hold legs by zero velocity
            ControlSpace.BASE: CommandKind.JOINT_VELOCITY,  # hold base by zero velocity
            ControlSpace.WHOLE_BODY: CommandKind.JOINT_VELOCITY,
        }
    )
