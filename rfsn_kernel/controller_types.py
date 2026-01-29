from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, Optional, Sequence, Tuple


class ControlSpace(str, Enum):
    ARM = "arm"
    LEGS = "legs"
    BASE = "base"
    WHOLE_BODY = "whole_body"


class CommandKind(str, Enum):
    JOINT_POSITION = "JOINT_POSITION"   # q_des on masked DOFs
    JOINT_VELOCITY = "JOINT_VELOCITY"   # qd_des on masked DOFs
    JOINT_TORQUE = "JOINT_TORQUE"       # tau_des on masked DOFs


@dataclass(frozen=True)
class MaskedCommand:
    """
    A command applies only to dof_mask indices into the robot's full joint vector.
    values length must match dof_mask length.

    This lets you run simultaneous spaces without double-commanding.
    """
    space: ControlSpace
    kind: CommandKind
    dof_mask: Tuple[int, ...]
    values: Tuple[float, ...]
    source: str  # skill name / "safety"

    def __post_init__(self) -> None:
        if len(self.dof_mask) != len(self.values):
            raise ValueError("MaskedCommand: dof_mask and values must have same length")
        if len(set(self.dof_mask)) != len(self.dof_mask):
            raise ValueError("MaskedCommand: dof_mask contains duplicates")
        if any(i < 0 for i in self.dof_mask):
            raise ValueError("MaskedCommand: negative DOF index")


@dataclass(frozen=True)
class CapabilityLease:
    """
    Controller enforces this lease at high rate (200–1000Hz).
    Gate only decides whether leases may be installed/changed.

    q_min/q_max/qd_abs_max are full-DOF vectors sized to the robot.
    """
    seq: int
    lease_id: str
    issued_t: float
    expiry_t: float

    q_min: Tuple[float, ...]
    q_max: Tuple[float, ...]
    qd_abs_max: Tuple[float, ...]
    tau_abs_max: Optional[Tuple[float, ...]] = None

    # Space -> primary authority skill
    primary_authority: Dict[str, str] = None
