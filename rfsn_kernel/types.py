from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple


class PerceptionTrust(str, Enum):
    VALID = "VALID"
    DEGRADED = "DEGRADED"
    UNTRUSTED = "UNTRUSTED"


class Phase(str, Enum):
    IDLE = "IDLE"
    APPROACH = "APPROACH"
    ALIGN = "ALIGN"
    GRASP = "GRASP"
    LIFT = "LIFT"
    RETREAT = "RETREAT"
    RECOVERY = "RECOVERY"
    TERMINATED = "TERMINATED"


class ActionKind(str, Enum):
    ENABLE_SKILL = "ENABLE_SKILL"
    DISABLE_SKILL = "DISABLE_SKILL"
    SET_GOAL = "SET_GOAL"
    SET_PHASE = "SET_PHASE"
    APPLY_ENVELOPE = "APPLY_ENVELOPE"
    EMERGENCY_STOP = "EMERGENCY_STOP"


@dataclass(frozen=True)
class Timestamped:
    value: object
    t: float  # seconds (monotonic clock)


@dataclass(frozen=True)
class StateSnapshot:
    """
    Gate NEVER consumes raw streaming signals. It consumes a snapshot with:
      - per-field timestamps
      - bounded skew invariant validated by the gate
    """
    t_kernel: float  # time gate is evaluating this snapshot

    # Minimal robotics-ish signals (extend as needed)
    joints_q: Timestamped              # tuple[float, ...]
    joints_qd: Timestamped             # tuple[float, ...]
    ee_pose: Timestamped               # tuple[x,y,z, qw,qx,qy,qz] or None
    contacts: Timestamped              # dict[str,bool] e.g. {"left_foot": True}
    perception_trust: Timestamped      # PerceptionTrust

    # Kernel-owned state (must be consistent with ledger)
    phase: Phase
    seq: int  # last committed sequence id

    # Environment fingerprint for envelope scoping (can be richer)
    env_fingerprint: str  # e.g. "lab_v1|camrig_v3|lighting_industrial"


@dataclass(frozen=True)
class Envelope:
    """
    Precomputed safety bounds (no simulation).
    Keep these monotone-checkable: box bounds, max speed, max joint range, etc.
    """
    name: str
    env_scope_prefix: str  # must match StateSnapshot.env_fingerprint prefix
    max_snapshot_skew_s: float
    max_state_staleness_s: float

    # Joint and velocity hard bounds
    q_min: Tuple[float, ...]
    q_max: Tuple[float, ...]
    qd_abs_max: Tuple[float, ...]

    # End-effector workspace (optional if ee_pose is present)
    ee_xyz_min: Tuple[float, float, float]
    ee_xyz_max: Tuple[float, float, float]

    # Perception policy
    allow_new_commits_when_degraded: bool
    allow_new_commits_when_untrusted: bool  # usually False

    # Phase transitions allowed
    allowed_phase_edges: Tuple[Tuple[Phase, Phase], ...]

    # Authority partitioning (primary control space owner)
    # Example: {"legs": "balance", "arm": "reach"}
    primary_authority: Dict[str, str]

    # NEW: Acceleration/Dynamics limits
    # If None, infinite acceleration is allowed (risky)
    q_acc_abs_max: Optional[Tuple[float, ...]] = None

    # NEW: Exclusion Zones (List of (min_xyz, max_xyz) tuples)
    # Example: [((-0.1, -0.1, 0), (0.1, 0.1, 0.5))] defines a forbidden pillar
    # Using simple AABBs for O(N) decidability
    exclusion_zones: Optional[List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]] = None


@dataclass(frozen=True)
class Action:
    kind: ActionKind
    seq: int  # proposed next seq; must be state.seq + 1 (except E-STOP)

    # generic payload fields (interpret by kind)
    skill_name: Optional[str] = None
    goal: Optional[Dict[str, object]] = None
    next_phase: Optional[Phase] = None
    envelope_name: Optional[str] = None

    # For deterministic ordering / replay protection
    # A planner can propose the same action twice; gate/ledger should reject duplicates.
    action_id: Optional[str] = None
