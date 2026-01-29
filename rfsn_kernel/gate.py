from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from .types import (
    Action,
    ActionKind,
    Envelope,
    PerceptionTrust,
    Phase,
    StateSnapshot,
)
from .ledger import Ledger


@dataclass(frozen=True)
class GateDecision:
    ok: bool
    reason: str
    reject_code: str


def gate(
    *,
    state: StateSnapshot,
    action: Action,
    envelope: Envelope,
    ledger: Ledger,
    enabled_skills: Dict[str, bool],
) -> GateDecision:
    """
    Deterministic, side-effect-free gate.
    No simulation, no learning, no IO, no randomness.
    """

    # 0) Emergency stop always allowed (still serially logged by controller path)
    if action.kind == ActionKind.EMERGENCY_STOP:
        return GateDecision(True, "E-STOP allowed", "OK")

    # 1) Ordering / seriality (ledger is the source of truth)
    if not ledger.can_apply(action):
        return GateDecision(False, "Ledger ordering / replay violation", "ORDER_VIOLATION")

    # 2) Envelope scope (distribution check, crude but explicit)
    if not state.env_fingerprint.startswith(envelope.env_scope_prefix):
        return GateDecision(False, "Envelope not valid for environment fingerprint", "ENV_SCOPE_MISMATCH")

    # 3) Snapshot consistency (bounded skew + staleness)
    ok, msg, code = _snapshot_time_ok(state, envelope)
    if not ok:
        return GateDecision(False, msg, code)

    # 4) Perception trust gating
    trust = state.perception_trust.value
    if trust == PerceptionTrust.DEGRADED and not envelope.allow_new_commits_when_degraded:
        return GateDecision(False, "Perception DEGRADED: no new commits allowed", "PERCEPTION_DEGRADED")
    if trust == PerceptionTrust.UNTRUSTED and not envelope.allow_new_commits_when_untrusted:
        return GateDecision(False, "Perception UNTRUSTED: no new commits allowed", "PERCEPTION_UNTRUSTED")

    # 5) State hard bounds (monotone checks, O(n))
    ok, msg, code = _state_bounds_ok(state, envelope)
    if not ok:
        return GateDecision(False, msg, code)

    # 6) Action allowlist + kind-specific validation
    ok, msg, code = _action_ok(state, action, envelope, enabled_skills)
    if not ok:
        return GateDecision(False, msg, code)

    return GateDecision(True, "Accepted", "OK")


def _snapshot_time_ok(state: StateSnapshot, env: Envelope) -> Tuple[bool, str, str]:
    # Collect timestamps
    ts = [
        state.joints_q.t,
        state.joints_qd.t,
        state.ee_pose.t,
        state.contacts.t,
        state.perception_trust.t,
    ]
    t_min = min(ts)
    t_max = max(ts)

    if (t_max - t_min) > env.max_snapshot_skew_s:
        return False, f"Snapshot skew too large: {t_max - t_min:.6f}s", "SNAPSHOT_SKEW"

    # staleness vs kernel evaluation time
    if (state.t_kernel - t_min) > env.max_state_staleness_s:
        return False, f"Snapshot too stale: {(state.t_kernel - t_min):.6f}s", "SNAPSHOT_STALE"

    # Future timestamps are also a problem (clock issues / transport)
    if t_max > state.t_kernel + 1e-6:
        return False, "Snapshot contains future-dated signals", "SNAPSHOT_FUTURE"

    return True, "Time OK", "OK"


def _state_bounds_ok(state: StateSnapshot, env: Envelope) -> Tuple[bool, str, str]:
    q = state.joints_q.value
    qd = state.joints_qd.value

    if len(q) != len(env.q_min) or len(q) != len(env.q_max) or len(q) != len(env.qd_abs_max):
        return False, "DOF mismatch between state and envelope", "DOF_MISMATCH"

    for i, qi in enumerate(q):
        if qi < env.q_min[i] or qi > env.q_max[i]:
            return False, f"Joint {i} out of range", "JOINT_LIMIT"

    for i, qdi in enumerate(qd):
        if abs(qdi) > env.qd_abs_max[i]:
            return False, f"Joint {i} velocity too high", "JOINT_VELOCITY"

    # EE workspace bound (only if provided)
    ee = state.ee_pose.value
    if ee is not None:
        x, y, z = ee[0], ee[1], ee[2]
        xmin, ymin, zmin = env.ee_xyz_min
        xmax, ymax, zmax = env.ee_xyz_max
        if not (xmin <= x <= xmax and ymin <= y <= ymax and zmin <= z <= zmax):
            return False, "End-effector out of workspace", "EE_WORKSPACE"

        # NEW: Check exclusion zones
        if env.exclusion_zones:
            for i, (zmin_tuple, zmax_tuple) in enumerate(env.exclusion_zones):
                zx1, zy1, zz1 = zmin_tuple
                zx2, zy2, zz2 = zmax_tuple
                # Check AABB intersection
                if (zx1 <= x <= zx2) and (zy1 <= y <= zy2) and (zz1 <= z <= zz2):
                    return False, f"End-effector inside exclusion zone {i}", "EE_IN_ZONE"

    return True, "Bounds OK", "OK"


def _action_ok(
    state: StateSnapshot,
    action: Action,
    env: Envelope,
    enabled_skills: Dict[str, bool],
) -> Tuple[bool, str, str]:

    if action.kind == ActionKind.ENABLE_SKILL:
        if not action.skill_name:
            return False, "Missing skill_name", "BAD_ACTION"
        # Deterministic allowlist: only skills known to the kernel
        if action.skill_name not in enabled_skills:
            return False, "Unknown skill", "UNKNOWN_SKILL"
        # Example: forbid enabling non-safety skills outside IDLE/RECOVERY
        if state.phase not in (Phase.IDLE, Phase.RECOVERY):
            return False, "Can only enable skills in IDLE/RECOVERY", "PHASE_RULE"
        return True, "OK", "OK"

    if action.kind == ActionKind.DISABLE_SKILL:
        if not action.skill_name:
            return False, "Missing skill_name", "BAD_ACTION"
        if action.skill_name not in enabled_skills:
            return False, "Unknown skill", "UNKNOWN_SKILL"
        return True, "OK", "OK"

    if action.kind == ActionKind.SET_GOAL:
        if action.goal is None:
            return False, "Missing goal", "BAD_ACTION"
        # Keep goal schema shallow & checkable
        # Example: {"type":"reach","target_xyz":[x,y,z]}
        gtype = action.goal.get("type")
        if gtype not in ("reach", "move_base", "grasp", "lift"):
            return False, "Unsupported goal type", "BAD_GOAL"
        return True, "OK", "OK"

    if action.kind == ActionKind.SET_PHASE:
        if action.next_phase is None:
            return False, "Missing next_phase", "BAD_ACTION"
        edge = (state.phase, action.next_phase)
        if edge not in env.allowed_phase_edges:
            return False, f"Illegal phase transition {state.phase}->{action.next_phase}", "PHASE_EDGE"
        return True, "OK", "OK"

    if action.kind == ActionKind.APPLY_ENVELOPE:
        # Gate validates "can switch envelope" in principle.
        # The controller/kernel must actually load the named envelope from a trusted store.
        if not action.envelope_name:
            return False, "Missing envelope_name", "BAD_ACTION"
        # Only allow envelope change in IDLE/RECOVERY to avoid mid-motion policy swaps
        if state.phase not in (Phase.IDLE, Phase.RECOVERY):
            return False, "Can only apply envelope in IDLE/RECOVERY", "PHASE_RULE"
        return True, "OK", "OK"

    return False, "Unknown action kind", "BAD_ACTION_KIND"
