from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Set, Tuple

from .controller_types import CommandKind, ControlSpace, MaskedCommand
from .hold_policy import HoldPolicy


@dataclass(frozen=True)
class ActuatorTargets:
    q_des: Optional[Tuple[float, ...]] = None
    qd_des: Optional[Tuple[float, ...]] = None
    tau_des: Optional[Tuple[float, ...]] = None


@dataclass(frozen=True)
class BuildResult:
    ok: bool
    reason: str
    targets: Optional[ActuatorTargets]


# -----------------------------------------------------------------------------
# Compatibility + precedence policy
# -----------------------------------------------------------------------------

def _allowed_kinds_by_space() -> Dict[ControlSpace, Set[CommandKind]]:
    # Default conservative matrix
    return {
        ControlSpace.ARM: {CommandKind.JOINT_POSITION, CommandKind.JOINT_VELOCITY},
        ControlSpace.LEGS: {CommandKind.JOINT_VELOCITY},
        ControlSpace.BASE: {CommandKind.JOINT_VELOCITY},
        ControlSpace.WHOLE_BODY: {CommandKind.JOINT_VELOCITY},
    }


def _whole_body_exclusive(spaces_present: Set[ControlSpace]) -> bool:
    return (ControlSpace.WHOLE_BODY in spaces_present) and (len(spaces_present) > 1)


def _mixed_kinds_allowed(
    kinds_present: Set[CommandKind],
    *,
    allow_safety_torque_stop: bool,
) -> bool:
    """
    Mixed-kind policy:
      - Position + Velocity is allowed.
      - Torque is exclusive UNLESS allow_safety_torque_stop=True AND torque is used only for "stop" semantics.
    """
    if CommandKind.JOINT_TORQUE in kinds_present:
        if len(kinds_present) == 1:
            return True
        return allow_safety_torque_stop
    return True


# -----------------------------------------------------------------------------
# Builder v2
# -----------------------------------------------------------------------------

def build_actuator_targets_v2(
    *,
    final_by_space: Dict[ControlSpace, MaskedCommand],
    now_q: Sequence[float],
    dof_count: int,
    space_dofs: Dict[ControlSpace, Tuple[int, ...]],
    hold_policy: HoldPolicy,
    allow_safety_torque_stop: bool = True,
) -> BuildResult:
    """
    Deterministic builder with:
      - compatibility matrix enforcement
      - optional safety torque stop mixing
      - per-space hold modes via space_dofs + hold_policy

    Notes:
      - space_dofs defines which joints are owned by each space for HOLD application.
      - Conflicts (overlaps) between spaces in space_dofs are allowed ONLY if you never rely on holds across both.
        Best practice: make them disjoint.
    """
    if dof_count <= 0:
        return BuildResult(False, "Invalid dof_count", None)
    if len(now_q) != dof_count:
        return BuildResult(False, "now_q length != dof_count", None)
    if not final_by_space:
        return BuildResult(False, "No commands to build", None)

    spaces_present = set(final_by_space.keys())
    if _whole_body_exclusive(spaces_present):
        return BuildResult(False, "WHOLE_BODY is exclusive; other spaces present", None)

    allowed = _allowed_kinds_by_space()
    for space, cmd in final_by_space.items():
        if space not in allowed:
            return BuildResult(False, f"Unknown control space: {space.value}", None)
        if cmd.kind not in allowed[space] and not (allow_safety_torque_stop and cmd.source == "safety" and cmd.kind == CommandKind.JOINT_TORQUE):
            return BuildResult(False, f"Kind {cmd.kind.value} not allowed in {space.value}", None)

    kinds_present = {cmd.kind for cmd in final_by_space.values()}
    if not _mixed_kinds_allowed(kinds_present, allow_safety_torque_stop=allow_safety_torque_stop):
        return BuildResult(False, f"Disallowed mixed kinds: {sorted(k.value for k in kinds_present)}", None)

    # Allocate vectors on demand
    q_des = list(now_q) if CommandKind.JOINT_POSITION in kinds_present else None
    qd_des = [0.0] * dof_count if CommandKind.JOINT_VELOCITY in kinds_present else None
    tau_des = [0.0] * dof_count if CommandKind.JOINT_TORQUE in kinds_present else None

    # Apply per-space HOLD preferences to joints that belong to that space
    # and are not explicitly commanded by that space (or any space).
    # To do this correctly, first track commanded joints.
    commanded: Set[int] = set()
    for cmd in final_by_space.values():
        for i in cmd.dof_mask:
            if i < 0 or i >= dof_count:
                return BuildResult(False, f"DOF index out of range in command: {i}", None)
            if i in commanded:
                return BuildResult(False, f"DOF overlap detected at index {i}", None)
            commanded.add(i)

    # Now apply hold preference by space for its DOFs (only for uncommanded joints)
    for space, dofs in space_dofs.items():
        pref = hold_policy.preferred_hold_kind.get(space)
        if pref is None:
            continue
        for i in dofs:
            if i < 0 or i >= dof_count:
                return BuildResult(False, f"DOF index out of range in space_dofs: {i}", None)
            if i in commanded:
                continue

            if pref == CommandKind.JOINT_POSITION:
                if q_des is None:
                    q_des = list(now_q)
                # ensure hold position is set (already now_q), so no-op
            elif pref == CommandKind.JOINT_VELOCITY:
                if qd_des is None:
                    qd_des = [0.0] * dof_count
                # hold velocity is 0.0, no-op
            elif pref == CommandKind.JOINT_TORQUE:
                if tau_des is None:
                    tau_des = [0.0] * dof_count
                # hold torque is 0.0, no-op
            else:
                return BuildResult(False, "Unknown preferred hold kind", None)

    # Finally, write commanded joints into the appropriate vectors
    for space, cmd in sorted(final_by_space.items(), key=lambda kv: kv[0].value):
        for k, i in enumerate(cmd.dof_mask):
            v = cmd.values[k]

            if cmd.kind == CommandKind.JOINT_POSITION:
                if q_des is None:
                    q_des = list(now_q)
                q_des[i] = v
            elif cmd.kind == CommandKind.JOINT_VELOCITY:
                if qd_des is None:
                    qd_des = [0.0] * dof_count
                qd_des[i] = v
            elif cmd.kind == CommandKind.JOINT_TORQUE:
                if tau_des is None:
                    tau_des = [0.0] * dof_count
                # Safety torque stop mixing is allowed, but only from safety
                if allow_safety_torque_stop and cmd.source != "safety" and len(kinds_present) > 1:
                    return BuildResult(False, "Torque mixed but not safety-sourced", None)
                tau_des[i] = v
            else:
                return BuildResult(False, "Unknown command kind", None)

    return BuildResult(
        True,
        "OK",
        ActuatorTargets(
            q_des=tuple(q_des) if q_des is not None else None,
            qd_des=tuple(qd_des) if qd_des is not None else None,
            tau_des=tuple(tau_des) if tau_des is not None else None,
        ),
    )
