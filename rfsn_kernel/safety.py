from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from .controller_types import CapabilityLease, CommandKind, MaskedCommand
from .types import Envelope


@dataclass(frozen=True)
class SafetyResult:
    ok: bool
    reason: str
    code: str


def lease_active(lease: CapabilityLease, now_t: float) -> bool:
    return lease.issued_t <= now_t <= lease.expiry_t


def clamp_masked_command_to_lease(
    *,
    cmd: MaskedCommand,
    lease: CapabilityLease,
) -> tuple[Optional[MaskedCommand], SafetyResult]:
    n = len(lease.q_min)
    if len(lease.q_max) != n or len(lease.qd_abs_max) != n:
        return None, SafetyResult(False, "Lease vectors inconsistent", "LEASE_SHAPE")

    if any(i >= n for i in cmd.dof_mask):
        return None, SafetyResult(False, "DOF index out of range", "DOF_OOB")

    if cmd.kind == CommandKind.JOINT_POSITION:
        out_vals = []
        for k, i in enumerate(cmd.dof_mask):
            v = cmd.values[k]
            v = min(max(v, lease.q_min[i]), lease.q_max[i])
            out_vals.append(v)
        return MaskedCommand(cmd.space, cmd.kind, cmd.dof_mask, tuple(out_vals), cmd.source), SafetyResult(True, "OK", "OK")

    if cmd.kind == CommandKind.JOINT_VELOCITY:
        out_vals = []
        for k, i in enumerate(cmd.dof_mask):
            lim = lease.qd_abs_max[i]
            v = cmd.values[k]
            v = min(max(v, -lim), lim)
            out_vals.append(v)
        return MaskedCommand(cmd.space, cmd.kind, cmd.dof_mask, tuple(out_vals), cmd.source), SafetyResult(True, "OK", "OK")

    if cmd.kind == CommandKind.JOINT_TORQUE:
        if lease.tau_abs_max is None:
            return None, SafetyResult(False, "Torque not permitted by lease", "TORQUE_NOT_ALLOWED")
        if len(lease.tau_abs_max) != n:
            return None, SafetyResult(False, "tau_abs_max shape mismatch", "LEASE_SHAPE")
        out_vals = []
        for k, i in enumerate(cmd.dof_mask):
            lim = lease.tau_abs_max[i]
            v = cmd.values[k]
            v = min(max(v, -lim), lim)
            out_vals.append(v)
        return MaskedCommand(cmd.space, cmd.kind, cmd.dof_mask, tuple(out_vals), cmd.source), SafetyResult(True, "OK", "OK")

    return None, SafetyResult(False, "Unknown command kind", "BAD_CMD_KIND")


def clamp_dynamics(
    *,
    cmd: MaskedCommand,
    prev_cmd: Optional[MaskedCommand],
    envelope: Envelope,  # Note: logic requires Envelope access now, not just Lease
    dt: float,
) -> tuple[Optional[MaskedCommand], SafetyResult]:
    """
    Limits the rate of change (acceleration) of the command.
    Decidable O(N).
    """
    if envelope.q_acc_abs_max is None:
        return cmd, SafetyResult(True, "No dynamics limits", "OK")

    if dt <= 0.0001:
        # dt too small to calculate safe derivative, fail safe or skip?
        # Safety choice: skip dynamics check if clock is weird, but warn.
        # Strict choice: reject. Let's return strict.
        return None, SafetyResult(False, "Invalid time delta", "BAD_DT")

    if prev_cmd is None:
        # First tick: we cannot check acceleration from "nowhere".
        # Assuming starting from valid state is handled by absolute clamps.
        return cmd, SafetyResult(True, "First tick", "OK")

    # Strict: Kinds must match to compare derivatives
    if cmd.kind != prev_cmd.kind:
        # If switching modes (Pos -> Vel), we technically can't limit accel simply.
        # Resetting dynamics state is safer.
        return cmd, SafetyResult(True, "Mode switch reset", "OK")

    # We only implement Velocity Acceleration limits here (most common).
    if cmd.kind == CommandKind.JOINT_VELOCITY:
        new_values = list(cmd.values)
        
        # We must align indices. MaskedCommands might have different masks!
        # This is the tricky part of masked logic.
        # Strategy: Create a map of prev_values for O(1) lookup.
        prev_map = {idx: val for idx, val in zip(prev_cmd.dof_mask, prev_cmd.values)}

        for k, joint_idx in enumerate(cmd.dof_mask):
            prev_val = prev_map.get(joint_idx)
            
            if prev_val is not None:
                # Calculate max step allowed
                acc_limit = envelope.q_acc_abs_max[joint_idx]
                max_step = acc_limit * dt
                
                curr_val = cmd.values[k]
                
                # Clamp: prev - step <= curr <= prev + step
                clamped_val = max(prev_val - max_step, min(curr_val, prev_val + max_step))
                new_values[k] = clamped_val
        
        return MaskedCommand(cmd.space, cmd.kind, cmd.dof_mask, tuple(new_values), cmd.source), SafetyResult(True, "Dynamics clamped", "OK")

    # Pass-through for Position/Torque in this reference impl
    return cmd, SafetyResult(True, "Dynamics skipped", "OK")
