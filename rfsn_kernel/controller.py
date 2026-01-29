from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

from .controller_types import CapabilityLease, MaskedCommand, ControlSpace
from .arbiter import arbitrate_multi
from .safety import clamp_masked_command_to_lease, lease_active, clamp_dynamics
from .types import Envelope


@dataclass
class ControllerState:
    active_lease: Optional[CapabilityLease] = None
    active_envelope: Optional[Envelope] = None # We need the full envelope for dynamics params
    estop: bool = False
    
    # NEW: History for dynamics
    last_commands: Dict[ControlSpace, MaskedCommand] = None
    last_tick_t: float = 0.0

    def __post_init__(self):
        if self.last_commands is None:
            self.last_commands = {}


@dataclass(frozen=True)
class ControllerOutput:
    ok: bool
    reason: str
    final_by_space: Dict[ControlSpace, MaskedCommand]
    # actuator layer will merge to full DOF using dof_mask; unspecified joints => hold


def apply_estop(ctrl: ControllerState) -> None:
    ctrl.estop = True
    ctrl.active_lease = None


def clear_estop(ctrl: ControllerState) -> None:
    ctrl.estop = False


def install_lease(ctrl: ControllerState, lease: CapabilityLease, now_t: float, envelope: Optional[Envelope] = None) -> bool:
    """
    Updated signature: optional envelope to allow backward compatibility with existing tests.
    If envelope is None, dynamics checks will be skipped.
    """
    if ctrl.estop:
        return False
    if not lease_active(lease, now_t):
        return False
    if ctrl.active_lease and lease.seq <= ctrl.active_lease.seq:
        return False
        
    ctrl.active_lease = lease
    ctrl.active_envelope = envelope # Store envelope
    # Reset dynamics history on new lease? 
    # Usually better to keep continuity if possible, but safe default is reset if lease changes drastically.
    # For this ref impl, we keep history.
    return True


def step_controller_multi(
    *,
    ctrl: ControllerState,
    now_t: float,
    proposals: List[MaskedCommand],
) -> ControllerOutput:
    
    # 1. Basic Checks
    if ctrl.estop:
        return ControllerOutput(False, "E-STOP active", {})
    lease = ctrl.active_lease
    if lease is None:
        return ControllerOutput(False, "No active lease", {})
    if not lease_active(lease, now_t):
        ctrl.active_lease = None
        return ControllerOutput(False, "Lease expired", {})

    # 2. Calculate dt for dynamics
    dt = now_t - ctrl.last_tick_t
    # Clamp dt to avoid division by zero or huge jumps if system slept
    if dt <= 0.0: dt = 0.001
    if dt > 0.1: dt = 0.1 # Cap calculation at 100ms lag to prevent huge jumps

    # 3. Arbitrate
    arb = arbitrate_multi(lease=lease, proposals=proposals)
    if not arb.ok:
        return ControllerOutput(False, f"Arbiter reject: {arb.reason}", {})

    final_by_space: Dict[ControlSpace, MaskedCommand] = {}

    # 4. Clamp (Absolute + Dynamics)
    for space, cmd in arb.selected_by_space.items():
        # A. Absolute Clamp (Lease Limits)
        clamped_abs, sres = clamp_masked_command_to_lease(cmd=cmd, lease=lease)
        if not sres.ok or clamped_abs is None:
            return ControllerOutput(False, f"Abs Clamp reject {space}: {sres.reason}", {})

        # B. Dynamics Clamp (Acceleration Limits)
        # We need the envelope. If not stored, we skip dynamics (compatibility).
        if ctrl.active_envelope:
            prev_cmd = ctrl.last_commands.get(space)
            clamped_dyn, dres = clamp_dynamics(
                cmd=clamped_abs, 
                prev_cmd=prev_cmd, 
                envelope=ctrl.active_envelope, 
                dt=dt
            )
            if not dres.ok or clamped_dyn is None:
                 return ControllerOutput(False, f"Dyn Clamp reject {space}: {dres.reason}", {})
            final_by_space[space] = clamped_dyn
        else:
            final_by_space[space] = clamped_abs

    # 5. Conflict Check (Merge)
    used: Set[int] = set()
    for space, cmd in final_by_space.items():
        overlap = used.intersection(cmd.dof_mask)
        if overlap:
            return ControllerOutput(False, f"DOF conflict: {sorted(overlap)}", {})
        used.update(cmd.dof_mask)

    # 6. Update State History
    ctrl.last_commands = final_by_space
    ctrl.last_tick_t = now_t

    return ControllerOutput(True, "OK", final_by_space)
