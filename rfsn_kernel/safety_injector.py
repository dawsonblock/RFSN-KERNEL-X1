from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Sequence

from .controller_types import CommandKind, ControlSpace, MaskedCommand
from .monitors import SafetyEvent, SafetyLevel


@dataclass(frozen=True)
class SafetyInjectorConfig:
    """
    How to stop each space.
    """
    stop_kind: CommandKind = CommandKind.JOINT_VELOCITY
    
    # NEW: Active damping (Soft Stop)
    # If > 0.0, and stop_kind is supported, injects damping torque (-gain * qd) instead of hard zero.
    # Requires current velocity feedback.
    damping_gain: float = 0.0
    
    # If True, when STOP triggers, injector emits commands for ALL spaces.
    global_stop: bool = True


def safety_injector(
    *,
    event: SafetyEvent,
    space_dofs: Dict[ControlSpace, Tuple[int, ...]],
    cfg: SafetyInjectorConfig = SafetyInjectorConfig(),
    current_velocities: Optional[Sequence[float]] = None, # NEW: Needed for damping
) -> List[MaskedCommand]:
    """
    Convert SafetyEvent -> list of MaskedCommand(source="safety").
    Supports Active Damping if current_velocities provided and damping_gain > 0.
    """
    if event.level in (SafetyLevel.NONE, SafetyLevel.WARN):
        return []

    # Choose target spaces
    if cfg.global_stop or not event.affected_spaces:
        target_spaces = list(space_dofs.keys())
    else:
        name_to_space = {s.value: s for s in space_dofs.keys()}
        target_spaces = []
        for k in event.affected_spaces.keys():
            s = name_to_space.get(k)
            if s is not None:
                target_spaces.append(s)

    cmds: List[MaskedCommand] = []
    
    use_damping = (
        cfg.damping_gain > 0.0 
        and current_velocities is not None 
        and len(current_velocities) > 0
    )

    for space in sorted(target_spaces, key=lambda s: s.value):
        dofs = space_dofs.get(space, ())
        if not dofs:
            continue
            
        if use_damping:
            # Generate Damping Torque: tau = -gain * qd
            try:
                # Extract velocities for these DOFs
                damped_values = []
                for idx in dofs:
                    if idx < len(current_velocities):
                        damped_values.append(-cfg.damping_gain * current_velocities[idx])
                    else:
                        damped_values.append(0.0) # Fallback
                
                cmds.append(
                    MaskedCommand(
                        space=space,
                        kind=CommandKind.JOINT_TORQUE, # Force Torque mode for damping
                        dof_mask=tuple(dofs),
                        values=tuple(damped_values),
                        source="safety",
                    )
                )
            except IndexError:
                # Fallback to zero velocity on error
                zeros = tuple(0.0 for _ in dofs)
                cmds.append(MaskedCommand(space, cfg.stop_kind, tuple(dofs), zeros, "safety"))
        else:
            # Standard hard stop (usually zero velocity)
            zeros = tuple(0.0 for _ in dofs)
            cmds.append(
                MaskedCommand(
                    space=space,
                    kind=cfg.stop_kind,
                    dof_mask=tuple(dofs),
                    values=zeros,
                    source="safety",
                )
            )

    return cmds
