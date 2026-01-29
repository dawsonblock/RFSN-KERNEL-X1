from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

from .controller_types import CommandKind, ControlSpace, MaskedCommand
from .monitors import SafetyEvent, SafetyLevel


@dataclass(frozen=True)
class SafetyInjectorConfig:
    """
    How to stop each space.
    Default policy:
      - STOP: zero velocities on affected DOFs
      - E_STOP: handled outside (controller apply_estop), but injector can still output zeros
    """
    stop_kind: CommandKind = CommandKind.JOINT_VELOCITY
    # If True, when STOP triggers, injector emits commands for ALL spaces, not only affected ones.
    global_stop: bool = True


def safety_injector(
    *,
    event: SafetyEvent,
    space_dofs: Dict[ControlSpace, Tuple[int, ...]],
    cfg: SafetyInjectorConfig = SafetyInjectorConfig(),
) -> List[MaskedCommand]:
    """
    Convert SafetyEvent -> list of MaskedCommand(source="safety").

    Deterministic:
      - STOP => zero command in cfg.stop_kind over target DOFs
      - WARN/NONE => no injection
    """
    if event.level in (SafetyLevel.NONE, SafetyLevel.WARN):
        return []

    # Choose target spaces
    if cfg.global_stop or not event.affected_spaces:
        target_spaces = list(space_dofs.keys())
    else:
        # map string keys to ControlSpace
        name_to_space = {s.value: s for s in space_dofs.keys()}
        target_spaces = []
        for k in event.affected_spaces.keys():
            s = name_to_space.get(k)
            if s is not None:
                target_spaces.append(s)

    cmds: List[MaskedCommand] = []
    for space in sorted(target_spaces, key=lambda s: s.value):
        dofs = space_dofs.get(space, ())
        if not dofs:
            continue
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
