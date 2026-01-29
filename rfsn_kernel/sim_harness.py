from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

from .controller import ControllerState, install_lease, step_controller_multi, apply_estop
from .controller_types import CapabilityLease, CommandKind, ControlSpace, MaskedCommand
from .hold_policy import default_hold_policy
from .actuators import build_actuator_targets_v2
from .monitors import SafetyEvent, SafetyLevel
from .safety_injector import safety_injector
from .trace import TraceRecord


@dataclass
class SimConfig:
    dof_count: int = 7
    dt: float = 0.01  # 100 Hz simulated tick for harness (controller could be higher in reality)
    steps: int = 200
    allow_safety_torque_stop: bool = True


@dataclass
class SimState:
    t: float
    q: List[float]
    qd: List[float]


def _fake_skill_proposals(sim: SimState) -> List[MaskedCommand]:
    """
    Deterministic fake skills:
      - reach moves arm joints (0,1,2) with a slow sinus-like pattern (but no trig; keep deterministic)
      - nav moves base joint (6)
    """
    # Simple deterministic piecewise pattern
    phase = int(sim.t * 10) % 4
    arm_v = 0.2 if phase in (0, 1) else -0.2
    base_v = 0.1 if phase in (0, 3) else -0.1

    return [
        MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, (0, 1, 2), (arm_v, arm_v, arm_v), "reach"),
        MaskedCommand(ControlSpace.BASE, CommandKind.JOINT_VELOCITY, (6,), (base_v,), "nav"),
    ]


def _fake_monitor(sim: SimState) -> SafetyEvent:
    """
    Deterministic monitor:
      - at t in [0.80, 1.00): STOP arm (collision margin)
      - at t in [1.50, 1.55): E_STOP
      - else NONE
    """
    if 0.80 <= sim.t < 1.00:
        return SafetyEvent(level=SafetyLevel.STOP, reason="collision_margin", affected_spaces={"arm": "too_close"})
    if 1.50 <= sim.t < 1.55:
        return SafetyEvent(level=SafetyLevel.E_STOP, reason="hard_fault", affected_spaces=None)
    return SafetyEvent(level=SafetyLevel.NONE, reason="ok", affected_spaces=None)


def run_sim() -> tuple[str, List[TraceRecord]]:
    cfg = SimConfig()
    hold = default_hold_policy()

    # DOF partitions for holds + injector
    space_dofs = {
        ControlSpace.ARM: (0, 1, 2, 3),
        ControlSpace.LEGS: (4, 5),   # unused in fake skills, but included for holds
        ControlSpace.BASE: (6,),
    }

    # Controller + lease
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L_sim",
        issued_t=0.0,
        expiry_t=999.0,
        q_min=tuple([-3.0] * cfg.dof_count),
        q_max=tuple([ 3.0] * cfg.dof_count),
        qd_abs_max=tuple([1.0] * cfg.dof_count),
        primary_authority={"arm": "reach", "base": "nav", "legs": "balance"},
    )
    ok = install_lease(ctrl, lease, now_t=0.0)
    if not ok:
        raise RuntimeError("Failed to install lease")

    sim = SimState(t=0.0, q=[0.0] * cfg.dof_count, qd=[0.0] * cfg.dof_count)
    trace: List[TraceRecord] = []

    for step in range(cfg.steps):
        sim.t = step * cfg.dt

        # Monitor
        event = _fake_monitor(sim)
        trace.append(TraceRecord(sim.t, "monitor", {"level": event.level.value, "reason": event.reason, "affected": event.affected_spaces}))

        if event.level == SafetyLevel.E_STOP:
            apply_estop(ctrl)
            trace.append(TraceRecord(sim.t, "estop", {"applied": True}))
            # In estop, we still tick to show "no output"
            proposals = []
        else:
            injected = safety_injector(event=event, space_dofs=space_dofs)
            skills = _fake_skill_proposals(sim)
            proposals = injected + skills

        trace.append(TraceRecord(sim.t, "proposals", {
            "count": len(proposals),
            "items": [
                {"space": p.space.value, "kind": p.kind.value, "mask": list(p.dof_mask), "values": list(p.values), "src": p.source}
                for p in proposals
            ]
        }))

        # Controller tick
        out = step_controller_multi(ctrl=ctrl, now_t=sim.t, proposals=proposals)
        trace.append(TraceRecord(sim.t, "controller", {
            "ok": out.ok,
            "reason": out.reason,
            "final": [
                {"space": s.value, "kind": c.kind.value, "mask": list(c.dof_mask), "values": list(c.values), "src": c.source}
                for s, c in sorted(out.final_by_space.items(), key=lambda kv: kv[0].value)
            ],
        }))

        # Actuator targets
        if out.ok:
            built = build_actuator_targets_v2(
                final_by_space=out.final_by_space,
                now_q=sim.q,
                dof_count=cfg.dof_count,
                space_dofs=space_dofs,
                hold_policy=hold,
                allow_safety_torque_stop=cfg.allow_safety_torque_stop,
            )
        else:
            built = None

        trace.append(TraceRecord(sim.t, "actuators", {
            "built": (built.ok if built is not None else False),
            "reason": (built.reason if built is not None else "no_controller_output"),
            "q_des": (list(built.targets.q_des) if built and built.targets and built.targets.q_des else None),
            "qd_des": (list(built.targets.qd_des) if built and built.targets and built.targets.qd_des else None),
            "tau_des": (list(built.targets.tau_des) if built and built.targets and built.targets.tau_des else None),
        }))

        # Minimal state integration (velocity control only in this harness)
        # If qd_des exists, apply it.
        if built and built.targets and built.targets.qd_des is not None:
            sim.qd = list(built.targets.qd_des)
            for i in range(cfg.dof_count):
                sim.q[i] += sim.qd[i] * cfg.dt
        else:
            sim.qd = [0.0] * cfg.dof_count

    # Return JSONL and records
    from .trace import dumps_jsonl
    return dumps_jsonl(trace), trace
