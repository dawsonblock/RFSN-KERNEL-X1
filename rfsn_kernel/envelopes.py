from __future__ import annotations

from dataclasses import replace
from typing import Dict

from .types import Envelope, Phase


def default_envelopes() -> Dict[str, Envelope]:
    # A conservative baseline envelope for a 7-DOF arm-like system.
    # Adjust tuples to match your robot DOF.
    q_min = (-2.9, -1.8, -2.9, -3.1, -2.9, -0.1, -2.9)
    q_max = ( 2.9,  1.8,  2.9,  0.1,  2.9,  3.8,  2.9)
    qd_abs = (2.0, 2.0, 2.0, 2.5, 2.5, 3.0, 3.0)

    edges = (
        (Phase.IDLE, Phase.APPROACH),
        (Phase.APPROACH, Phase.ALIGN),
        (Phase.ALIGN, Phase.GRASP),
        (Phase.GRASP, Phase.LIFT),
        (Phase.LIFT, Phase.RETREAT),
        (Phase.RETREAT, Phase.IDLE),

        # Recovery paths
        (Phase.APPROACH, Phase.RECOVERY),
        (Phase.ALIGN, Phase.RECOVERY),
        (Phase.GRASP, Phase.RECOVERY),
        (Phase.LIFT, Phase.RECOVERY),
        (Phase.RETREAT, Phase.RECOVERY),
        (Phase.RECOVERY, Phase.IDLE),

        # Termination
        (Phase.IDLE, Phase.TERMINATED),
        (Phase.RECOVERY, Phase.TERMINATED),
    )

    base = Envelope(
        name="base_arm_v1",
        env_scope_prefix="lab_v1",
        max_snapshot_skew_s=0.010,        # 10 ms skew budget
        max_state_staleness_s=0.020,      # 20 ms stale budget

        q_min=q_min,
        q_max=q_max,
        qd_abs_max=qd_abs,

        ee_xyz_min=(-0.6, -0.6, 0.0),
        ee_xyz_max=( 0.6,  0.6, 1.2),

        allow_new_commits_when_degraded=False,
        allow_new_commits_when_untrusted=False,

        allowed_phase_edges=edges,

        primary_authority={"arm": "reach", "base": "safety"},
    )

    return {base.name: base}


def tighten(envelope: Envelope, *, qd_scale: float = 0.5) -> Envelope:
    """Example monotone tightening operation (allowed)."""
    qd_new = tuple(max(0.01, x * qd_scale) for x in envelope.qd_abs_max)
    return replace(envelope, qd_abs_max=qd_new)
