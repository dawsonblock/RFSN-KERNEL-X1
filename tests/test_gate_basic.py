from rfsn_kernel.gate import gate
from rfsn_kernel.ledger import Ledger
from rfsn_kernel.envelopes import default_envelopes
from rfsn_kernel.types import (
    Action,
    ActionKind,
    PerceptionTrust,
    Phase,
    StateSnapshot,
    Timestamped,
)

def mk_state(t=1.0, trust=PerceptionTrust.VALID, phase=Phase.IDLE, seq=0, env_fp="lab_v1|cam_v3"):
    return StateSnapshot(
        t_kernel=t,
        joints_q=Timestamped(tuple([0.0]*7), t),
        joints_qd=Timestamped(tuple([0.0]*7), t),
        ee_pose=Timestamped((0.0, 0.0, 0.5, 1.0, 0.0, 0.0, 0.0), t),
        contacts=Timestamped({"left_foot": True}, t),
        perception_trust=Timestamped(trust, t),
        phase=phase,
        seq=seq,
        env_fingerprint=env_fp,
    )

def test_enable_skill_ok():
    env = default_envelopes()["base_arm_v1"]
    ledger = Ledger(last_seq=0)
    enabled = {"reach": False, "safety": True}

    s = mk_state()
    a = Action(kind=ActionKind.ENABLE_SKILL, seq=1, skill_name="reach", action_id="a1")
    d = gate(state=s, action=a, envelope=env, ledger=ledger, enabled_skills=enabled)
    assert d.ok, d

def test_unknown_skill_reject():
    env = default_envelopes()["base_arm_v1"]
    ledger = Ledger(last_seq=0)
    enabled = {"reach": False, "safety": True}

    s = mk_state()
    a = Action(kind=ActionKind.ENABLE_SKILL, seq=1, skill_name="teleport", action_id="a2")
    d = gate(state=s, action=a, envelope=env, ledger=ledger, enabled_skills=enabled)
    assert not d.ok
