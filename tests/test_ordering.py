from rfsn_kernel.gate import gate
from rfsn_kernel.ledger import Ledger
from rfsn_kernel.envelopes import default_envelopes
from rfsn_kernel.types import Action, ActionKind, PerceptionTrust, Phase, StateSnapshot, Timestamped

def mk_state(seq):
    t = 1.0
    return StateSnapshot(
        t_kernel=t,
        joints_q=Timestamped(tuple([0.0]*7), t),
        joints_qd=Timestamped(tuple([0.0]*7), t),
        ee_pose=Timestamped((0.0,0.0,0.5,1,0,0,0), t),
        contacts=Timestamped({"left_foot": True}, t),
        perception_trust=Timestamped(PerceptionTrust.VALID, t),
        phase=Phase.IDLE,
        seq=seq,
        env_fingerprint="lab_v1|cam_v3",
    )

def test_replay_rejected():
    env = default_envelopes()["base_arm_v1"]
    ledger = Ledger(last_seq=0)
    enabled = {"reach": False, "safety": True}
    s = mk_state(seq=0)

    a1 = Action(kind=ActionKind.ENABLE_SKILL, seq=1, skill_name="reach", action_id="dup")
    d1 = gate(state=s, action=a1, envelope=env, ledger=ledger, enabled_skills=enabled)
    assert d1.ok
    ledger.apply(a1)

    # same action_id again -> reject
    s2 = mk_state(seq=1)
    a2 = Action(kind=ActionKind.ENABLE_SKILL, seq=2, skill_name="reach", action_id="dup")
    d2 = gate(state=s2, action=a2, envelope=env, ledger=ledger, enabled_skills=enabled)
    assert not d2.ok
    assert d2.reject_code == "ORDER_VIOLATION"
