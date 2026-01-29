from rfsn_kernel.gate import gate
from rfsn_kernel.ledger import Ledger
from rfsn_kernel.envelopes import default_envelopes
from rfsn_kernel.types import Action, ActionKind, PerceptionTrust, Phase, StateSnapshot, Timestamped

def test_snapshot_skew_reject():
    env = default_envelopes()["base_arm_v1"]
    ledger = Ledger(last_seq=0)
    enabled = {"reach": False, "safety": True}

    # camera/ee_pose stale vs joints -> skew too big
    s = StateSnapshot(
        t_kernel=1.000,
        joints_q=Timestamped(tuple([0.0]*7), 1.000),
        joints_qd=Timestamped(tuple([0.0]*7), 1.000),
        ee_pose=Timestamped((0.0,0.0,0.5,1,0,0,0), 0.970),  # 30ms skew
        contacts=Timestamped({"left_foot": True}, 1.000),
        perception_trust=Timestamped(PerceptionTrust.VALID, 1.000),
        phase=Phase.IDLE,
        seq=0,
        env_fingerprint="lab_v1|cam_v3",
    )

    a = Action(kind=ActionKind.ENABLE_SKILL, seq=1, skill_name="reach", action_id="skew1")
    d = gate(state=s, action=a, envelope=env, ledger=ledger, enabled_skills=enabled)
    assert not d.ok
    assert d.reject_code in ("SNAPSHOT_SKEW", "SNAPSHOT_STALE")
