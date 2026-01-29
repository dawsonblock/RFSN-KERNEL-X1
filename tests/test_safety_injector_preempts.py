from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind
from rfsn_kernel.monitors import SafetyEvent, SafetyLevel
from rfsn_kernel.safety_injector import safety_injector

def test_safety_injector_preempts_in_one_tick():
    # Lease allows arm reach + base nav
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L",
        issued_t=1.0,
        expiry_t=10.0,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=tuple([1.0]*7),
        primary_authority={"arm": "reach", "base": "nav"},
    )
    assert install_lease(ctrl, lease, now_t=1.0)

    # DOF partition
    space_dofs = {
        ControlSpace.ARM: (0, 1, 2),
        ControlSpace.BASE: (6,),
    }

    # Primary skill proposals want motion
    primary = [
        MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, (0,1,2), (0.5,0.5,0.5), "reach"),
        MaskedCommand(ControlSpace.BASE, CommandKind.JOINT_VELOCITY, (6,), (0.4,), "nav"),
    ]

    # Monitor detects hazard and injects STOP
    event = SafetyEvent(level=SafetyLevel.STOP, reason="collision_margin", affected_spaces={"arm": "too_close"})
    injected = safety_injector(event=event, space_dofs=space_dofs)

    # Same tick: include injected first (ordering doesn't matter; arbiter selects safety anyway)
    proposals = injected + primary

    out = step_controller_multi(ctrl=ctrl, now_t=1.1, proposals=proposals)
    assert out.ok

    # Safety should override ARM at least (global_stop default True -> also overrides BASE)
    assert out.final_by_space[ControlSpace.ARM].source == "safety"
    assert all(v == 0.0 for v in out.final_by_space[ControlSpace.ARM].values)
    assert out.final_by_space[ControlSpace.BASE].source == "safety"
    assert out.final_by_space[ControlSpace.BASE].values == (0.0,)
