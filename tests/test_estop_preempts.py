from rfsn_kernel.controller import ControllerState, apply_estop, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind

def test_estop_preempts_everything():
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L1",
        issued_t=1.0,
        expiry_t=10.0,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=tuple([1.0]*7),
        primary_authority={"arm": "reach"},
    )
    assert install_lease(ctrl, lease, now_t=1.0)

    apply_estop(ctrl)

    out = step_controller_multi(
        ctrl=ctrl,
        now_t=1.1,
        proposals=[MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, tuple(range(7)), tuple([0.5]*7), "reach")],
    )
    assert not out.ok
    assert not out.final_by_space
