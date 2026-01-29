from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind

def test_lease_expiry_blocks_motion():
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L1",
        issued_t=1.0,
        expiry_t=1.5,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=tuple([1.0]*7),
        primary_authority={"arm": "reach"},
    )
    assert install_lease(ctrl, lease, now_t=1.1)

    # Before expiry: OK
    out1 = step_controller_multi(
        ctrl=ctrl,
        now_t=1.2,
        proposals=[MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, tuple(range(7)), tuple([0.5]*7), "reach")],
    )
    assert out1.ok and out1.final_by_space

    # After expiry: blocked
    out2 = step_controller_multi(
        ctrl=ctrl,
        now_t=1.6,
        proposals=[MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, tuple(range(7)), tuple([0.5]*7), "reach")],
    )
    assert not out2.ok
    assert not out2.final_by_space
