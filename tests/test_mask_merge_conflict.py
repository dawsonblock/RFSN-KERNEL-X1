from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind

def test_conflicting_dof_masks_rejected():
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L",
        issued_t=1.0,
        expiry_t=10.0,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=tuple([0.5]*7),
        primary_authority={"arm": "reach", "legs": "balance"},
    )
    assert install_lease(ctrl, lease, now_t=1.0)

    proposals = [
        # ARM commands joints 0,1,2
        MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, (0,1,2), (0.1,0.1,0.1), "reach"),
        # LEGS incorrectly also commands joint 2 -> conflict
        MaskedCommand(ControlSpace.LEGS, CommandKind.JOINT_VELOCITY, (2,3), (0.2,0.2), "balance"),
    ]

    out = step_controller_multi(ctrl=ctrl, now_t=1.1, proposals=proposals)
    assert not out.ok
    assert "DOF conflict" in out.reason
