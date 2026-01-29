from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind

def test_masked_velocity_clamp_applies_only_on_mask():
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="L2",
        issued_t=1.0,
        expiry_t=10.0,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=(0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7),
        primary_authority={"arm": "reach"},
    )
    assert install_lease(ctrl, lease, now_t=1.0)

    proposals = [
        MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, (0,2,6), (9.0, 9.0, 9.0), "reach"),
    ]

    out = step_controller_multi(ctrl=ctrl, now_t=1.1, proposals=proposals)
    assert out.ok
    cmd = out.final_by_space[ControlSpace.ARM]
    assert cmd.dof_mask == (0,2,6)
    # clamped per-index
    assert cmd.values[0] == 0.1
    assert cmd.values[1] == 0.3
    assert cmd.values[2] == 0.7
