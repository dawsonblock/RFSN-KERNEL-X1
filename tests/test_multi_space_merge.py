from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind

def test_multi_space_commands_selected_and_clamped():
    ctrl = ControllerState()
    lease = CapabilityLease(
        seq=1,
        lease_id="Lmulti",
        issued_t=1.0,
        expiry_t=10.0,
        q_min=tuple([-1.0]*7),
        q_max=tuple([ 1.0]*7),
        qd_abs_max=tuple([0.3]*7),
        primary_authority={"arm": "reach", "legs": "balance", "base": "nav"},
    )
    assert install_lease(ctrl, lease, now_t=1.0)

    # Use disjoint masks for the different spaces to allow merging
    proposals = [
        MaskedCommand(ControlSpace.ARM,  CommandKind.JOINT_VELOCITY, (0,1), (5.0, 5.0), "reach"),
        MaskedCommand(ControlSpace.LEGS, CommandKind.JOINT_VELOCITY, (2,3), (0.2, 0.2), "balance"),
        MaskedCommand(ControlSpace.BASE, CommandKind.JOINT_VELOCITY, (6,), (0.1,), "nav"),
    ]

    out = step_controller_multi(
        ctrl=ctrl,
        now_t=1.1,
        proposals=proposals,
    )

    assert out.ok
    assert set(out.final_by_space.keys()) == {ControlSpace.ARM, ControlSpace.LEGS, ControlSpace.BASE}
    assert all(abs(v) <= 0.3 + 1e-9 for v in out.final_by_space[ControlSpace.ARM].values)
