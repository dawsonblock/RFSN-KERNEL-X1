from rfsn_kernel.controller import ControllerState, install_lease, step_controller_multi
from rfsn_kernel.controller_types import CapabilityLease, MaskedCommand, ControlSpace, CommandKind
from rfsn_kernel.types import Envelope

def test_acceleration_clamp_smooths_jumps():
    # Setup: 7 DOF arm, Max Velocity 2.0, Max Accel 1.0 rad/s^2
    dof = 7
    # Mock envelope with acceleration limits
    class MockEnvelope:
        q_acc_abs_max = tuple([1.0] * dof) # 1.0 rad/s^2 limit
        q_min = tuple([-5.0]*dof)
        q_max = tuple([5.0]*dof)
        qd_abs_max = tuple([2.0]*dof) # Abs max velocity
        
    # Create tighter envelope for test
    env = MockEnvelope()

    lease = CapabilityLease(
        seq=1, lease_id="L1", issued_t=0.0, expiry_t=10.0,
        q_min=env.q_min, q_max=env.q_max, qd_abs_max=env.qd_abs_max,
        primary_authority={"arm": "reach"}
    )

    ctrl = ControllerState()
    # Install lease AND envelope
    install_lease(ctrl, lease, now_t=0.0, envelope=env)

    # Tick 1: Command 0.0 (Init history)
    step_controller_multi(
        ctrl=ctrl, now_t=0.0,
        proposals=[MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, tuple(range(7)), tuple([0.0]*7), "reach")]
    )

    # Tick 2: Command jumps to 2.0 rad/s immediately!
    # With dt=0.1s and Accel=1.0, we can only change by 0.1 rad/s.
    out = step_controller_multi(
        ctrl=ctrl, now_t=0.1,
        proposals=[MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_VELOCITY, tuple(range(7)), tuple([2.0]*7), "reach")]
    )

    assert out.ok
    cmd = out.final_by_space[ControlSpace.ARM]
    
    # Check Joint 0
    # Expected: prev(0.0) + accel(1.0)*dt(0.1) = 0.1
    # The requested 2.0 should be clamped down to 0.1
    assert abs(cmd.values[0] - 0.1) < 0.0001
    
    print("Dynamics Limit Verified: Request 2.0 -> Output 0.1")
