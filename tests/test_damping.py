from rfsn_kernel.safety_injector import safety_injector, SafetyInjectorConfig
from rfsn_kernel.monitors import SafetyEvent, SafetyLevel
from rfsn_kernel.controller_types import ControlSpace, CommandKind

def test_safety_damping_injection():
    # Setup: Active STOP event
    event = SafetyEvent(SafetyLevel.STOP, "collision")
    space_dofs = {ControlSpace.ARM: (0, 1)}
    
    # 1. Config with NO damping (default)
    cfg_nodamp = SafetyInjectorConfig(damping_gain=0.0)
    cmds = safety_injector(
        event=event, 
        space_dofs=space_dofs, 
        cfg=cfg_nodamp,
        current_velocities=[1.0, -1.0]
    )
    assert len(cmds) == 1
    assert cmds[0].kind == CommandKind.JOINT_VELOCITY # Default stop kind
    assert cmds[0].values == (0.0, 0.0) # Hard zero

    # 2. Config WITH damping
    cfg_damp = SafetyInjectorConfig(damping_gain=5.0)
    cmds = safety_injector(
        event=event, 
        space_dofs=space_dofs, 
        cfg=cfg_damp,
        current_velocities=[1.0, -0.5] # Robot moving at 1.0 and -0.5
    )
    assert len(cmds) == 1
    assert cmds[0].kind == CommandKind.JOINT_TORQUE # Should switch to torque
    
    # Expected: tau = -gain * vel
    # J0: -5.0 * 1.0 = -5.0
    # J1: -5.0 * -0.5 = 2.5
    assert abs(cmds[0].values[0] - (-5.0)) < 1e-6
    assert abs(cmds[0].values[1] - (2.5)) < 1e-6
