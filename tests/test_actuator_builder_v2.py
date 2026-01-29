from rfsn_kernel.actuators import build_actuator_targets_v2
from rfsn_kernel.controller_types import MaskedCommand, ControlSpace, CommandKind
from rfsn_kernel.hold_policy import default_hold_policy


def test_space_hold_policy_allocates_vectors():
    dof = 7
    now_q = [0.5] * dof
    hold = default_hold_policy()

    # Define which joints belong to which space (example partition)
    space_dofs = {
        ControlSpace.ARM: (0, 1, 2, 3),
        ControlSpace.BASE: (6,),
        ControlSpace.LEGS: (4, 5),
    }

    # Only a BASE velocity command is provided.
    # Hold policy says ARM prefers POSITION holds -> builder should allocate q_des.
    final_by_space = {
        ControlSpace.BASE: MaskedCommand(
            space=ControlSpace.BASE,
            kind=CommandKind.JOINT_VELOCITY,
            dof_mask=(6,),
            values=(0.3,),
            source="nav",
        )
    }

    res = build_actuator_targets_v2(
        final_by_space=final_by_space,
        now_q=now_q,
        dof_count=dof,
        space_dofs=space_dofs,
        hold_policy=hold,
    )
    assert res.ok
    t = res.targets
    assert t is not None
    assert t.qd_des is not None  # because BASE velocity command exists
    assert t.q_des is not None   # because ARM prefers position hold, so builder allocates q_des

    # q_des should be now_q everywhere (no position commands)
    assert all(abs(x - 0.5) < 1e-12 for x in t.q_des)
    # qd_des holds 0 everywhere except joint 6
    assert t.qd_des[6] == 0.3
    assert all((i == 6) or (abs(v) < 1e-12) for i, v in enumerate(t.qd_des))


def test_safety_torque_stop_can_mix_with_velocity():
    dof = 7
    now_q = [0.0] * dof
    hold = default_hold_policy()
    space_dofs = {
        ControlSpace.ARM: (0, 1, 2, 3),
        ControlSpace.BASE: (6,),
        ControlSpace.LEGS: (4, 5),
    }

    final_by_space = {
        ControlSpace.BASE: MaskedCommand(ControlSpace.BASE, CommandKind.JOINT_VELOCITY, (6,), (0.2,), "nav"),
        # Safety torque stop on ARM joints (example: damping torque)
        ControlSpace.ARM: MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_TORQUE, (0, 1), (0.0, 0.0), "safety"),
    }

    res = build_actuator_targets_v2(
        final_by_space=final_by_space,
        now_q=now_q,
        dof_count=dof,
        space_dofs=space_dofs,
        hold_policy=hold,
        allow_safety_torque_stop=True,
    )
    assert res.ok
    t = res.targets
    assert t is not None
    assert t.qd_des is not None
    assert t.tau_des is not None
    assert t.qd_des[6] == 0.2
    assert t.tau_des[0] == 0.0 and t.tau_des[1] == 0.0


def test_rejects_torque_mixed_if_not_safety():
    dof = 7
    now_q = [0.0] * dof
    hold = default_hold_policy()
    space_dofs = {ControlSpace.ARM: (0, 1, 2, 3), ControlSpace.BASE: (6,)}

    final_by_space = {
        ControlSpace.BASE: MaskedCommand(ControlSpace.BASE, CommandKind.JOINT_VELOCITY, (6,), (0.2,), "nav"),
        # Non-safety torque mixed -> reject
        ControlSpace.ARM: MaskedCommand(ControlSpace.ARM, CommandKind.JOINT_TORQUE, (0,), (0.1,), "reach"),
    }

    res = build_actuator_targets_v2(
        final_by_space=final_by_space,
        now_q=now_q,
        dof_count=dof,
        space_dofs=space_dofs,
        hold_policy=hold,
        allow_safety_torque_stop=True,
    )
    assert not res.ok
    assert "not allowed" in res.reason.lower() or "not safety" in res.reason.lower()
