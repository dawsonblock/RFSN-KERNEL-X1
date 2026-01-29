# RFSN Kernel (Reference Functional Safety Node)

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-alpha-orange.svg)](https://github.com/dawsonblock/RFSN-KERNEL-X1)
[![Safety](https://img.shields.io/badge/safety-decidable-red.svg)]()

**A deterministic, decidable safety kernel for embodied AI and robotics.**

RFSN acts as a hypervisor for robot control. It isolates the hardware from the "untrusted" AI planner, ensuring that no matter how much the AI hallucinates, lags, or crashes, the robot remains within safe physical and temporal bounds.

---

## 🏗 Architecture

The kernel splits control into two distinct loops: **Permission (Gate)** and **Enforcement (Controller)**.

```mermaid
graph TD
    subgraph "Untrusted User Space (AI)"
        Planner[AI Planner / Policy]
        Perception[Vision / State Est]
    end

    subgraph "RFSN Kernel (Trusted)"
        Gate[Gate (Slow Loop)]
        Controller[Controller (Fast Loop)]
        Monitor[Safety Monitors]
    end

    Hardware[Robot Hardware]

    Perception -->|Snapshot| Gate
    Planner -->|Request Action| Gate
    Gate -->|Issue Lease| Controller
    
    Monitor -->|Safety Event| Controller
    Controller -->|Clamped Command| Hardware
```

### 1. The Gate (Decide Slowly)
*   **Role**: Issues **Capability Leases**.
*   **Checks**: Snapshot time consistency (skew/staleness), Envelope geometry (exclusion zones), Perception trust levels.
*   **Guarantee**: "It is safe to *attempt* this action given the current state."

### 2. The Controller (Enforce Quickly)
*   **Role**: Executes the Lease at high frequency (500Hz+).
*   **Checks**: Dynamics (acceleration limits), Hard Stops, Active Damping.
*   **Preemption**: If a Monitor triggers (e.g., Lidar intrusion), the Controller injects safety commands immediately, bypassing the Planner.

---

## 🚀 Key Features

*   **Time-Consistent Snapshots**: Rejects inputs if sensors are desynchronized (skew > 10ms) or stale.
*   **Capability Leases**: Permissions expire. If the planner crashes, the robot stops automatically.
*   **Envelope Safety**:
    *   **Spatial**: End-effector workspace bounds + Internal Exclusion Zones.
    *   **Dynamics**: Velocity and Acceleration (Jerk) clamping.
*   **Deterministic Arbitration**: 
    *   Multi-space control (Arm + Legs + Base).
    *   Masked commands (no joint conflict guarantees).
*   **Active Damping**: Soft-stop generation (`tau = -gain * velocity`) to protect gears during emergency stops.

---

## 🛠 Usage

### 1. Define an Envelope
Envelopes define the "Physics Sandbox" for a specific environment.

```python
from rfsn_kernel.types import Envelope, Phase

envelope = Envelope(
    name="lab_safe_mode",
    env_scope_prefix="lab_v1",
    max_snapshot_skew_s=0.010,
    q_min=(-2.9, -1.8, -2.9, ...),
    q_max=(2.9, 1.8, 2.9, ...),
    qd_abs_max=(2.0, 2.0, ...),
    q_acc_abs_max=(1.0, 1.0, ...), # Dynamics limiting
    exclusion_zones=[((-0.1, -0.1, 0), (0.1, 0.1, 0.5))], # Forbidden pillar
    # ...
)
```

### 2. Request a Lease (Gate)
The planner sends a `StateSnapshot` and an `Action` to the gate.

```python
from rfsn_kernel.gate import gate
from rfsn_kernel.types import Action, ActionKind

# ... acquire snapshot ...

decision = gate(
    state=snapshot,
    action=Action(kind=ActionKind.ENABLE_SKILL, skill_name="reach", seq=1),
    envelope=envelope,
    ledger=ledger,
    enabled_skills={"reach": True}
)

if decision.ok:
    # Pass lease to realtime controller
    controller.install_lease(decision.lease)
```

### 3. Run the Controller Loop
This runs in your high-priority thread (e.g., `RT_PREEMPT`, `timeloop`).

```python
from rfsn_kernel.controller import step_controller_multi
from rfsn_kernel.actuators import build_actuator_targets_v2

# 1. Gather proposals from AI skills (Reach, Walk)
proposals = [reach_skill.get_command(), balance_skill.get_command()]

# 2. Step the kernel (handles Safety Injection, Arbitration, Clamping)
output = step_controller_multi(
    ctrl=controller_state,
    now_t=time.monotonic(),
    proposals=proposals
)

# 3. Build hardware targets (Handles Hold Policy & Mixed Modes)
if output.ok:
    hw_targets = build_actuator_targets_v2(
        final_by_space=output.final_by_space,
        now_q=current_joints,
        dof_count=7,
        # ...
    )
    robot.send(hw_targets)
```

---

## 🧪 Testing

The kernel is designed to be fully testable without hardware.

```bash
# Run the full test suite
PYTHONPATH=. pytest

# Run the simulation harness
python -c "from rfsn_kernel.sim_harness import run_sim; print(run_sim()[0][:500])"
```

## 📜 License

MIT License. See [LICENSE](LICENSE) for details.
