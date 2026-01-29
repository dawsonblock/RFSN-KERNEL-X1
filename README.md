# RFSN Kernel (Reference Functional Safety Node)

<div align="center">

[![Python](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![Status](https://img.shields.io/badge/status-alpha-orange.svg)](https://github.com/dawsonblock/RFSN-KERNEL-X1)
[![Safety](https://img.shields.io/badge/safety-decidable-red.svg)]()

**The Safety Hypervisor for Embodied AI**

</div>

---

**RFSN Kernel** is a deterministic, decidable software interlock designed to sit between a high-level AI planner (LLM, VLA, RL Policy) and low-level robot hardware. 

It solves the **"Smart Planner, Safe Robot"** paradox: How do you allow a probabilistic, non-deterministic AI to control dangerous hardware without risking physical damage?

**The Solution:** Treat the AI as an "untrusted user" and the Physics as the "root authority." The kernel enforces strict spatio-temporal contracts that the AI cannot bypass.

---

## 🏗 Architecture: The Two-Loop Model

The kernel splits robotic control into two strictly isolated loops. This decoupling ensures that a slow or crashing planner never freezes the robot's safety reflexes.

```mermaid
graph TD
    subgraph "Untrusted User Space (Probabilistic)"
        Planner[AI Planner / LLM / Policy]
        Perception[Vision / State Est]
    end

    subgraph "RFSN Kernel (Deterministic)"
        direction TB
        Gate[<b>The Gate</b><br/><i>The Permission Layer</i><br/>~10-50Hz]
        Controller[<b>The Controller</b><br/><i>The Enforcement Layer</i><br/>~500Hz+]
        Monitor[<b>Safety Monitors</b><br/><i>The Reflex Layer</i><br/>Interrupt Driven]
    end

    Hardware[Robot Hardware]

    Perception -->|State Snapshot| Gate
    Planner -->|Request Action| Gate
    Gate -->|Issue Capability Lease| Controller
    
    Monitor -->|Safety Event (STOP)| Controller
    Controller -->|Clamped Command| Hardware
```

### 1. The Gate (Permission Layer)
*   **Philosophy**: "Decide Slowly."
*   **Input**: A `StateSnapshot` (sensors) and a proposed `Action` (plan).
*   **Logic**: Pure function O(1)/O(N). No simulation. No optimization.
*   **Output**: A **Capability Lease** (a time-bounded permission slip) or a Rejection.
*   **Safety Checks**: Snapshot time skew, Staleness, Envelope Geometry, Perception Trust.

### 2. The Controller (Enforcement Layer)
*   **Philosophy**: "Enforce Quickly."
*   **Input**: The active `CapabilityLease` and high-frequency `ControlCommand` proposals.
*   **Logic**: Simple clamping, geometry checks, and signal mixing.
*   **Output**: Actuator targets (Position/Velocity/Torque).
*   **Safety Checks**: Lease expiry, Acceleration limits, Active damping, Multi-space arbitration.

---

## 🛡 Failure Mode Protection Matrix

The kernel is specifically architected to catch the most common failures in Embodied AI systems.

| Failure Source | Symptom | Kernel Defense Mechanism |
| :--- | :--- | :--- |
| **Hallucination** | AI generates a goal inside the robot's body or through a wall. | **Exclusion Zones**: The Gate performs AABB geometry checks against forbidden zones in the `Envelope` and rejects the action. |
| **System Lag** | Vision pipeline lags by 200ms; Planner acts on old data. | **Skew & Staleness Checks**: The Gate calculates `t_max - t_min` of the snapshot. If > `max_skew`, action is rejected. |
| **Planner Crash** | The Python process controlling the robot freezes. | **Lease Expiry**: The Controller holds a lease with an `expiry_t`. If not renewed, the controller automatically creates a stop command. |
| **Dynamics Violation** | Planner commands a 0-to-Max velocity jump in 1ms. | **Dynamics Clamping**: The Controller limits `dq/dt` based on the `q_acc_abs_max` defined in the Envelope. |
| **Sensor Noise** | A single bad Lidar frame screams "COLLISION". | **Aggregator**: The `MonitorRegistry` creates a deterministic "worst-case" system state from multiple sensors. |
| **Mechanical Shock** | Emergency stop triggers at high speed, stripping gears. | **Active Damping**: Instead of commanding `Velocity=0`, the `SafetyInjector` commands `Torque = -Gain * Velocity` for a compliant stop. |
| **Resource Conflict** | "Walk" policy and "Reach" policy fight for the legs. | **Arbitration**: The Lease defines a `primary_authority` for each control space. Only the owner can command; others are ignored. |

---

## 🚀 Key Features V2

### 1. Spatial Exclusion Zones
Define geometric boundaries not just for the workspace, but for internal obstacles.
```python
envelope = Envelope(
    # ...
    # Define a forbidden zone (e.g., a table or self-collision zone)
    # List of (min_xyz, max_xyz) tuples
    exclusion_zones=[
        ((-0.2, -0.2, 0.0), (0.2, 0.2, 0.4)) 
    ]
)
```

### 2. Active Damping (Soft Stop)
Protects hardware during E-Stops by actively fighting momentum rather than applying a rigid lock.
```python
# In sim_harness or robot loop
injector_cfg = SafetyInjectorConfig(
    stop_kind=CommandKind.JOINT_VELOCITY, 
    damping_gain=5.0, # Active damping gain
    global_stop=True  # Stop everything if one monitor triggers
)
```

### 3. Multi-Space Arbitration
Control different parts of the robot (Arm, Legs, Base) with different policies simultaneously, with zero risk of "Double Commanding" a joint.
```python
# The Controller ensures "Reach" only talks to Arm, "Nav" only talks to Base
proposals = [
    MaskedCommand(ControlSpace.ARM, ..., source="reach"),
    MaskedCommand(ControlSpace.BASE, ..., source="nav")
]
```

---

## 📦 Integration Guide

### Installation
```bash
git clone https://github.com/dawsonblock/RFSN-KERNEL-X1.git
cd RFSN-KERNEL-X1
pip install .
```

### Basic Loop (Pseudo-code)

```python
from rfsn_kernel.gate import gate
from rfsn_kernel.controller import ControllerState, step_controller_multi, install_lease
from rfsn_kernel.types import Action, ActionKind, StateSnapshot

# 1. Setup
ctrl_state = ControllerState()
envelope = load_envelope("lab_safe")

# 2. Main Loop (High Frequency)
while True:
    now = time.monotonic()
    
    # --- SLOW PATH (Planner) ---
    if planner_ready():
        snapshot = capture_snapshot()
        action = planner.plan(snapshot)
        
        # GATE CHECK
        decision = gate(state=snapshot, action=action, envelope=envelope, ...)
        
        if decision.ok:
            # Install permissions into the fast loop
            install_lease(ctrl_state, decision.lease, now, envelope)

    # --- FAST PATH (Realtime) ---
    # Monitors check for immediate hazards
    safety_event = monitors.check()
    safety_cmds = safety_injector(safety_event, ...)
    
    # Skills propose motions
    skill_cmds = skills.get_commands()
    
    # KERNEL STEP: Arbitrate, Clamp, Damp
    output = step_controller_multi(
        ctrl=ctrl_state, 
        now_t=now, 
        proposals=safety_cmds + skill_cmds
    )
    
    # Send to hardware
    if output.ok:
        robot.send(build_actuator_targets(output.final_by_space))
```

---

## 🧪 Verification

This kernel is built with **Test-Driven Development (TDD)**. It includes a deterministic simulation harness that generates replayable traces.

```bash
# Run the full test suite (17+ tests)
PYTHONPATH=. pytest

# Run the deterministic simulation
python -c "from rfsn_kernel.sim_harness import run_sim; print(run_sim()[0][:500])"
```

## 📜 License

MIT License. See [LICENSE](LICENSE) for details.
