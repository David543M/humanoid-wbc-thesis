# Scope
*Locked: 2026-04-30 — Source: mem_core.md §1.5*

---

## In Scope

- Simulation only (no hardware experiments)
- Whole-Body Control (WBC) for simultaneous locomotion and manipulation (loco-manipulation)
- TALOS humanoid robot (PAL Robotics) — URDF in MuJoCo
- Two-layer control architecture: centroidal MPC (planning) + QP-based WBC (execution)
- Reproducible evaluation protocol: scenarios S1–S5 with standardised metrics
- Comparative baselines: task-priority WBC, unconstrained torque control, RL policy

## Out of Scope

- Hardware experiments (physical robot deployment)
- Manipulation-only or locomotion-only tasks
- Wheeled or non-humanoid legged robots
- Deep reinforcement learning as a primary contribution (RL used as baseline only)
- Sensor fusion and state estimation algorithms
- Long-horizon task sequencing
- Energy efficiency analysis

---

## Target Platform

| Parameter | Value |
|-----------|-------|
| Robot | TALOS (PAL Robotics) |
| DOF | 28–32 |
| Sensors (sim) | IMU, joint encoders, contact force sensors |
| Joint control | Torque-controlled (SEA or ideal) |
| Simulator | MuJoCo |
| Kinematics lib | Pinocchio |
| Terrain | Flat ground, stairs, obstacles (simulation) |

---

## Real-Time Constraints

| Loop | Frequency | Max solve time |
|------|-----------|---------------|
| Torque control (QP) | 1 kHz | < 1 ms |
| Linear centroidal MPC | 50–100 Hz | < 10 ms |
| Nonlinear centroidal MPC | 20–50 Hz | < 50 ms |

---

## Key Assumptions

1. The simulation contact model (MuJoCo soft contacts) is representative enough for proof-of-concept validation — the sim-to-real gap is acknowledged as a limitation, not denied.
2. The TALOS URDF inertial parameters are sufficiently accurate for model-based WBC.
3. Full state observation is available in simulation (no state estimation noise).
4. Baselines are evaluated under strictly identical simulation conditions to the proposed framework.
