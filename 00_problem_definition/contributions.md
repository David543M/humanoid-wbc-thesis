# Contributions
*Defined: 2026-04-30 — Source: mem_core.md + Ch1 v1.0*

---

## C1 — Documented WBC Framework for Simultaneous Loco-Manipulation

A hierarchical control architecture coupling a **centroidal MPC planning layer** with a **QP-based whole-body execution layer**, designed and implemented in simulation at reproduction level.

**Specifics:**
- Centroidal MPC operates on a reduced dynamic model of TALOS
- QP execution layer enforces rigid-body dynamics, joint torque limits, and friction-cone constraints at 1 kHz
- The **planning–execution interface** (contact schedule representation, rate decoupling) is specified explicitly — this is the novel architectural contribution
- Full implementation released as open-access repository: MuJoCo scene, TALOS URDF, controller code, scenario configs

**What this is NOT:** a claim to supersede Crocoddyl or TSID. It is an explicit coupling design documented for the specific case of simultaneous loco-manipulation.

---

## C2 — Reproducible Evaluation Protocol (S1–S5)

A standardised benchmark protocol addressing the evaluation gap in humanoid WBC literature.

| Scenario | Description |
|----------|-------------|
| S1 | Static balance |
| S2 | Bipedal walking |
| S3 | Push recovery |
| S4 | Simultaneous walking and manipulation |
| S5 | Perturbation under loco-manipulation |

**Standardised metrics:**
- CoM tracking error
- Contact force variance
- Task success rate
- QP solve time

All thresholds anchored in benchmark literature (to be finalised in `03_processed/benchmark_metrics.md`).

**Design intent:** any WBC framework should be evaluable against this protocol given access to the simulation environment.

---

## C3 — Comparative Baseline Analysis

Quantitative comparison of the proposed framework against three baselines on scenarios S1–S5:

1. Task-priority null-space WBC (no explicit inequality constraints)
2. Unconstrained joint-torque controller
3. Learned locomotion policy (RL baseline)

Provides a quantitative reference point for future work and allows evaluation of H1–H3.

---

## Novelty Positioning

| Aspect | This work | Crocoddyl | TSID | RL baselines |
|--------|-----------|-----------|------|-------------|
| Explicit MPC–QP coupling for loco-manip | ✅ | Partial | ❌ | N/A |
| Reproducible evaluation protocol | ✅ | ❌ | ❌ | Partial |
| Open simulation repository | ✅ | ✅ | Partial | Varies |
| Constraint guarantees | ✅ | ✅ | ✅ | ❌ |
| Simultaneous loco-manipulation | ✅ | Partial | ❌ | Partial |
