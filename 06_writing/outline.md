# Thesis Outline — Detailed Structure
*Last updated: 2026-05-03 | Status reflects latex_thesis/ source*

---

## Chapter 1 — Introduction ✅ v1.0
- 1.1 Motivation
- 1.2 Problem Statement (two gaps: architectural coupling + evaluation standardisation)
- 1.3 Research Question and Objectives (RQ + SQ1–SQ3)
- 1.4 Hypotheses (H1–H3)
- 1.5 Scope and Limitations (in/out table + narrative)
- 1.6 Contributions (C1–C3)
- 1.7 Thesis Structure

## Chapter 2 — State of the Art 🟠 v0.2 (post-council)
- 2.1 Foundations of Whole-Body Control
- 2.2 Task-Priority and Null-Space Methods
- 2.3 QP-Based Whole-Body Control Formulations
- 2.4 Centroidal MPC and Planning Layers
- 2.5 Learning-Based Approaches (baseline reference)
- 2.6 Simulation Environments and Benchmarks
- 2.7 Positioning: gaps → thesis contributions
- **⚠️ Pending:** bibliography verification (27 sources ⚠️/🔲), §2.6 placement

## Chapter 3 — Methodology 🔲 Not started
- 3.1 Notation and mathematical conventions
- 3.2 Floating-base dynamics (equation of motion)
- 3.3 Task hierarchy and priority enforcement
- 3.4 QP-based WBC formulation (decision variables, cost, constraints)
- 3.5 Centroidal MPC planning layer
- 3.6 Planning–execution interface (contact schedule, rate decoupling)  ← key contribution C1
- 3.7 Contact model and friction cone
- 3.8 Evaluation protocol: scenarios S1–S5 and metrics  ← key contribution C2
- 3.9 Baseline definitions (task-priority WBC, unconstrained torque, RL)

## Chapter 4 — Simulation Framework 🔲 Not started
- 4.1 MuJoCo environment setup
- 4.2 TALOS URDF and inertial parameters
- 4.3 Contact model configuration and validation
- 4.4 Software stack (Pinocchio + QP solver + MPC)
- 4.5 State observation and logging

## Chapter 5 — Experiments and Results 🔲 Not started
- 5.1 Experimental setup and reproducibility protocol
- 5.2 S1: Static balance
- 5.3 S2: Bipedal walking
- 5.4 S3: Push recovery
- 5.5 S4: Simultaneous walking and manipulation
- 5.6 S5: Perturbation under loco-manipulation
- 5.7 Comparative analysis vs baselines
- 5.8 Computation time analysis

## Chapter 6 — Discussion 🔲 Not started
- 6.1 Hypothesis evaluation (H1–H3)
- 6.2 Architectural coupling: lessons learned
- 6.3 Evaluation protocol: validity and generalisability
- 6.4 Sim-to-real gap analysis
- 6.5 Limitations

## Chapter 7 — Conclusion and Future Work 🔲 Not started
- 7.1 Summary of contributions
- 7.2 Future work (hardware transfer, long-horizon sequencing, energy efficiency)
