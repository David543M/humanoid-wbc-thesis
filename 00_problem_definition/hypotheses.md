# Hypotheses
*Locked: 2026-04-30 — Source: mem_core.md §1.4*

---

| ID | Hypothesis | Expected Evidence | Falsification Criterion |
|----|-----------|-------------------|------------------------|
| **H1** | A hierarchical QP-based WBC formulation, planned by a centroidal MPC layer, can achieve stable whole-body coordination across simultaneous locomotion and manipulation tasks. | Task success rate above benchmark-anchored threshold; bounded CoM tracking error across scenarios S1–S5. | Persistent constraint violations or instability in > Y% of trials (Y to be anchored in Ch3 from benchmark literature). |
| **H2** | MuJoCo simulation is a sufficient environment to validate proof-of-concept for the proposed WBC framework before hardware deployment. | Tasks succeed consistently under varied perturbation conditions in simulation. | Significant performance degradation attributable to simulation artefacts rather than controller design. |
| **H3** | Explicit contact modelling and task-priority hierarchies outperform naive torque control in multi-contact transition scenarios. | Lower CoM deviation and lower contact force variance than unconstrained torque-control baseline. | Equivalent or worse metrics vs. baseline on the same S1–S5 scenarios. |

---

## Notes

- Thresholds marked as placeholders (X, Y, Z) are to be anchored in `03_processed/benchmark_metrics.md` before Ch5 is written — see PQ6.
- H2 does not claim zero sim-to-real gap; it claims simulation is sufficient for proof-of-concept. The gap is characterised explicitly in Ch6.
