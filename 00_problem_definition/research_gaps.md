# Research Gaps
*Source: mem_lit.md + Ch1 v1.0 §1.2 + Ch2 v0.2 §2.7*

This file maps the gaps in the literature to the contributions of this thesis.

---

## Gap 1 — Architectural Coupling (Methodological Gap)

**Statement:** No consensus framework exists for coupling a centroidal MPC planning layer with a QP-based WBC execution layer for simultaneous loco-manipulation.

**Evidence:** Crocoddyl, TSID, and iCub-controllers each resolve the planning–execution interface differently, in platform-specific code not documented at reproduction level.

**Specific unresolved sub-problems:**
- How is the contact schedule passed from planner to executor?
- How are timing mismatches handled when MPC and QP loops run at different rates?
- How do execution-level constraint violations feed back to the planner?

**Addressed by:** Contribution C1

---

## Gap 2 — Evaluation Standardisation (Evaluation Gap)

**Statement:** No standardised benchmark scenarios or metrics exist for humanoid WBC involving simultaneous locomotion and manipulation.

**Evidence:** Published papers report heterogeneous tasks, perturbation protocols, and success criteria. The only commonly reported metric is often binary (robot fell / did not fall). See Sferrazza et al. (2024) for a related benchmark gap analysis.

**Specific unresolved sub-problems:**
- No public dataset of contact force distributions during loco-manipulation
- Robustness to perturbations rarely reported with reproducible protocols
- Sim-to-real degradation rarely quantified

**Addressed by:** Contribution C2

---

## Gap 3 — Simultaneous Loco-Manipulation (Application Gap)

**Statement:** Most WBC papers evaluate locomotion *or* manipulation, rarely both simultaneously.

**Evidence:** Literature survey (Ch2) shows task-priority and QP-WBC papers typically demonstrate either balance/walking *or* upper-body manipulation tasks. Unified evaluation is rare.

**Addressed by:** Contributions C1 + C3 (scenarios S4 and S5 specifically)

---

## Gap 4 — Data Gap

**Statement:** No public dataset of contact force distributions for multi-contact loco-manipulation exists for humanoid platforms.

**Status:** Partially out of scope for this thesis — noted as a future work direction in Ch7.

---

## Logical Chain (mandatory — from mem_core.md)

```
Problem
  Humanoid WBC for loco-manipulation: high-DOF, contact-rich, real-time
      ↓
Gap
  No consensus coupling of MPC planning + QP-WBC execution for loco-manipulation
  No standardised evaluation protocol for cross-framework comparison
      ↓
Proposed Solution
  Two-layer hierarchical WBC: centroidal MPC + QP execution (C1)
  Reproducible evaluation protocol S1–S5 with standardised metrics (C2)
      ↓
Validation
  Simulation (MuJoCo / TALOS) — scenarios S1–S5 vs. 3 baselines (C3)
      ↓
Contribution
  Reproducible framework + evaluation protocol + comparative analysis
```
