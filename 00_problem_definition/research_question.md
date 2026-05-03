# Research Question
*Locked: 2026-04-30 — Source: mem_core.md §1.2*

---

## Primary Question

> *"How can a Whole-Body Control (WBC) framework — combining a centroidal MPC planning layer with a QP-based execution layer — be designed, implemented, and rigorously evaluated in simulation to enable a humanoid robot to perform stable, simultaneous locomotion and manipulation tasks under dynamic and contact constraints?"*

---

## Sub-Questions

| ID | Question | Addressed in |
|----|----------|-------------|
| SQ1 | What are the key limitations of existing WBC formulations (task-priority, QP-based, MPC-based) when applied to full-body humanoid coordination involving simultaneous locomotion and manipulation? | Chapter 2 — State of the Art |
| SQ2 | How can contact dynamics be modelled and integrated in a simulation environment to faithfully replicate the physical behaviour expected of a torque-controlled humanoid? | Chapter 4 — Simulation Framework |
| SQ3 | What scenarios and metrics constitute a rigorous, reproducible, simulation-based evaluation of a WBC framework for humanoid loco-manipulation? | Chapter 3 (protocol) + Chapter 5 (results) |

---

## Rationale

Humanoid robots operating in human-centric environments require the simultaneous execution of locomotion and manipulation — a problem known as loco-manipulation. Existing WBC frameworks address either the planning layer (centroidal MPC) or the execution layer (QP-based WBC) in isolation. No consensus architectural blueprint exists for coupling these two layers for simultaneous loco-manipulation, and no standardised evaluation protocol allows cross-framework comparison. This thesis targets both gaps.
