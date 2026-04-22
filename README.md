# Humanoid WBC Thesis

> **Development and Simulation of a Whole-Body Control Framework for a Humanoid Robot**
> Master's Thesis — David — 2026

[![Status](https://img.shields.io/badge/status-Literature%20Review-yellow)](.)
[![Simulator](https://img.shields.io/badge/simulator-MuJoCo%20%7C%20Isaac%20Sim-blue)](.)
[![Framework](https://img.shields.io/badge/framework-QP--based%20WBC-green)](.)

---

## Research Question

> *How can a whole-body control framework be designed and validated in simulation to enable a humanoid robot to perform stable, multi-contact locomotion and manipulation tasks under dynamic constraints?*

**Last structure update.** 2026-04-22

---

## Overview

This repository organizes the full research workflow of the thesis, from problem definition to publication-grade manuscript. The work proposes a **hierarchical WBC architecture**: a centroidal MPC planning layer coupled with a QP-based execution layer, validated across five simulation scenarios (S1–S5) on a humanoid URDF model.

The pipeline combines a multi-LLM research workflow (Perplexity, Gemini, Claude) with adversarial/council analysis, targeted gap-filling, simulation development, and structured academic writing.

---

## Thesis Chapters

| Chapter | Title | Status |
|---------|-------|--------|
| 1 | Introduction | 🔲 Not started |
| 2 | State of the Art — WBC for Humanoids | 🔲 Not started |
| 3 | Methodology | 🔲 Not started |
| 4 | Simulation Framework | 🔲 Not started |
| 5 | Experiments and Results | 🔲 Not started |
| 6 | Discussion | 🔲 Not started |
| 7 | Conclusion and Future Work | 🔲 Not started |

---

## Technical Stack (Planned)

| Component | Tool |
|-----------|------|
| Dynamics library | [Pinocchio](https://github.com/stack-of-tasks/pinocchio) |
| Optimal control | [Crocoddyl](https://github.com/loco-3d/crocoddyl) |
| QP solver | OSQP / qpOASES |
| Simulator | MuJoCo or NVIDIA Isaac Sim (TBD) |
| Robot model | TALOS or Unitree H1 URDF (TBD) |

---

## Test Scenarios

| ID | Description | Primary Metric |
|----|-------------|----------------|
| S1 | Static balancing under external pushes | CoM error, recovery time |
| S2 | Flat-ground walking (3 m) | Task success, CoM tracking |
| S3 | Stair climbing (3 steps) | Foot clearance, contact forces |
| S4 | Loco-manipulation (walk + reach) | End-effector error + CoM tracking |
| S5 | Perturbation robustness | Recovery rate |

---

## Key References

- Sentis & Khatib (2005) — Synthesis of whole-body behaviors, *IJRR*
- Wensing & Orin (2013) — Dynamic humanoid behaviors through task-space control, *ICRA*
- Koolen et al. (2016) — Momentum-based control framework (Atlas), *IJHR*
- Carpentier et al. (2019) — Pinocchio C++ library, *IROS*
- Mastalli et al. (2020) — Crocoddyl: efficient framework for MPC, *ICRA*

---

## Purpose

## Directory layout

| Folder | Role |
|---|---|
| `00_problem_definition/` | Research question, scope boundaries, hypotheses. Frozen early, revisited at major milestones. |
| `01_prompts/` | Canonical prompts for each LLM and for council/adversarial passes. Versioned for reproducibility. |
| `02_raw_research/` | Untouched outputs from LLMs, PDFs of peer-reviewed papers, conference videos/talks. Read-only once archived. |
| `03_processed/` | Per-paper summaries, extracted atomic claims (`extracted_claims.json`), taxonomy of approaches, knowledge graph. |
| `04_council_analysis/` | Cross-source analysis: contradictions, consensus points, research gaps, per-claim confidence scores. |
| `05_targeted_research/` | Second-pass research explicitly filling gaps identified in `04_`. |
| `06_writing/` | Outline → drafts → chapters. |
| `07_review/` | Adversarial self-review, supervisor feedback, revision logs. |
| `08_references/` | BibTeX, validated sources list, Zotero/Mendeley exports. |
| `09_appendices/` | Frozen code snapshots for the manuscript, final figures, reference datasets. |
| `10_simulation/` | URDF/SDF models, scenes (Gazebo/MuJoCo/PyBullet/Drake), controller configs, live controller source. |
| `11_experiments/` | Experiment logs, plots, tabulated results. Iterative, distinct from the frozen `09_appendices/`. |
| `12_meetings/` | Supervisor meeting notes + action items. |
| `13_logbook/` | Chronological research journal — daily/weekly progress. Fuel for writing and retrospectives. |

## Workflow (high level)

1. **Define** (`00_`) — research question, scope, hypotheses.
2. **Prompt** (`01_`) — author and version LLM prompts.
3. **Collect** (`02_`) — run LLMs, archive raw outputs and primary PDFs.
4. **Process** (`03_`) — summarize, extract atomic claims, build taxonomy.
5. **Analyze** (`04_`) — council/adversarial cross-examination; flag contradictions and gaps.
6. **Refine** (`05_`) — targeted research to close gaps.
7. **Simulate** (`10_`, `11_`) — build/modify URDF, run controllers, record experiments.
8. **Write** (`06_`) — outline then chapter drafts.
9. **Review** (`07_`) — adversarial self-review + supervisor loop.
10. **Finalize** (`08_`, `09_`) — consolidate references and appendices.

## Conventions

- **Filenames.** `snake_case`, date-prefixed where chronological (`YYYY-MM-DD_topic.md`).
- **Atomic claims.** Each claim in `extracted_claims.json` must carry: `id`, `statement`, `sources[]`, `confidence`, `contested (bool)`.
- **Sources.** Never cite a claim without a peer-reviewed or otherwise verifiable anchor. Tag uncertain claims explicitly.
- **Versioning.** Draft files use `_vN` suffix; never overwrite past versions during the writing phase.
