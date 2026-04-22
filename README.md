# Humanoid WBC Thesis

> **Development and Simulation of a Whole-Body Control Framework for a Humanoid Robot**
> Master's Thesis — David — 2026

[![Status](https://img.shields.io/badge/status-Literature%20Review-yellow)](https://github.com/David543M/humanoid-wbc-thesis)
[![Simulator](https://img.shields.io/badge/simulator-MuJoCo%20%7C%20Isaac%20Sim-blue)](https://github.com/David543M/humanoid-wbc-thesis)
[![Framework](https://img.shields.io/badge/framework-QP--based%20WBC-green)](https://github.com/David543M/humanoid-wbc-thesis)
[![GitHub](https://img.shields.io/badge/GitHub-David543M%2Fhumanoid--wbc--thesis-black?logo=github)](https://github.com/David543M/humanoid-wbc-thesis)

---

## Research Question

> *How can a whole-body control framework be designed and validated in simulation to enable a humanoid robot to perform stable, multi-contact locomotion and manipulation tasks under dynamic constraints?*

---

## Overview

This repository organizes the full research workflow of the Master's thesis, from problem definition to publication-grade manuscript. The work proposes a **hierarchical WBC architecture**: a centroidal MPC planning layer coupled with a QP-based Quadratic Program execution layer, validated across five simulation scenarios (S1–S5) on a humanoid URDF model.

The research pipeline combines a multi-LLM workflow (Perplexity, Gemini, Claude) with adversarial and council analysis, targeted gap-filling, simulation development, and structured academic writing aimed at publication-level quality.

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
| Language | Python / C++ |

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

## Repository Structure

```
humanoid-wbc-thesis/
├── 00_problem_definition/     # Research question, scope boundaries, hypotheses
├── 01_prompts/                # Canonical LLM prompts (versioned for reproducibility)
├── 02_raw_research/           # Raw LLM outputs, PDFs — read-only once archived
├── 03_processed/              # Summaries, extracted claims, taxonomy, knowledge graph
├── 04_council_analysis/       # Contradictions, consensus, gaps, confidence scores
├── 05_targeted_research/      # Second-pass gap-filling research
├── 06_writing/                # Outline → drafts → final chapters
├── 07_review/                 # Adversarial review, supervisor feedback, revisions
├── 08_references/             # BibTeX, validated sources, Zotero exports
├── 09_appendices/             # Frozen code, figures, datasets for manuscript
├── 10_simulation/             # URDF models, simulator configs, controller source
├── 11_experiments/            # Experiment logs, plots, tabulated results
├── 12_meetings/               # Supervisor meeting notes and action items
├── 13_logbook/                # Daily research journal
└── master_memory.md           # Single source of truth — read before any work session
```

---

## Research Workflow

1. **Define** (`00_`) — research question, scope, hypotheses
2. **Prompt** (`01_`) — author and version LLM prompts
3. **Collect** (`02_`) — run LLMs, archive raw outputs and primary PDFs
4. **Process** (`03_`) — summarize, extract atomic claims, build taxonomy
5. **Analyze** (`04_`) — council/adversarial cross-examination; flag contradictions and gaps
6. **Refine** (`05_`) — targeted research to close gaps
7. **Simulate** (`10_`, `11_`) — build/modify URDF, run controllers, record experiments
8. **Write** (`06_`) — outline then chapter drafts
9. **Review** (`07_`) — adversarial self-review + supervisor loop
10. **Finalize** (`08_`, `09_`) — consolidate references and appendices

---

## Git Workflow

### Branching strategy

```
main          ← stable, always reflects current thesis state
  └── chapter/N-title     ← one branch per chapter draft
  └── sim/feature-name    ← simulation development branches
  └── fix/description     ← corrections and revisions
```

### Commit conventions

```
[chapter-N] Short description of change
[sim] Short description of simulation update
[lit] Literature / research update
[mem] master_memory.md update
[fix] Correction or revision
```

### Push to GitHub

```powershell
# Set token (never hardcode in files)
$env:GITHUB_TOKEN = "your_token_here"

# Stage, commit, push
git add -A
git commit -m "[mem] Update master memory — session YYYY-MM-DD"
git push origin main
```

---

## Conventions

- **Filenames** — `snake_case`, date-prefixed where chronological (`YYYY-MM-DD_topic.md`)
- **Atomic claims** — each entry in `extracted_claims.json` must carry: `id`, `statement`, `sources[]`, `confidence`, `contested (bool)`
- **Sources** — never cite a claim without a peer-reviewed or verifiable anchor; tag uncertain claims explicitly
- **Versioning** — draft files use `_vN` suffix; never overwrite past versions during the writing phase
- **Token security** — never hardcode credentials in versioned files; use `$env:GITHUB_TOKEN`

---

## Key References

- Sentis & Khatib (2005) — Synthesis of whole-body behaviors, *IJRR*
- Wensing & Orin (2013) — Dynamic humanoid behaviors through task-space control, *ICRA*
- Koolen et al. (2016) — Momentum-based control framework (Atlas), *IJHR*
- Carpentier et al. (2019) — Pinocchio C++ library, *IROS*
- Mastalli et al. (2020) — Crocoddyl: efficient framework for MPC, *ICRA*

---

*Last updated: 2026-04-22 — [github.com/David543M/humanoid-wbc-thesis](https://github.com/David543M/humanoid-wbc-thesis)*
