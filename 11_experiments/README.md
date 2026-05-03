# 11_experiments — Experimental Results
*Last updated: 2026-05-03*

Iterative experimental work. Each run gets a dated subfolder: `YYYY-MM-DD_scenario-name/`.

## Evaluation Scenarios (S1–S5)

| ID | Name | Description | Key metrics | Status |
|----|------|-------------|-------------|--------|
| S1 | Static balance | Robot stands stationary; disturbance applied at t=5s | CoM error, contact force variance | 🔲 |
| S2 | Bipedal walking | Straight-line walk 3 m on flat ground | CoM tracking RMSE, step timing | 🔲 |
| S3 | Push recovery | External impulse during walking (varied magnitude/direction) | Recovery time, fall rate | 🔲 |
| S4 | Loco-manipulation | Walking while tracking end-effector target (object carry) | CoM error + EE tracking error, success rate | 🔲 |
| S5 | Perturbed loco-manip | S4 + external impulse during task | Combined metrics, constraint violation rate | 🔲 |

## Baselines

| Baseline | Description | BibKey |
|----------|-------------|--------|
| B1 | Task-priority null-space WBC (no inequality constraints) | Sentis2005 |
| B2 | Unconstrained joint-torque controller | — |
| B3 | RL locomotion policy | Radosavovic2024 |

## Standardised Metrics

| Metric | Definition | Target (anchored in benchmark_metrics.md) |
|--------|------------|------------------------------------------|
| CoM tracking RMSE | RMS of CoM position error over trial | < 3 cm steady-state, < 5 cm under perturbation |
| Contact force variance | Variance of normal contact forces at feet | Lower = better; threshold TBD |
| Task success rate | % of trials without fall or constraint violation | > X% (to anchor from literature — PQ6) |
| QP solve time | Mean and max QP solve time per control cycle | < 1 ms mean |

## Folder structure per experiment
```
YYYY-MM-DD_scenario-name/
├── config.yaml      # exact config used (QP weights, MPC horizon, task gains)
├── seed.txt         # RNG seeds for reproducibility
├── metrics.csv      # per-timestep metrics
├── summary.md       # what was tested, outcome, observations
└── plots/           # generated figures
```
