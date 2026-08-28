# Whole-Body Control for a Humanoid Robot — evaluated pipeline

Simulation code, robot model and campaign reports for the MSc thesis
**Development and Simulation of a Whole-Body Control Framework for a Humanoid Robot**
(Cranfield University, 2026).

This repository is the reproducibility artefact cited in Appendix A of the thesis.
It contains the controller, the MuJoCo model and scenes, the aggregated results
behind every reported figure, and a screen capture of one run of each scenario.
It is a snapshot taken at submission.

## What is here

Everything lives under [`simulation/pal_talos/`](simulation/pal_talos/).
The path is kept as the thesis prints it.

| | |
|---|---|
| `talos_wbc.py` | The QP-WBC executor: full floating-base dynamics as a hard equality, 6D no-slip contact per stance foot, unilaterality, friction pyramid, actuation bounds. Solved with ProxQP at 1 kHz. |
| `talos_dcm_walk.py`, `talos_dcm_walk_timing.py` | Analytic DCM reference generator with reactive step timing (S2, S5). |
| `talos_s3_stairs.py`, `talos_s3_stairs_qs.py` | Stair climbing: the event-timed attempt, then the quasi-static weight-transfer sequencer that replaced it (S3). |
| `talos_s4_qs_carry.py` | Two-handed carry over the quasi-static sequencer (S4). |
| `talos_s5_perturb.py` | Directional impulse sweep (S5). |
| `centroidal_mpc.py`, `lipm_mpc.py` | The centroidal MPC planning layer. Implemented, evaluated in isolation, and **not** used in any reported campaign; it is reported as a negative result. |
| `s?_batch.py`, `s1_baselines.py` | Batch drivers. Each emits a report and a row set per campaign. |
| `probe_*.py`, `qp_*.py`, `s2_determinism.py` | The diagnostic probes behind the threats-to-validity section. |
| `*.xml`, `assets/` | MuJoCo model and scenes (TALOS, from `mujoco_menagerie`). |
| `s?_batch*/` | Aggregated campaign results: one report and one row set each. |
| `videos/` | One screen capture per scenario, with a still frame. |
| `RESULTS.md`, `EXECUTORS.md`, `*_note.md`, `*_diagnosis.md` | Working records for the campaigns and diagnoses. |

Raw per-trial archives are **not** included. They run to several hundred megabytes
and are reproducible from the drivers and the frozen configurations recorded in
Appendix A.

## What the campaigns returned

Under the shared verdict rule (the Wilson 95% lower bound must exceed the target,
not the point estimate):

| Scenario | Result | |
|---|---|---|
| S1 static balancing | 50/50 | **pass** |
| S2 flat-ground walking | 35/50 | fail |
| S3 stair climbing | 10/35 | fail |
| S4 loco-manipulation (2 kg) | 6/30 | fail |
| S5 perturbation robustness | directional margin | measured, not a rate |

The executor QP was 100% feasible in every scenario, including in trials that
ended in a fall. The reported fragility is in the reduced-order planning layer,
not in the whole-body optimisation. One scenario passes; the thesis reports the
other four as they came out.

## Running it

```bash
conda env create -f simulation/pal_talos/environment.yml
conda activate talos-wbc
cd simulation/pal_talos
python talos_dcm_walk_timing.py     # a single S2 run
python s2_batch.py                  # the seed-level campaign
```

`environment.yml` records the exact dependency closure that produced the reported
numbers. It matters more here than it usually would, because outcomes are decided
by sub-epsilon numerical events: per-seed labels are **not** bit-reproducible, and
the reproducible unit is the aggregate rate with its interval, not the individual
trajectory. Appendix A of the thesis explains why.

## Videos

`simulation/pal_talos/videos/` holds one capture per scenario, mirrored at
<https://youtube.com/playlist?list=PLGPFWyAgfJ08> for readers who prefer not to
clone. They are illustrative: they show what a run looks like, not how often it
succeeds, and no claim in the thesis rests on them.

## Licence

See [`simulation/pal_talos/LICENSE`](simulation/pal_talos/LICENSE). The
TALOS model is redistributed from `mujoco_menagerie` under its own terms.
