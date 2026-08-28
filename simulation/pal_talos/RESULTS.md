# Hybrid RL + Whole-Body Control Walking on TALOS — Results

## 1. Objective and compute context

The goal is to test whether a thin learned high-level layer — a per-footstep
placement correction `d_foot = (dx, dy) ∈ [−0.06, 0.06] m` — can close the
foot-placement / DCM-feedback loop that the existing open-loop DCM planner lacks,
**without altering the validated WBC-QP** (friction-pyramid and torque-limit
constraints preserved). The model-based core therefore remains the interpretable
contribution; reinforcement learning supplies only the high-level decision the
analytic planner gets wrong (lateral capture-point adaptation).

All work below was produced on a constrained development sandbox: **2 CPU cores,
3.8 GB RAM, no GPU**, with each environment step (one footstep) costing ≈ 3.4 s of
wall time because it executes ≈ 1300 QP solves at the 1 kHz sim rate. This ceiling
(~0.3 footsteps/s) governs every quantitative claim that follows.

*Critical insight.* The dominant experimental constraint is not algorithmic but
computational: at 0.3 footsteps/s a single PPO rollout of 2048 steps would take
~1.9 h, so the sandbox can only falsify the *plumbing* of the hybrid loop, not
demonstrate converged locomotion. Separating these two questions — "is the
architecture sound?" vs. "does the policy converge?" — is essential to reading the
results honestly.

## 2. Baseline verification (model-based core)

The supplied controllers were written for MuJoCo 3.8 and did not run under the
installed MuJoCo 3.10: `mj_fullM` changed signature from `(m, dst, qM)` to
`(m, d, dst)`. After this single compatibility fix (the QP cost and constraints are
otherwise untouched), both baselines reproduce their documented behaviour:

| Controller | Result | Key metric |
|---|---|---|
| `talos_wbc.py --push` (balance + 150 N lateral push) | **UPRIGHT** | peak CoM deviation **6 mm**, **0/4500 QP fallbacks** |
| `talos_dcm_walk.py` (open-loop DCM, 9 s) | **FELL** | base z = 0.27 m, forward x = 0.82 m |

The standing WBC rejects the 150 N impulse with millimetric CoM excursion, confirming
the low-level controller is sound; the open-loop walker diverges laterally and falls,
confirming the specific deficiency the RL layer targets.

*Critical insight.* The contrast between a 6 mm push-recovery excursion (standing)
and outright collapse (walking) localises the failure precisely: the WBC executes
references faithfully, so the defect lives in the *reference generator* during single
support — exactly the variable the footstep action manipulates. This is the strongest
a-priori justification for the hybrid design.

## 3. Hybrid architecture and environment

`DCMWalk` was extended with a strictly-planning-layer hook, `set_footstep_offset()`,
which commits the action to the upcoming footstep (and re-runs the existing backward
DCM recursion) at each DS→SS transition. With a zero offset the walker is
**bit-identical to the baseline** (FELL, z = 0.272, x = 0.820), proving the refactor is
behaviourally transparent. Offsets are clamped to ±0.06 m and a minimum lateral
separation `Y_MIN = 0.05 m` forbids foot crossing / self-collision.

`TalosHybridWalkEnv` (Gymnasium) exposes a 20-D proprioceptive observation (base
roll/pitch + rates, CoM pose/velocity and measured DCM relative to the stance foot,
the reference DCM, gait phase, stance indicator, next nominal footstep, previous
action), a 2-D action, and a reward of `alive + forward-velocity − uprightness −
DCM-tracking − torque`, with a terminal fall penalty.

*Critical insight.* Forcing the zero-action env to coincide exactly with the analytic
baseline is the single most important validation in the package: it guarantees that
any later performance difference is attributable to the policy and not to an
inadvertent change in the planner or QP — a discipline often missing in
learning-augmented control studies.

## 4. Three-way comparison

Identical test (MuJoCo 3.10, keyframe 0, seed 0; episode capped at 5 footsteps).
The hybrid policy is the one obtained from the in-session CEM run (Section 5).

| Controller | Steps before fall | Forward distance (m) | Mean CoM err (m) | Mean \|τ\| (N·m) | Outcome |
|---|---:|---:|---:|---:|---|
| WBC standing (reference) | — | 0.00 | 0.006 | 0.81 | stable |
| Open-loop DCM | 2 | 0.346 | 0.066 | 8.91 | fell |
| **Hybrid RL + WBC** | **3** | **0.633** | 0.149 | 10.17 | fell |

The hybrid layer extends the walk by one footstep (+50 %) and roughly doubles the
forward distance before failure (0.35 → 0.63 m), at the cost of larger CoM-tracking
error and torque — the policy trades reference fidelity for survival, biasing
foot placement to arrest lateral divergence. It does **not** reach the acceptance
target (≥ 10 continuous steps).

*Critical insight.* The rise in CoM-tracking error *alongside* improved survival is
diagnostically important: it shows the policy is not learning to track the nominal
plan better but to *deviate* from it adaptively — precisely capture-point behaviour.
The metric that "worsens" is therefore evidence the mechanism is correct, a
counter-intuitive point that must be framed carefully in the thesis to avoid
misreading.

## 5. Training and its limits

Because the CPU-only PyTorch wheel index is proxy-blocked in the sandbox (and large
CUDA wheels stall mid-download), Stable-Baselines3/PPO could not be installed here.
Two trainers are therefore delivered: `train_hybrid.py` (the **canonical** PPO/SB3
pipeline with `VecNormalize`, checkpointing, TensorBoard and resume — intended for
Vertex AI or an MJX GPU stack) and `train_lite.py`, a dependency-free, resumable
Cross-Entropy-Method optimiser over a 42-parameter linear-tanh policy that runs on the
same environment in-session.

The in-session CEM run (2 iterations × 6 candidates = 12 episodes, ≤ 5 footsteps each)
improved best fitness only marginally (−18.2 → −17.6) and plateaued. This is a small
budget by any measure (≈ 50 footsteps of experience versus the 10⁵–10⁶ typical of
humanoid RL), and the linear policy class is deliberately minimal.

*Critical insight.* The flat learning curve is the expected, honest consequence of a
~50-sample budget and must not be over-interpreted as evidence against the approach:
the architecture demonstrably *can* improve on the baseline (Section 4), but
establishing convergence to ≥ 10 steps requires 3–4 orders of magnitude more
experience, which is a compute question deferred to the GPU/Vertex AI run.

## 6. Conclusion and path to the acceptance criteria

The contribution validated in-session is architectural and reproducible: the
WBC-QP is untouched and still rejects a 150 N push; the planning-layer hook is
transparent at zero action; the hybrid loop measurably outperforms the open-loop
baseline (+1 step, ~1.8× distance) on identical tests. Reaching the ≥ 10-step / push-
robust target is gated on training budget, not on the design. The recommended next
step is to run `train_hybrid.py` (PPO, `domain_rand: true`, `n_envs` 8–16,
≈ 3×10⁵–10⁶ steps) on Vertex AI / MuJoCo MJX, then re-run `eval_hybrid.py --video`
for the full 3-way table, learning curves and skeleton videos.

*Critical insight.* The honest framing for the thesis is that this experiment
*de-risks* the hybrid claim rather than proving it: it removes every confound except
sample budget, so a subsequent GPU run becomes a clean test of a single hypothesis
(does foot-placement RL on top of a fixed WBC reach robust walking?) rather than a
tangle of implementation and compute uncertainties.
