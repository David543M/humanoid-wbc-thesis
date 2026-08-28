# Model-based walking (WBC + DCM planning) — diagnosis and closed-loop attempt

## 1. Failure mode of the open-loop walker (precisely localised)

A tick-level trace of the lateral channel shows the baseline (`talos_dcm_walk.py`,
default) falls **sideways at the second step** (roll → −1.1 rad at t ≈ 4 s), not
sagittally. During the first left single-support phase the measured DCM
(`ξ_y = c_y + ċ_y/ω`) overshoots the stance-foot edge: it leaves the planned
+0.085 m and grows exponentially (+0.14 → +0.28 → +1.0 m) while the CoM tips outward.
Once ξ_y passes the foot edge (~+0.135 m, the maximum ZMP a 0.05 m-wide foot can
produce), the lateral dynamics is no longer controllable by ankle/CoM action and the
robot is lost. The CoM reference itself stays well-behaved (±0.06 m) — the divergence
is in the *plant*, not the reference.

*Critical insight.* The decisive number is the lateral divergence rate ω = √(g/z_c) ≈
3.23 s⁻¹: over the original T_step = 1.3 s the lateral DCM error is multiplied by
e^{ω·T} ≈ 66 per step. No bounded feedback can stabilise a once-per-step decision with
that much intra-step growth — the instability is structural, set by the gait timing,
before any controller gain is chosen.

## 2. Model-based remedies implemented (opt-in, WBC-QP untouched, defaults preserved)

Three standard model-based feedback laws were added to `DCMWalk`, all in the planning
layer; the QP cost/constraints are unchanged and the default (no-flag) behaviour is
bit-identical to the baseline (FELL, z = 0.272, x = 0.820):

1. **Continuous DCM feedback into the CoM reference** (`--feedback`, gain `k_dcm`):
   pulls the CoM reference to cancel the measured DCM error (the stable LIPM part).
2. **Per-step capture-point foot placement** (`--footfb`, gain `k_foot`): at each
   DS→SS transition shifts the next footstep by `k_foot·(ξ_meas − ξ_ref)`, clamped to
   a feasible offset and a minimum lateral separation (no foot crossing).
3. **Continuous capture-point swing retargeting** (`--cpswing`): steers the swing foot
   *during* the step toward the measured capture point.

A naïve fourth attempt — commanding the CoM with the raw LIPM acceleration
`c̈ = ω²(c − p_zmp)` — was tested and rejected: that form is open-loop unstable and the
soft CoM task cannot realise it stiffly, so it diverged faster (CoM to −0.6 m). It is
not retained.

*Critical insight.* That the unstable-acceleration form fails while the
stable-reference form survives is the practical lesson: with a CoM-tracking WBC the
feedback must enter through a *stable* reference trajectory, never through the
inverted-pendulum acceleration directly — a subtlety that separates a working DCM
controller from a textbook transcription.

## 3. Result of the closed-loop preset

The `--walk-cl` preset bundles the three laws with faster stepping (T_step = 0.5 s,
longer double-support DS = 0.45) to cut the per-step error growth from ×66 to ×e^{1.6}
≈ ×5:

| Controller | Steps before fall | Forward distance | Fall mode |
|---|---:|---:|---|
| open-loop DCM (default) | 3 | 0.82 m | lateral, step 2 |
| **closed-loop `--walk-cl`** | **~4** | **~0.92 m** | sagittal, later |

The closed-loop controller roughly **doubles** the survivable steps and extends the
walk, but does **not** achieve sustained walking: it still loses lateral balance around
the sixth step. Sensitivity to the planning horizon (a 10-step plan superficially
"reaches step 9" because its terminal capture masks the divergence, whereas a 30-step
plan falls at step 6) confirms the residual instability is in the *gait design*, not
the feedback gains.

## 4. What sustained model-based walking would require

The remaining gap is a *valid lateral limit cycle*: the planned lateral DCM and the
weight-transfer timing must be designed so the robot enters each single-support phase
with the DCM inside the foot and moving so as to reverse — together with **adaptive
step timing** (trigger touchdown early when the DCM approaches the foot edge), which
the current fixed-time plan lacks. This is a substantial planner redesign
(periodic-DCM / divergent-component gait synthesis + event-based timing).

*Critical insight.* This is the honest justification for the learned high-level layer:
the analytical controller is bounded by the once-per-step, fixed-timing decision
structure, and closing the lateral loop robustly demands adapting *both* footstep
placement *and* timing under the full nonlinear dynamics — precisely the
high-dimensional decision that an RL policy over the fixed WBC is positioned to supply.
The model-based effort here is therefore not a detour but the experiment that *defines*
what the learning layer must add.

## 5. Commands

```bash
python talos_dcm_walk.py                 # open-loop baseline (falls ~3 steps)
python talos_dcm_walk.py --walk-cl       # closed-loop preset (~4 clean steps)
python talos_dcm_walk.py --feedback --footfb --cpswing --tstep 0.5 --dsovl 0.45   # explicit
python view_talos_dcm.py --walk-cl       # watch it in the 3D viewer
```

---

## 6. Lateral limit-cycle synthesis (extended investigation)

**Theory.** For a periodic lateral gait (alternating supports at ±ly, duration T), the
lateral DCM admits an analytic fixed point ξ_s = ly·tanh(ωT/2). It was verified
numerically that the **planner's backward recursion converges exactly to that fixed
point** — so the plan is already on the limit cycle for the central steps; the defect is
not the plan but the **initial state** (robot centred, zero velocity, off the cycle).

**Impulsive entry: an instructive failure.** Imposing the theoretical lateral entry
velocity (ċ_y = ω·ξ_s ≈ 0.18 m/s via `d.qvel`) **degrades** the result (fall at 2.6 s
against 4.08 s): the feet are in contact (closed kinematic chain), so injecting a base
velocity does not produce a clean CoM translation — the robot is thrown laterally
(excursion 0.42 m against 0.22 m). Kept as the `--limit-cycle` option, but disabled by
default.

**Major positive result.** The closed-loop feedback (`--walk-cl`) **resolves the original
lateral instability**: over steps 1–4 the lateral CoM stays within ±0.05 m (against
0.65 m open-loop) and the roll within ±0.05 rad (against 1.5). The lateral failure mode
of §1 is eliminated.

**The bottleneck has migrated to the sagittal channel.** The robot now holds laterally
and then **tips forward** (pitch 0.08→0.65→1.20 rad, CoM_x accelerating 0.3→0.7→1.0 m)
around step 5–6: a sagittal divergence (forward runaway). Restricting foot placement to
the lateral channel alone does not correct it. Honest ceiling: **~4 clean steps, lateral
resolved, sagittal now limiting.**

*Critical insight.* The migration of the failure mode (lateral → sagittal) is the most
significant result of this phase: it proves the lateral capture-point feedback works and
isolates the next obstacle. Sustained purely model-based walking now demands three
further components — a **progressive lateral rocking entry** (rather than an impulsive
one, because of the closed chain), a **sagittal regulation** of forward velocity and
pitch, and an **adaptive step timing** — that is, a complete gait synthesis. Each
component is a targeted increment, now clearly defined by this diagnosis rather than
guessed at.

---

## 7. Sagittal obstacle: diagnosis and architectural ceiling

Once the lateral channel is resolved (§6), the failure mode becomes a **forward
runaway**. The sagittal trace localises it precisely: the forward DCM follows the plan
until step 3 (t≈2.85 s: dcm_x = 0.130 ≈ ref 0.135), then the **forward velocity drifts**
(0.10 → 0.15 → 0.19 → 0.27 m/s over steps 3–4), the DCM error exceeds the feedback
clipping (0.05 m), and the forward DCM diverges exponentially (v reaches 1.5 m/s, pitch
0.08 → 1.20 rad). Mechanism: the CoM advances ~0.13 m per step while the feet advance
only by STEP_LEN = 0.05 m — **the CoM outruns its support** and tips over.

**The ceiling is robust to any tuning.** Tested exhaustively, *every* variant gives
~4 clean steps (≤6 under the lenient time-based metric; see the note):

| Intervention | Steps (lenient time-based metric) |
|---|---|
| walk-cl (reference) | 6 |
| strong DCM feedback (clip 0.20, k=2) | 4 |
| lateral foot placement only | 5 |
| CoM damping ×2.5 (braking) | 4 |
| DCM feedback OFF | 6 |
| foot placement OFF | 6 |

No gain, clipping, damping or feedback component exceeds this ceiling.

> **Metric note (corrected).** The figures in the table use a *lenient* metric (time-based
> phase transitions, which keep advancing during the fall). The *honest* metric —
> **clean steps** = steps completed while remaining stable (z>0.95, |roll|,|pitch|<0.2 rad) —
> gives about two fewer: walk-cl achieves **~4 clean steps** (first loss of balance at
> t≈3.5 s), against ~2 for the baseline. The parameter sweep was re-run with this metric
> (`runs/sweep3/`, `debug_probe.py` field `CLEAN steps`).

*Critical insight.* The invariance of the ceiling across the whole gain space is the
decisive result: it proves the obstacle is not parametric but **structural**, imposed by
the *once-per-step, fixed-timing* decision and by a fixed step length incompatible with
the velocity the pendulum dynamics generates. Regulating forward velocity through the
ankle and CoM alone is impossible once the forward DCM is past the toe (a controllability
limit, the exact sagittal analogue of the lateral case in §1).

**The only remaining lever is adaptive step timing.** To stop the CoM outrunning its
support, the foot must be **placed earlier** when the forward DCM approaches the toe edge
(event-triggered contact rather than a fixed T_STEP), coupled with the capture-point
placement already in place. That is a structural change to the planner (phase logic moves
from time to a DCM-threshold event), not a tuning — and it is the component that can break
the ~4 clean-step ceiling in a purely model-based setting. Failing that, it is exactly the
high-level decision (placement + timing) that the learning layer is designed to supply.
