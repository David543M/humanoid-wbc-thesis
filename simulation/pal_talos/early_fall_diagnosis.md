# S2 — Early-fall diagnosis (robustness pass, option C)

*Session 2026-07-08. Data: N=50 batch (`s2_batch/s2_seed*.npz`) + reconstruction
of the initial perturbations. Companion to `RESULTS.md` / `WBC_planning_diagnosis.md`.*

## Question
The N=50 batch gives a 70% success rate; of the 15 falls, 7 are **early**
(t < 16 s, < 2 m): seeds 3, 15, 30, 32, 38, 48, 49. Can the rate be raised
by targeted hardening, or is the fragility structural?

## 1. The initial condition does NOT predict an early fall
`--seed` injects a single reproducible noise burst on the joint velocities
(`np.random.default_rng(seed).normal(0, 0.01, nv=50)` on `d.qvel`). Correlating
this kick with the outcome over the 50 seeds:

- total kick magnitude **identical** across groups: successes 0.069, early
  falls 0.070, late falls 0.070; `corr(|kick|, t_chute) = −0.01`.
- even seed 15 — the only **reproducible** faller (0.66 m at 9.4 s in N=20 *and* N=50)
  — has a modest baseline kick (0.021), nothing extreme.

**Conclusion:** early collapse is not "a large initial shock". This is
consistent with result P1e (the outcome is set in part by a discrete downstream
bifurcation, not by the input).

## 2. Every fall is a terminal DCM divergence
The common signature is clear: the DCM error `|ξ − ξ_ref|` blows up to
**870–1060 mm** in the final instants, against **1–3 mm** for the successes
(which close cleanly). The fall IS a divergence of the divergent component.

## 3. Two distinct mechanisms (step durations)
Looking at the last 8 steps before the fall:

| Mechanism | Seeds | Signature |
|---|---|---|
| **Chattering** | 3, 30, 32, 48 | 5–6 of the 8 steps below 0.20 s, emergency steps at **0.075–0.09 s** (≪ T_MIN=0.24) → stumbling into ultra-short steps |
| **Clean divergence** | 15 (reproducible) | 0 short steps: the last 8 steps are **normal** (0.26–0.35 s) and the DCM diverges all the same |

Ultra-short steps are not a cause but a **symptom**: the body tips, the
stance foot strikes the ground very early (event-driven touchdown), the plan re-anchors
on a bad contact → spiral. Seed 15 shows that a fall can happen **without** chattering.

## 4. The capture envelope is not the limiting factor
Measuring the largest DCM excursion the runs **survive**:

- successes: max survived `ξ` excursion = **286 ± 66 mm** (up to **404 mm**), then recovery.
- fallers, just before the runaway (80–90% window of the episode): only
  **72–147 mm** — *well inside* the survivable band — then a blow-up to ~1000 mm.

**Fallers diverge from a calmer state than the ones successes routinely recover
from.** The fall is therefore not an excursion that is "too large" crossing a
smooth limit: it is a **sudden, event-triggered runaway** starting from an
ordinary state.

## 5. Verdict for the robustness pass
The three standard tuning levers are **ruled out by the data**:
- it is not the initial condition (§1, zero correlation);
- it is not the capture envelope / the step size (§4, the envelope is already
  sufficient — 286 mm survived);
- chattering is a downstream symptom, not the root cause (§3, seed 15).

The root cause is the **discrete bifurcation** of P1e: a rare event (a QP
active-set switch / a contact inclusion right at the threshold) triggers a
sudden DCM runaway from a healthy state. Tuning gains/offsets/timing cannot
remove a discrete trigger; it merely shifts which seeds
fall (already observed from N=20 → N=50). The underlying fix remains the **co-design** identified
earlier: an explicit double-support duration in the DCM recursion (the plan assumes
an instantaneous support switch), so as to remove the thin margin that makes the system
sensitive to the event. → *future work*, not a tuning patch.

**Testable mitigation (should one wish to try anyway):** an early-runaway detector
on the growth *rate* of `ξ` (not its level), triggering a stabilised
stop (halt sagittal progression, widen the support) BEFORE the point of
no return. Uncertain payoff — the runaway is fast (§4) — but falsifiable in a single
batch run. Failing that, accept the ~70% ceiling as the bound of this
flat-foot DCM/ZMP architecture and document it.

## Brief critical insight
The evaluation protocol did exactly its job: it turned three
tuning intuitions ("bad seed", "steps too short", "envelope too
narrow") into three hypotheses *refuted* by measurement, and converged with the
reproducibility analysis (P1e) on a single structural cause. The lesson is not
that the controller is poor — its capture envelope (≈ 290 mm) is healthy — but
that **driving a thin-margin limit cycle close to the constraint boundaries leaves
the outcome to be decided by a discrete event**, beyond the reach of tuning. That is a
design argument (Ch6), not a bug to be fixed.
