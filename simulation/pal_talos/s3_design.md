# S3 — Stair Climbing: design document (three-step ascent)

*Kickoff: 2026-07-09 | Status: design (no S3 code written yet) | Scope: ascent only*
*Satellite of the thesis (S3), §9.2 (metrics), §13 (flow), §10 (definitions)*
*Target pipeline: `pal_talos/` (native MuJoCo + ProxQP), a subclass of `talos_dcm_walk_timing.py`*

> **Purpose of this document.** Specify — before a single line of code — what the S3 scenario
> (a three-step ascent, metrics *foot clearance* + *contact forces*) demands beyond the flat
> S2 walker, settle the planning approach, and fix the campaign protocol. This document is the
> "problem → method" component of S3; it precedes the implementation and the future §5.6 (Ch5).

---

## 1. Objective and positioning

S3 extends the validation of the hierarchical WBC framework (centroidal planning + QP-WBC) to
**crossing discrete vertical obstacles**: climbing three steps. Following the thesis, the primary
metric is the pair *(foot clearance, contact forces)*, with the transverse metrics reported
alongside (CoM tracking, task success, QP feasibility, solve time) already instrumented for
S1/S2.

Its position in the logical chain (§13) is direct: S1 validates static balance, S2 flat
locomotion; S3 tests the **robustness of the same QP-WBC executor against a break in the
flat-terrain assumption**. It is the first time in the campaign that the ground geometry stops
being a half-plane at constant z = fz. S3 introduces **no** new manipulation task (reserved for
S4) and no stochastic perturbation (reserved for S5): the only added variable is the contact
altitude.

> **Critical insight.** The value of S3 for the thesis is not "making TALOS climb" — many works
> have done that (Caron-Kheddar-Tempier 2019 on HRP-4) — but measuring, under a reproducible
> protocol bounded by confidence intervals, **how far the S2 pipeline degrades once the planar
> LIPM assumption is lifted**. A quantified failure (insufficient clearance, a GRF peak out of
> bounds) carries as much evidential value as a success, provided it is attributed to an
> identified cause. This is the editorial line already adopted for the negative S2 result (Ch6).

---

## 2. The technical obstacle: breaking the constant-height LIPM assumption

The current planner (`talos_dcm_walk.py`) rests on the **Linear Inverted Pendulum Model** at
constant CoM height. Three lines fix that assumption (verified at code level):

```python
self.fz    = p0[self.left][2]          # height of BOTH feet = a single scalar (flat ground)
self.zc    = subtree_com[base][2]      # CoM height frozen at init
self.omega = np.sqrt(9.81 / self.zc)   # constant LIPM frequency, computed ONCE
```

and the swing trajectory systematically places the foot at that same altitude:

```python
swing_pos[2] = self.fz + STEP_H * np.sin(np.pi * s)   # bell above a single flat ground
goal = np.array([zmp[k+1][0], zmp[k+1][1], self.fz])  # foothold target at constant fz
```

On a staircase this assumption breaks at three levels:

1. **Variable foothold altitude.** Each step `k` has its own height `fz_k = k · h_riser`. A
   scalar `self.fz` can no longer describe the foothold target, the touchdown detection, or the
   swing foot's ground clearance.
2. **Non-constant CoM height.** Climbing raises `z_c` by roughly `h_riser` per step. But
   `omega = sqrt(g/z_c)` governs the whole DCM recursion. Keeping it frozen introduces a
   systematic model error that worsens with every step. This is the heart of the problem: the
   planar DCM dynamics (Englsberger 2015, 2D form) is valid only at constant `z_c`.
3. **Vertical component of the dynamics.** The planar LIPM assumes a vertical force equal to the
   weight (zero vertical CoM acceleration). Climbing demands net vertical work (`m·g·h_riser` per
   step) that the 2D model does not represent — it appears as an unmodelled perturbation on the
   sagittal channel.

Two families of answer exist in the literature, and they must be distinguished:

| Approach | Reference (verified) | Idea | Cost of integration into `pal_talos/` |
|----------|----------------------|------|--------------------------------------|
| **3D-DCM (eCMP/VRP)** | Englsberger, Ott, Albu-Schäffer, *IEEE T-RO* 31(2):355-368, 2015 | Extends the DCM to 3D; the *Virtual Repellent Point* encodes both the direction **and** the magnitude of the total force → handles variable `z_c` | High: rewriting the DCM recursion (2D→3D), a new vertical CoM reference |
| **VHIP capturability** | Caron, Escande, Lanari, Mallein, *IEEE T-RO*, 2019 (*Capturability-based Pattern Generation for Walking with Variable Height*; HRP-4 stair demo) | Pattern generator on a variable-height pendulum, capturability over uneven terrain; solved fast enough for real-time MPC | Very high: a new pattern generator and a dedicated optimisation scheme |
| **Plateau-wise LIPM (quasi-static)** | A pragmatic extension of the existing pipeline (no new theory) | Each step = a plateau at locally constant `z_c`; `omega` recomputed per step, vertical transitions absorbed in a slow double support | Low: overload `self.fz` → `fz_k`, `omega` per plateau, swing bell relative to max(takeoff, landing) |

> ⚠️ **Source-integrity note.** The `Caron2019` **already present** in `references.bib` is the
> **ICRA** paper *Stair Climbing Stabilization of the HRP-4 … Whole-body Admittance Control*
> (admittance stabilisation, NOT pattern generation). The **T-RO 2019** paper
> *Capturability-based Pattern Generation with Variable Height* is a **distinct** work and will
> need its own BibKey (proposed: `Caron2019VHIP`) **with volume, pages and DOI verified before
> insertion** — the thesis rule (§19): never invent. Englsberger2015 is likewise to be added
> (`Englsberger2015`, T-RO 31(2):355-368) after DOI verification.

> **Critical insight.** The choice of approach is a risk/time trade-off, not a matter of
> theoretical purity. 3D-DCM and VHIP are the "correct" answers but require replacing the
> planning core that took weeks to stabilise for S2 — and that remains fragile (70 %). Importing
> them now would stack two unresolved sources of instability. The defensible strategy for a
> **first pass** is the plateau approach: it is honest about its limits (it *approximates* the
> vertical dynamics), it reuses a validated QP executor, and it turns the question into a testable
> hypothesis — "up to what step height does the quasi-static approximation hold?" — rather than
> into an open-ended project.

---

## 3. Approach adopted for the first pass: plateau-wise LIPM + a climbing double support

**Decision (to be confirmed): the quasi-static plateau approach**, with an explicit switch to
3D-DCM/VHIP documented as *future work* should the approximation fail beyond a height threshold.
Justification: consistency with the thesis risk "framework too complex, insufficient time →
modular design, reduce horizon/DoF" (§8.4), and with the S2 line (iterate on a core that is
mastered rather than replace it).

Proposed mechanics (by overload; `talos_dcm_walk.py` and `_timing` **unchanged** — an S3
subclass):

1. **3D footstep plan.** The `zmp[k]` plan (x,y) is augmented with an altitude `fz_k` per
   foothold: feet on the ground for the first footholds, then `fz_k = n_k · h_riser` where `n_k`
   is the step number targeted by foothold `k`. Climbing cadence: one foot per step
   ("step-over-step" rather than "step-together", closer to S2 and avoiding a prolonged double
   support on a single step).
2. **`omega` per plateau.** Recompute `omega_k = sqrt(g / z_c,k)` with `z_c,k` the nominal CoM
   height on step `k` (≈ `z_c,0 + n_k · h_riser`). The CoM rise is treated as a reference ramp
   during double support, not as active DCM dynamics.
3. **Raised swing bell.** Ground clearance becomes relative to the **highest point** between
   takeoff and landing: `z_swing(s) = max(fz_takeoff, fz_land) + STEP_H · sin(π s)`, with
   `STEP_H` increased (see §4) to clear the step nosing. This is the modification that addresses
   the *foot clearance* metric directly.
4. **Per-step touchdown detection.** `_foot_contact` already tests contact with the worldbody
   (body 0); if the steps are `geom` children of the worldbody, the detection stays valid
   **without modification**. The height guard `foot_z <= fz + FOOT_TOL`, however, must target
   `fz_land,k` rather than the global scalar.
5. **Vertical transfer in double support.** Between two steps, lengthen the double-support window
   (`DS_OVL`) to let the CoM reference rise quasi-statically before engaging the next swing —
   this is where the vertical work `m·g·h` is supplied, outside the DCM recursion.

> **Critical insight.** The plateau approximation displaces the problem rather than solving it:
> it assumes the vertical transition can be made "slow relative to the DCM dynamics" through
> double support. That assumption is **false beyond a certain cadence** — if the robot has to
> climb fast, the CoM accelerates vertically and the sagittal channel sees a perturbation the
> planar controller does not compensate. The protocol (§6) must therefore **sweep step height and
> cadence** to map the validity boundary, which turns this limit into a measurable result rather
> than a blind spot.

---

## 4. Geometry of the MuJoCo scene (`scene_stairs.xml`)

The current flat scene (`scene_motor.xml`) has only a plane `geom`. S3 requires a new scene
including `talos_motor.xml` and adding 3 steps as `box` geoms that are children of the worldbody
(hence `geom_bodyid == 0`, which preserves the existing contact-detection logic).

Proposed design parameters (to be frozen after a kinematic check on the loaded TALOS model —
**starting values, status: *design commitment*, not anchored in the literature**):

| Parameter | Starting value | Justification / to be checked |
|-----------|------------------|----------------------------|
| Riser height `h_riser` | **0.10 m** | Conservative against human steps (~0.17 m); the order of magnitude of robot stair demos. To be swept over {0.05, 0.10, 0.15}. |
| Tread depth `d_tread` | **0.30 m** | ≥ TALOS sole length (~0.20 m) + support margin. Check the real sole in `talos.xml`. |
| Step width | **1.0 m** | Wide: removes any spurious lateral coupling for this first pass. |
| Number of steps | **3** | Fixed by the thesis |
| Target ground clearance `STEP_H` | **≥ 0.12 m** | Must exceed `h_riser` plus a safety margin (≥ 2 cm) so as not to strike the nosing; the flat baseline is 0.04 m, far too little. |

Anticipated XML structure (sketch, to be implemented in the next pass): a flat starting landing,
then 3 `box` geoms stacked as a staircase (each step a block whose top face is at `n · h_riser`),
and a flat arrival landing for the final balancing phase (reusing the `ended` logic of `_timing`).

> **Critical insight.** The decisive parameter is the ratio `STEP_H / h_riser`. Too little
> clearance and the foot strikes the riser (a clearance failure); too much lengthens the flight,
> delays touchdown and — per the S2 mechanics — lets the lateral DCM diverge further before
> contact, which was the dominant cause of the S2 falls. S3 therefore inherits S2's timing
> fragility **directly**: climbing demands higher steps, and higher steps aggravate precisely the
> failure mode already diagnosed. This interaction must be watched from the first run.

---

## 5. S3 metrics

### 5.1 Primary metrics (§9.4)

| Metric | Operational definition | Instrumentation |
|----------|---------------------------|-----------------|
| **Foot clearance** | Minimum height of the lowest point of the swing sole **above the nosing of the step being crossed**, over the whole flight phase. Positive = cleared; negative = collision. | New: sole-to-step-edge distance per tick during swing (from the foot's `d.xpos` plus the known step geometry). |
| **Contact forces** | Peak and mean normal GRF at touchdown on each step; make/break distribution (bounces). | **Already available**: `_grf_z(fb)` + the `_land_metrics_tick` window (reused unchanged from `_timing`). |

### 5.2 Transverse metrics (reported, thesis thresholds)

CoM tracking RMSE (< 3 cm steady state), QP feasibility rate (> 99 %), friction-cone compliance
(100 % by construction), QP solve time (< 1 ms mean / < 5 ms p99), task success rate (complete
ascent of the 3 steps + final balance, > 90 %, Wilson 95 % CI, N ≥ 20).

> **Critical insight.** *Foot clearance* is the campaign's only **new** metric and the only one
> without a threshold anchored in the literature: the thesis does not reference one. It will have
> to be declared explicitly as a *design commitment* (like the other unanchored thresholds) and
> given a justified threshold — at minimum "clearance > 0 on 100 % of successful crossings",
> ideally a positive margin (say ≥ 2 cm) linking clearance to robustness. This threshold must not
> be left implicit, on pain of reproducing the reviewer objection "arbitrary metrics" (§18).

---

## 6. Campaign protocol

Consistent with the S1/S2 methodology (Wilson CI, N ≥ 20, reproducible seed) and with the
**non-determinism caveat** documented for S2 (the reproducible unit is the **aggregate rate**,
not the per-seed label):

1. **Deterministic bring-up** (single seed, no noise): confirm that a nominal ascent of the 3
   steps is possible before any statistics. Deliverable: one `s3_run.npz` trajectory + a video.
2. **Design sweep**: `h_riser ∈ {0.05, 0.10, 0.15}` × cadence (through `T_STEP`) to locate the
   validity boundary of the plateau approximation (§3). Not statistical — a feasibility probe.
3. **Statistical campaign**: at the best design point, a batch of N ≥ 20 seeds → success rate +
   Wilson CI, on the exact model of `s2_batch.py`. Report: clearance (min/mean/CI), GRF
   (peak/mean), CoM RMSE per step, ctrl p99.
4. **Ablations**: high against low ground clearance `STEP_H`; long against short climbing double
   support.

> **Critical insight.** Reusing `s2_batch.py` guarantees methodological comparability between S1,
> S2 and S3 — an asset for the "reproducible evaluation protocol" argument. But the per-seed
> non-determinism diagnosed in S2 (a discrete bifurcation of the QP active set) **will propagate
> to S3** and will probably be **amplified**: contacts grazing the step edges multiply the
> marginal contact-inclusion decisions. One must therefore expect the aggregate S3 rate to carry
> a wider CI at equal N, and size N accordingly (perhaps N ≥ 35, as proved necessary to settle
> S2).

---

## 7. Code touch points (summary for the implementation)

Subclass `DCMWalkS3(DCMWalkT)` — no modification to `talos_dcm_walk.py`, `talos_wbc.py` or
`talos_dcm_walk_timing.py` (the same discipline as the S1→S2 transition):

- scalar `self.fz` → array `fz_k` indexed by foothold (or a `foot_height(k)` function).
- single `omega` → `omega_k` recomputed on a step change.
- `z_swing`: bell relative to `max(fz_takeoff, fz_land)` + an increased `STEP_H`.
- touchdown guard `foot_z <= fz + FOOT_TOL` → target `fz_land,k`.
- new clearance metric (per swing tick) added to the `log`.
- new scene `scene_stairs.xml` (3 `box` children of the worldbody).
- `_grf_z` / `_land_metrics_tick` / `_foot_contact`: **reused unchanged**.

> **Critical insight.** The "subclass only, core untouched" discipline is what kept S1 valid
> throughout the S2 development. Holding to it for S3 is what makes the campaign defensible: if
> S3 fails, the failure is localised in the stair layer, not in a QP executor that would then have
> to be re-validated. Any temptation to "quickly fix" `talos_dcm_walk.py` to make S3 pass would
> break that traceability and retroactively invalidate S1/S2.

---

## 8. S3-specific risks

| Risk | Type | Mitigation |
|--------|------|------------|
| Plateau approximation invalid beyond a threshold `h_riser` | Model | Sweep §6.2: turn the limit into a measured result; switching to 3D-DCM is future work |
| Ground clearance ↔ DCM divergence (high steps aggravate the S2 failure mode) | Technical | `STEP_H` ablation; watch clearance against time-to-fall from the bring-up onwards |
| QP non-determinism amplified by contacts grazing the edges | Reproducibility | Report the aggregate rate (not the seed label); N ≥ 35 if the CI is too wide |
| Sole longer than the tread → overhang | Geometry | Check the real TALOS sole against `d_tread` before freezing the scene |
| Englsberger2015 / Caron2019VHIP sources unverified | Integrity | Verify volume, pages and DOI before inserting into `references.bib` (rule §17-5) |

> **Critical insight.** The dominant risk is not technical but **narrative**: with S2 at 70 %
> (FAIL against the 90 % threshold), launching S3 on the same fragile core exposes the work to a
> second metric below threshold. The answer is not to "do better at any cost" but to frame S3, as
> S2 was framed, as an honest and diagnosed result — the thesis contribution being the *protocol*
> and the *diagnoses*, not a success rate. This document sets that frame before the first run.

---

## 9. Next actions

1. **Verify the TALOS kinematics** (sole length, hip height) on the loaded model in order to
   freeze the starting `d_tread` and `h_riser`.
2. **Write `scene_stairs.xml`** (3 `box` geoms, start and arrival landings).
3. **Write `DCMWalkS3`** (subclass: `fz_k`, `omega_k`, raised bell, clearance metric).
4. **Deterministic bring-up** → `s3_run.npz` + video.
5. **Verify and insert** `Englsberger2015` and `Caron2019VHIP` into `references.bib` (DOIs
   validated).
6. Write **Ch5 §5.6** once the bring-up is obtained.

---

*This document is the S3 design specification. It modifies no validated result (S1) and no part
of the control core. Any implementation must conform to it or flag the departure.*
