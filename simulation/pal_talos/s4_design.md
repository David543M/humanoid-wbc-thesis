# S4 — Loco-manipulation: Design Document (walking + two-handed payload carrying)

*Kickoff: 2026-07-10 | Status: design + first implementation | Scope: static two-arm carrying during flat walking*
*Satellite of the thesis (S4), §9.2 (metrics), §13 (flow), §10 (definitions), §5 (hypothesis H1)*
*Target pipeline: `pal_talos/` (native MuJoCo + ProxQP), subclass of `talos_dcm_walk_timing.py` (S2)*

> **Purpose of this document.** To specify — before freezing the implementation — what scenario S4
> (flat walking while carrying/holding an object, *end-effector error* + *CoM tracking* metrics,
> the thesis) requires beyond the flat walker S2, to settle the manipulation variant
> and the way the manipulation task is injected into the QP-WBC, and to fix the campaign
> protocol. S4 is the **decisive test of hypothesis H1** ("a hierarchical QP-WBC enables
> *stable and simultaneous* locomotion + manipulation coordination") — a hypothesis that the thesis §6 still marks as *open/unfavourable* for want of an executed loco-manipulation scenario.

---

## 1. Objective and positioning

S4 is the only scenario in the campaign that exercises **the research question directly** as
it is locked down by the research question: the *simultaneity* of locomotion + manipulation. S1 (balance),
S2 (flat walking) and S3 (step negotiation) validate the locomotion layer of the framework; none
exercises an end-effector. S4 adds an **end-effector task** to the execution-layer tasks
(CoM, base orientation, posture, swing foot) and measures whether the QP can arbitrate
the whole set under the same rigid-body dynamics and friction-cone constraints, without
sacrificing walking stability.

In line with the thesis, the primary metric is the pair *(end-effector error,
CoM tracking)*, together with the cross-cutting metrics (QP feasibility, solve time, task
success) already instrumented for S1/S2/S3. Its position in the logical chain (§13) is as
follows: S2 provides a walker whose fragility is *already characterised* (70 %, seed-dependent
non-determinism, failure mode = lateral DCM divergence); S4 **grafts** manipulation onto that
substrate and poses an incremental, falsifiable question — do the payload and the EE task
**degrade** the success rate and CoM tracking of S2, and by how much?

> **Critical insight.** The value of S4 for the thesis does not lie in "making TALOS carry an
> object" — many loco-manipulation works have already done so (Sentis 2010; Dietrich 2012; Murooka 2021).
> It lies in the fact that S4 is the **only measurement point for H1**: without it, H1 remains an
> untested conjecture and the contribution reduces to a protocol applied to locomotion alone.
> Grafting manipulation onto the S2 walker *already bounded by a CI* turns H1 into a measurable
> difference (Δ success rate, Δ CoM RMSE, EE RMSE) rather than a qualitative claim — consistent
> with the editorial line of honest reporting adopted for the negative S2 results (Ch6).

---

## 2. The technical obstacle: coupling an end-effector task to a walker with thin margins

At each tick the S2 walker (`DCMWalkT`) builds a QP over `x = [qdd, tau, f]` whose cost
aggregates four weighted tasks `w·‖J·qdd − a_des‖²` (CoM, base orientation, posture, swing
foot during single support), subject to the dynamics equality `M·qdd − Sᵀ·tau − Jcᵀ·f = −h` and
friction-cone plus torque-limit inequalities (`talos_dcm_walk.py::control`). Adding
manipulation raises three distinct difficulties.

1. **Where should the EE task be injected?** The QP-WBC formulation absorbs an additional task
   naturally: it suffices to add a term `W_EE·‖J_EE·qdd − a_EE‖²` to the cost, where `J_EE` is
   the translational Jacobian of the hand and `a_EE` a PD-type desired acceleration. The solver
   then arbitrates manipulation and locomotion *within the same program*, under the same constraints —
   precisely the advantage claimed for the QP-WBC over task-switching.
   The difficulty is not theoretical but one of **relative authority**: a `W_EE` weight that is too high steals
   from the CoM and the swing foot the degrees of freedom and torques needed for balance;
   too low, and the hand drifts and the EE metric blows up. `W_EE` is therefore the central
   arbitration parameter of S4.

2. **What does "the object" represent?** Two models are defensible and **not equivalent**:
   - *Known payload*: the mass of the object is added to the hand bodies in the MuJoCo model. The
     WBC computes `M` and `h = qfrc_bias` on **that same loaded model** → the gravity term of the
     payload enters `h` and the controller compensates it in feedforward. What is tested is then
     *coordination* (holding the hand steady while walking), not mass estimation.
   - *Unknown payload (disturbance)*: the mass is absent from the WBC model; an external vertical
     force is applied to the hands (`xfrc_applied`). The weight becomes an **unmodelled
     disturbance** rejected by the feedback loops alone (CoM, EE, posture). What is tested is then
     *robustness* to an unidentified payload.
   These two cases correspond to two real physical regimes (weighed object vs surprise object) and
   provide a natural **ablation**, relevant to H3 (contact/task-priority > naive
   control).

3. **Should the EE task "follow the body"?** Carrying an object means holding the hand **fixed
   relative to the torso**: when the base translates and rotates during walking, the hand
   reference must translate/rotate with it, otherwise the EE task fights the walking motion. The
   task must therefore be formulated in terms of **relative hand↔base motion** (position AND velocity), and not
   as a fixed target in the world frame. A fixed world target would amount to asking the robot to
   leave its hand motionless while it advances — the opposite of carrying.

| Manipulation variant | What it tests | Cost / risk | S4 decision |
|--------------------------|------------------|---------------|-------------|
| **Static two-arm carrying** (hands fixed w.r.t. torso, payload carried) | Pure H1: simultaneous locomotion + end-effector holding | Low: no grasp contact, no sequencing | **SELECTED (1st pass)** |
| World-target tracking (hand follows an independent trajectory) | Explicit hand/base decoupling | Medium: moving reference, tracking while walking | Future work / ablation |
| Reach → grasp → carry | Complete realistic cycle | High: grasp contact + state machine ⇒ close to the S3 pitfalls | Future work |

> **Critical insight.** The choice of static two-arm carrying is not a convenience but an
> **experimental discipline**: it isolates the variable H1 asks about (can *one* end-effector task
> be added without breaking the gait?) by neutralising two orthogonal sources of risk — grasp
> contact (which would reopen the contact-model debate of Ch3/Ch6) and event-driven
> sequencing (whose incompatibility with the walker's adaptive timing was demonstrated by S3). Any
> richer variant introduced now would confound a possible failure of H1 with a failure of
> grasping or of the sequencer. The symmetric two-arm configuration (David's choice) additionally reduces the
> net lateral disturbance from the payload, which gives H1 its fairest chance before the
> asymmetric variants.

---

## 3. Approach selected for the first pass: base-relative two-handed EE task inside the QP

**Decision: static two-handed carrying, payload modelled as mass added to the hand bodies
("known" case) with an external-force ablation ("unknown" case), EE task injected as an additional
cost term in the S2 walker's QP.** Rationale: consistency with the thesis risk "framework too complex → modular design" (§8) and with the S1→S2→S3 line (subclass, core
untouched).

Mechanics (overridden in `DCMWalkS4(DCMWalkT)`; `talos_wbc.py`, `talos_dcm_walk.py`,
`talos_dcm_walk_timing.py` **unchanged**):

1. **End-effector bodies.** `arm_left_7_link` and `arm_right_7_link` (last arm segment, parent
   of the gripper) serve as hand frames. Verified present in `talos.xml`; TALOS = 2 × 7 arm DOF
   (14 actuated arm joints), already in the QP command vector.
2. **Base-relative reference frozen at init.** For each hand, the local pose
   `p_local = R_baseᵀ (p_main − p_base)` is recorded at the initial instant (flexed *home* posture). At each
   tick, the world reference is reconstructed as `p_ref = p_base + R_base · p_local`: the hand is
   thus commanded to remain **rigidly attached to the torso**, so the carried object stays steady
   relative to the body while the robot walks.
3. **EE task in relative motion.** The hand Jacobian `J_main` (translational, via
   `mj_jac`) is formed together with the Jacobian of the **reference point rigidly attached to the base** `J_ref`
   (`mj_jac` of the point `p_ref` on `base_link`). The task drives the *relative* motion:
   `J_rel = J_main − J_ref`, `a_EE = KP_EE·(p_ref − p_main) − KD_EE·(J_rel · qvel)`, added to the cost
   by `add(J_rel, a_EE, W_EE)`. Formulating the task on `J_rel` guarantees that it demands **no**
   torque for the common hand+base rigid-body motion (walking "carries" the hand along
   for free) and penalises only the hand-vs-torso deviation — this is what distinguishes *carrying* from
   *world-frame holding*.
4. **Payload.** `--payload M` (kg) split 50/50: known case = `m.body_mass[main] += M/2` before
   the WBC is built (⇒ the QP's `M` and `h` include the payload, feedforward compensation); unknown
   case (`--payload-unknown`) = `d.xfrc_applied[main] = [0,0,−(M/2)g]` at each tick, with the WBC model
   unloaded (pure disturbance). No new scene: the payload is an inertial property /
   a force, not a grasped body.
5. **EE instrumentation.** At each tick, `e_EE[side] = ‖p_main − p_ref‖` (world) is logged per
   hand; RMSE and peak are reported, separated by regime — *initial transfer* vs *established walking* vs *final
   balance* — for comparison against the §9.2 thresholds.

> **Critical insight.** Injecting the EE task as a simple additional cost term is
> exactly what the thesis claims as the strength of the QP-WBC over strict task-priority
> (null-space) schemes: no hierarchical projection, no switching, a single program under
> hard constraints. But this elegance has a measurable price — the QP is a **weighted-sum**
> arbitration, and therefore *soft*: nothing guarantees *a priori* that balance prevails over
> manipulation when the two conflict (arm extended near a torque limit, or payload
> pulling the CoM outside the support polygon). The question "does the QP sacrifice the gait to hold the
> hand, or the reverse?" is empirical and will be read directly from the pair (EE error, CoM RMSE)
> along a trial: this is the heart of the S4 result, and the temptation to
> pre-tune it to a flattering compromise must be resisted — the `W_EE` sweep (§6) must expose the frontier, not
> hide it.

---

## 4. Design parameters

Starting values (status: *design commitment*, not grounded in the literature unless stated; to be
refined at bring-up):

| Parameter | Starting value | Rationale / to be verified |
|-----------|------------------|----------------------------|
| Payload `M` | **2.0 kg** (2 × 1.0 kg) | Order of magnitude of a manipulable object; ~2.5 % of the TALOS mass (~95 kg). To be swept over {0, 1, 2, 4, 6} to map the degradation. |
| EE task weight `W_EE` | **50** | Between posture (8) and CoM (200): the hand must be held firmly but **never** at the expense of balance. To be swept over {10, 50, 100, 200}. |
| EE gains `KP_EE, KD_EE` | **400, 40** | Hand stiffness comparable to the foot gains (KP_SW=420); damping ~critical. |
| End-effector bodies | `arm_{left,right}_7_link` | Last arm segment (gripper parent). Alternative: gripper base — a ~12 cm offset, not critical for carrying. |
| Initial arm posture | keyframe *home* | Arms along the body (default posture). An "object in front" posture (flexed elbows) would be more realistic but changes the lever arm; possible ablation. |

> **Critical insight.** The decisive parameter is the ratio `W_EE / W_COM`. Too high, and the robot
> favours a perfect hand and lets the CoM diverge (an S2-type fall, aggravated by the payload);
> too low, and the hand loses the object (EE error out of threshold) but the gait survives. There is **no**
> theoretical reason to prefer one setting: the value of S4 lies in *tracing this arbitration
> curve*, not in finding a single point. Arm posture is the second hidden lever:
> arms extended along the body minimise the payload's lever arm on the CoM (favourable case),
> arms extended forwards maximise it (severe case) — to be stated explicitly so as not to
> overestimate H1 using the most lenient geometry.

---

## 5. S4 metrics

### 5.1 Primary metrics (§9.4)

| Metric | Operational definition | Instrumentation |
|----------|---------------------------|-----------------|
| **End-effector error** | `‖p_main − p_ref‖` per hand (world frame), the reference being the torso-rigid pose. RMSE and peak, separated by regime. | New: hand↔reference distance per tick, logged left+right. |
| **CoM tracking** | RMSE of the CoM trajectory (xy) vs the DCM reference, **already** in S2's `log["com_err"]` — reused as is for a direct loaded vs unloaded comparison. | **Already available** (`DCMWalkT`). |

### 5.2 Cross-cutting metrics (reported, the thesis thresholds)

QP feasibility rate (> 99 %), friction-cone compliance (100 % by construction), QP solve time
(< 1 ms mean / < 5 ms p99), task success rate (3 m walk + EE held below threshold + final balance,
> 90 %, Wilson 95 % CI, N ≥ 20). EE threshold (§9.2, status *extrapolated*): **RMSE < 2 cm** in a static
reach; **peak < 5 cm** while walking — to be reused, while stressing its ungrounded status.

> **Critical insight.** Since CoM tracking is strictly the S2 metric, S4 offers a
> **clean A/B comparison** (same walker, same seed, payload on/off): this is the strongest
> argument in the whole chapter, because it isolates the effect of manipulation *all else being
> equal*. The EE threshold, by contrast, is the weak link: the thesis gives it as
> *extrapolated* (TALOS workspace via Pinocchio), with no experimental source. It must therefore be presented
> as a *design commitment*, and **relative magnitudes** should be favoured (EE error loaded vs
> unloaded, EE error walking vs static) over a binary verdict against an arbitrary threshold — on
> pain of reopening the "arbitrary metrics" reviewer criticism (§18).

---

## 6. Campaign protocol

Consistent with S1/S2/S3 (Wilson CI, N ≥ 20, reproducible seed) and with the **non-determinism
caveat** inherited from S2 (the reproducible unit is the aggregate rate, not the per-seed label):

1. **Deterministic bring-up** (single seed, nominal 2 kg payload): verify that a 3 m walk with
   two-handed carrying is possible before any statistics. Deliverable: `s4_run.npz` + video, with the
   EE error(t) and CoM err(t) curves overlaid.
2. **Payload A/B**: same seed, `--payload 0` vs `2` — quantify the Δ CoM RMSE and Δ success rate due
   to the payload alone (isolating the manipulation effect).
3. **Arbitration sweep**: `W_EE ∈ {10, 50, 100, 200}` × `M ∈ {0, 2, 4, 6}` — map the
   EE error ↔ CoM RMSE ↔ survival frontier. Not statistical: a feasibility probe.
4. **Known/unknown ablation**: `--payload` (mass in the model) vs `--payload-unknown` (external force)
   — relevant to H3 (feedforward vs pure rejection).
5. **Statistical campaign**: at the best operating point, a batch of N ≥ 20 seeds (reusing the
   `s2_batch.py` template) → success rate + Wilson CI; reporting EE (RMSE/peak/CI), CoM RMSE, QP feas, ctrl p99.

> **Critical insight.** Reusing `s2_batch.py` guarantees methodological comparability across S1–S4
> (an asset for the "reproducible protocol" contribution). But the per-seed non-determinism
> diagnosed in S2 (discrete bifurcation of the QP active set) **will propagate** to S4, and the
> EE task **adds degrees of freedom to the arbitration** — hence potentially new
> marginal solver decisions. A CI at least as wide as S2's at equal N must be anticipated
> and, as for S2, the **aggregate rate** must be reported rather than the fate of a single seed. The A/B point (same
> seed, payload on/off) is the only one that partly escapes this caveat and must therefore carry
> the main argument.

---

## 7. Code touchpoints (implementation summary)

Subclass `DCMWalkS4(DCMWalkT)` — no modification of `talos_wbc.py`, `talos_dcm_walk.py`,
`talos_dcm_walk_timing.py`:

- `control()` **overridden**: a faithful copy of `DCMWalk.control()` (same QP assembly, same
  constraints) + insertion of the two EE tasks `add(J_rel, a_EE, W_EE)` before regularisation.
  The copy is deliberate (no hook in the base class) and commented as such — the same discipline as the
  S3 QS sequencer.
- payload injection at construction (`m.body_mass`, known case) or per tick (`d.xfrc_applied`,
  unknown case).
- base-relative EE references frozen at init; recomputed per tick.
- new log `ee_err_L`, `ee_err_R` + regime (transfer / walking / end).
- `update()` **inherited unchanged** from `DCMWalkT`: manipulation does not touch the gait
  planning layer (the ZMP/DCM plan remains that of S2).
- `_grf_z` / `_foot_contact` / `_land_metrics_tick`: reused without change.

> **Critical insight.** Inheriting `update()` untouched is what makes S4 defensible: the
> locomotion layer remains *bit-for-bit* that of S2, so any difference in success rate is
> attributable to the EE task + payload alone, and not to opportunistic re-planning. It is the
> same traceability logic that preserved S1 during the development of S2/S3. The only technical
> debt is the copy of `control()`: it will have to be re-synchronised if the base `control()`
> evolves — to be flagged at the top of the file.

---

## 8. S4-specific risks

| Risk | Type | Mitigation |
|--------|------|------------|
| `W_EE` too high ⇒ CoM diverges (fall aggravated by the payload) | Arbitration | Sweep §6.3; bound `W_EE` below `W_COM`; read the pair (EE, CoM) instead of a single point |
| Payload moves the CoM outside the polygon (arms forward) | Mechanical | Arms-along-body posture in the 1st pass; state the geometry; posture ablation |
| Arm/shoulder torque limits saturated under load | Technical | Report the feasibility rate; the QP degrades gracefully (grav+PD fallback) already in place |
| QP non-determinism amplified by the EE task | Reproducibility | Report the aggregate rate (not the per-seed label); same-seed A/B for the main argument |
| Ungrounded EE threshold (§9.2 extrapolated) | Integrity | Declare it a *design commitment*; favour relative loaded/unloaded magnitudes |
| S2 gait already at 70 % ⇒ S4 ≤ 70 % | Narrative | Frame S4 as a *differential* vs S2, not as an absolute; honest reporting (cf. S2/S3) |

> **Critical insight.** The dominant risk is, as for S3, **narrative** before it is technical:
> building S4 on a walker at 70 % mechanically exposes it to a third rate below the 90 % threshold.
> The remedy is not to over-tune in order to "pass", but to reframe the S4 question as a **delta**
> measured relative to S2 (does the payload cost X points of success and Y mm of CoM RMSE?), which
> remains informative *even if the absolute figure is low*. An S4 at 55 % with a CoM degradation of +8 mm under
> 2 kg is a publishable result; an S4 "tuned to 92 %" on the most lenient posture and payload,
> without a sweep, would be fragile in peer review. This document commits to the former course.

---

## 9. Next actions

1. **Implement `DCMWalkS4`** (`talos_s4_locomanip.py`) — delivered with this document (bring-up not
   executed: the sandbox truncates the .py files; run it on a conda machine / Vertex AI).
2. **Deterministic bring-up** → `s4_run.npz` + video; verify feasibility + EE holding.
3. **Payload A/B** (payload 0 vs 2, same seed) → first differential figure for H1.
4. **`W_EE` × M sweep** → arbitration frontier.
5. Write **Ch5 §5.7** (S4) once bring-up is obtained, following the §5.5/§5.6 template.
6. **Verify and insert** the cited loco-manipulation references (Sentis2010, Dietrich2012, Murooka2021)
   into `references.bib` only after validating vol/pp/DOI (the thesis rule — never
   invent).

**Bring-up commands:**
```
python talos_s4_locomanip.py --seed 0 --payload 2.0 --save s4_run.npz         # known 2 kg carry
python talos_s4_locomanip.py --seed 0 --payload 0                             # unloaded A/B
python talos_s4_locomanip.py --seed 0 --payload 2.0 --payload-unknown         # payload as disturbance
python talos_s4_locomanip.py --seed 0 --payload 2.0 --wee 100 --viewer        # visual inspection
```

---

*This document is the S4 design specification. It modifies no validated result (S1) and no part of
the control core (QP-WBC, DCM planner, S2 walker). Any implementation must conform to it
or flag the deviation.*
