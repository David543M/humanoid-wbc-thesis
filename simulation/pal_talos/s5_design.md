# S5 — Perturbation Robustness: design document (impulses during walking)

*Kickoff: 2026-07-10 | Status: design + first implementation | Scope: impulsive pushes on the pelvis during S2 walking*
*Satellite of the thesis (S5), §9.2 (metrics — the "perturbation recovery" flag is OPEN), §13 (flow)*
*Target pipeline: `pal_talos/` (native MuJoCo + ProxQP), above `talos_dcm_walk_timing.py` (S2), UNCHANGED*

> **Purpose of this document.** Specify the last scenario of the S1–S5 protocol: flat-ground
> walking under random external impulses, primary metric *recovery rate* (methodology chapter).
> Decisions taken 2026-07-10: **(1) walker = the dynamic S2 one** (`DCMWalkT`, frozen config) —
> the timing adaptation (Khadiv2020) is precisely the recovery mechanism to be tested;
> **(2) calibration = a magnitude sweep** — the recovery-versus-impulse curve IS the
> calibration, which closes the Ch3 §3.6 flag ("recovery threshold: calibrated jointly with the
> S5 impulse distribution") by measurement rather than by a postulated figure.

---

## 1. Objective and positioning

S5 completes the protocol: S1 validated static balance under pushes, S2–S4 locomotion and
loco-manipulation *without* external perturbation. S5 measures the **dynamic rejection
capability** of the {DCM planner + adaptive timing + QP-WBC} triple: an impulse on the pelvis
displaces the measured DCM, and recovery goes through the mechanisms already in place —
foothold re-planning (capture point, `--cpswing`), step shortening (adaptive timing), the
`[recov]` mode (pure lateral step). S5 therefore tests **no new mechanism**: it quantifies the
S2 ones under controlled stress.

The choice of the dynamic walker (over the QS sequencer) follows from that reading: the QS has
no reactive mechanism at all (a position-convergence state machine), so its failure to recover
from any significant push is predictable and uninformative; the S2 walker, by contrast,
incorporates exactly the timing law whose value in push recovery the literature (Khadiv2020)
claims — S5 is its direct test inside our framework.

> **Critical insight.** S5 has a dual function the other scenarios do not: (i) measuring
> robustness, and (ii) **producing the calibration its own metric lacks**. Table 3.1 leaves the
> recovery threshold open for want of a defensible impulse distribution; postulating
> "150 N × 0.1 s" now would be circular (the reviewer objection "arbitrary metrics", §18). The
> sweep inverts the logic: recovery(I) is measured and the **I₅₀ margin** (impulse at 50 %
> recovery) is reported as a result, instead of judging against an invented threshold. This is
> the same discipline as S2/S3: the honestly measurable quantity first, the binary verdict only
> where an anchor exists.

---

## 2. The methodological obstacle: perturbing a walker that succeeds 70 % of the time

The S2 substrate already falls in about 30 % of trials **without** any push (batch N=50, Wilson
[0.562, 0.809]), with per-seed non-determinism (discrete QP bifurcation). Attributing a
post-push fall to the push is therefore not trivial. Three precautions structure the protocol:

1. **Conditioning**: a trial is *eligible* only if the robot is upright and walking at the
   instant of the push. Pre-push falls are excluded from the denominator (they belong to S2,
   already characterised).
2. **A mag = 0 control arm**: every sweep cell is compared against identical trials (same seeds,
   same code, same window) with no force. The "recovery" rate of the control arm estimates
   baseline survival over the observation window — robustness is read as **Δ against the
   control**, not in absolute terms.
3. **Reproducible unit = the aggregate rate** (inherited from S2): the fate of one seed is not
   reproducible, the per-cell rate is. Wilson CI per cell, N ≥ 10–20 depending on the width
   sought.

> **Critical insight.** The definition of "recovery" is the most attackable point in the whole of
> S5 and must be fixed BEFORE the campaign (a de facto pre-registration): *recovered = upright
> (base_z > 0.8 m) T_REC = 5 s after the end of the impulse, among eligible trials*. The
> continuous quantities (time for the DCM to return inside the band, maximum DCM excursion,
> number of `[recov]` steps) are **reported** but take no part in the verdict — including them in
> the criterion would multiply unanchored thresholds. A reviewer may contest 5 s or 0.8 m; they
> cannot contest that the criterion was fixed before the data.

---

## 3. Adopted approach: an `xfrc_applied` impulse on the pelvis, swept over magnitude × direction

**Decision: a constant external force F applied to the `base_link` body for DUR = 0.1 s
(impulse I = F·DUR), triggered in steady state at a random phase of the step, swept over
magnitude and direction.** The walker, the QP and the planning are **bit-for-bit S2** (frozen
config `--steps 70 --offlat 0.08 --dsovl 0.12 --tmin 0.24`); the perturbation is injected into
the simulation loop, not into the controller — the WBC "knows" nothing.

Mechanics (`talos_s5_perturb.py`, no S2 file modified):

1. **Injection**: `d.xfrc_applied[base, :3] = F · û` over `[t_push, t_push + DUR]`, reset to zero
   afterwards. Force at the CoM of the base body (no applied torque) — a pattern already
   validated in S4 (`--payload-unknown`).
2. **Trigger**: the push fires when the current step `k` reaches `K_PUSH = 12` (limit cycle
   established, far from the start and from the closing step), at a **random phase** of the step
   drawn from `default_rng(seed + 1000)` — decoupled from the initial `qvel` noise (seed) so that
   the control arm shares exactly the same initial state. The random phase samples SS and DS
   without stratifying (single/double-support stratification is an ablation, not the first pass).
3. **Directions**: {+x (from behind), −x (from the front), +y (left), −y (right)} in world frame.
   The lateral channel is the dominant S2 failure mode (lateral DCM divergence) — the expected
   sagittal/lateral asymmetry is a result in itself.
4. **Magnitudes**: F ∈ {50, 100, 150, 200, 250} N × 0.1 s → I ∈ {5, 10, 15, 20, 25} N·s
   (≈ 0.05–0.26 × m·v with m ≈ 95 kg: induced velocity ~0.05–0.26 m/s). Bounds chosen to bracket
   the model's capturability scale (ξ_s ≈ 28 mm, measured S2 envelope: successful runs survive
   DCM excursions of ~286 ± 66 mm); to be adjusted at bring-up if the result is 0 % or 100 %
   everywhere.
5. **Instrumentation**: push tick, eligibility, upright at t_push + T_REC, maximum post-push DCM
   excursion, time to return inside the band (band = max(2 × pre-push median, 50 mm) held for
   0.5 s), number of post-push `[recov]` steps, plus every S2 metric (com_err, ctrl_ms, t_steps,
   touchdown GRF).

> **Critical insight.** Applying the force at the simulation-loop level rather than through a
> controller hook is what makes S5 clean: the controller goes through the push with the same
> information a real robot would have (none), and the observed recovery chain — measured DCM
> divergence → step shortening → capture-point re-placement → possibly a `[recov]` step — is
> entirely attributable to the documented S2 mechanisms. The acknowledged weak point: a force at
> the pelvis CoM is the *standard proxy* but not the only realistic perturbation (torque, a push
> at the shoulder, slippery ground); generalising the conclusion beyond that proxy would be an
> over-reading, to be said in Ch6.

---

## 4. Design parameters

Status *design commitment* (not anchored in the literature unless stated):

| Parameter | Value | Justification / to be checked |
|-----------|--------|----------------------------|
| Impulse duration DUR | 0.1 s | Dominant convention in push-recovery papers (short impulse against T_step = 0.5 s) |
| Magnitudes F | {50, 100, 150, 200, 250} N | A priori bracketing of the capturability scale; refine at bring-up |
| Directions | ±x, ±y world | 4 cells; diagonals are future work |
| K_PUSH | step k = 12 | Steady state (S2: cycle stable from k ≈ 5), margin before the closing step (k = 69) |
| Phase | U[0, 1) × Tk, rng(seed+1000) | Samples SS and DS; decoupled from the initial noise |
| T_REC | 5.0 s | ~10 nominal steps; check that no "slow" fall exceeds the window |
| DCM return band | max(2 × pre-push median, 50 mm), held 0.5 s | Reported quantity, NOT a verdict criterion |
| N per cell | ≥ 10 (sweep), ≥ 20 (cells near I₅₀) | Wilson usable where it matters |

> **Critical insight.** The parameter with the heaviest consequences is the fixed K_PUSH: always
> pushing at the same step standardises the comparison between cells but samples only one point
> of the walking transient. The alternative (an instant drawn uniformly over the whole trial)
> would confound the effect of magnitude with the effect of position in the walking plan. The
> "fixed K + random phase" choice is the compromise: controlled within-cell variance, cycle phase
> covered. If the results suggest strong sensitivity to phase (DS against SS), stratification
> becomes the first ablation to run.

---

## 5. S5 metrics

| Metric | Operational definition | Role |
|----------|---------------------------|------|
| **Recovery rate** | upright at t_push + T_REC \| eligible; Wilson 95 % per cell (mag × dir) | **Verdict** (against the control arm) |
| I₅₀ margin | impulse at 50 % recovery, per direction (interpolated over the sweep) | **Calibration** — closes the Ch3 §3.6 flag |
| Max post-push DCM excursion | max ‖ξ_meas − ξ_ref‖ over [t_push, t_push + T_REC] | Mechanism |
| Time to return inside the band | first instant at which the DCM error < band, held 0.5 s | Mechanism |
| Post-push `[recov]` steps | count of degenerate entries | Mechanism |
| S2 transverse metrics | QP feasibility (> 99 %), ctrl (< 1 ms mean / 5 ms p99), CoM RMSE, touchdown GRF | Protocol continuity |

> **Critical insight.** The verdict/mechanism separation is what spares S5 the S4 trap (the
> EE < 5 cm gate, never met, had turned a metric into a false criterion): ONE binary variable
> decides (upright at T_REC), everything else explains. The per-direction I₅₀ is the scenario's
> most durable deliverable: it is a margin figure in physical units (N·s), comparable between
> controllers and publishable independently of the host walker's absolute success rate.

---

## 6. Campaign protocol

1. **Deterministic bring-up**: seed 0, one +y 100 N push, `--viewer` then headless — check the
   injection (force trace), the eligibility, and that the recovery chain is visible. Deliverable:
   `s5_run.npz`.
2. **Scale probe**: seed 0, ±y, F ∈ {50 … 250} — check that the range does bracket the
   100 % → 0 % transition; adjust the magnitudes otherwise.
3. **Full sweep** (`s5_batch.py`): 4 directions × 5 magnitudes × N ≥ 10 seeds + control arm
   (mag 0, same seeds) → recovery rate + Wilson per cell, recovery(I) curves per direction,
   interpolated I₅₀.
4. **Densification**: N → 20 on the 2 cells bracketing each I₅₀.
5. **Writing Ch5 §5.8** (pattern of §5.5–§5.7) + **updating Ch3 §3.6**: replace the open flag with
   the measured calibration (explicit cross-reference to S5).

> **Critical insight.** Step 5 closes a methodological debt open since 2026-05-04 and turns a
> declared weakness (an unanchored threshold) into a contribution (calibration by measurement).
> The campaign risk is its cost: 4 × 5 × 10 + control ≈ 210+ trials at ~30–60 s — within an
> overnight budget on the conda machine, or delegable to Vertex AI (Session 16 protocol). Do NOT
> cut the control arm to save time: without it, the recovery rate of a walker with a 70 % baseline
> is uninterpretable.

---

## 7. Code touch points

`talos_s5_perturb.py` — reuses `DCMWalkT` and `apply_squat` from `talos_dcm_walk_timing.py`
(by import, **zero modification of the S2 files**):

- gait setup identical to S2 (`B.N_STEPS = 70`, `B.STEP_H = 0.04`, frozen W/B gains — an
  acknowledged and commented copy, the same discipline as S3/S4); CLI defaults = the frozen S2
  config.
- simulation loop: detect `c.k >= K_PUSH` → draw the phase → force window
  `d.xfrc_applied[base]` → reset; tick markers (`i_push0`, `i_push1`) to split the `com_err` and
  `xi_err` logs into pre/post regimes.
- summary: eligibility, recovered (T_REC), maximum DCM excursion, band return time, post-push
  `[recov]` steps, standard S2 metrics; enriched npz (push metadata included).
- `s5_batch.py`: the `s2_batch.py` pattern (one subprocess per trial, Wilson aggregation per
  cell, md + csv + npz report, cmd.exe guards), plus per-direction I₅₀ interpolation.

**Bring-up commands:**
```
python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --save s5_run.npz
python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --viewer
python talos_s5_perturb.py --pushmag 100 --pushdir +y --viewer --manual
                                     # push on ENTER/SPACE + arrow key (exploration)
python talos_s5_perturb.py --seed 0 --pushmag 0                      # control arm
python s5_batch.py                                                    # full sweep
```

Exploration note: the viewer shows the push as an **orange arrow** (S1 pattern, displayed for
0.6 s against 0.1 s of physics); `--manual` triggers on demand (ENTER/SPACE, re-triggerable —
the MuJoCo passive viewer does not forward modifier keys, hence no Ctrl+combination). The
metrics and the verdict stay tied to the first push; `--manual` is a qualitative tool, never
used in a campaign.

> **Critical insight.** S5 is the most loosely coupled scenario of the whole campaign — no
> subclass, no `control()`/`update()` override, just a force in the loop. That is a direct
> benefit of the architecture: if S5 fails, it will be attributable neither to a new layer nor to
> copy debt, but to the S2 mechanisms themselves under stress — exactly the question being asked.
> The one debt is the duplication of the gait setup block from `main()` (globals B/W), to be
> re-synchronised if the frozen S2 config changes — flagged at the top of the file.

---

*This document is the S5 design specification. It modifies no validated result and no S1–S4
file. Any implementation must conform to it or flag the departure.*
