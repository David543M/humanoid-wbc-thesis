# Audit — S2 timing law vs the official Khadiv2020 implementation
*2026-07-15 | Reference: machines-in-motion/reactive_planners (`src/dcm_vrp_planner.cpp`, BSD-3, NYU/MPI — Righetti/Khadiv group) vs `talos_dcm_walk_timing.py` (DCMWalkT)*

## 1. Reference formulation (DcmVrpPlanner)

QP solved at **every control cycle**, 9 variables: `u_x, u_y` (location of the next
step, relative to the stance foot), `τ = e^{ωT}` (exponentiated duration → linear
constraints), `b_x, b_y` (DCM offset at the next touchdown), 4 slacks.

- **Cost**: quadratic tracking of nominal values (`l_nom, w_nom, τ_nom,
  bx_nom, by_nom`) derived from the desired velocity `v_des`.
- **Equalities (2)**: DCM recursion — `u + b = (ξ_mes − p_appui)·e^{−ωt}·τ`
  (consistency of location/duration/offset, x and y channels **coupled through τ**).
- **Inequalities (10)**: box bounds on step length/width (asymmetric according to
  the stance side), `τ ∈ [τ_min, τ_max]` with `τ_min` raised dynamically to
  `e^{ω·max(t_min, t_écoulé + t_min_restant)}`, **viability** bounds on
  `b` (`bx = l/(τ−1)`, asymmetric `by_in/by_out` — the stance/swing side
  asymmetry is structural).
- **Fallback**: infeasible QP → nominal values.

## 2. Our implementation (DCMWalkT, lines 296–317 + 356–380)

**Analytic, no QP.** Timing and placement are decoupled:

- **Timing**: closed-form divergence law `d(τ) = d0·e^{ωτ}` →
  `t_rem = ln(d*/d)/ω` per channel; lateral: `d* = ly + ξ_s`; sagittal:
  ceiling `SAG_MAX`; `T_k = clip(τ + min(t_lat_rem, t_sag_rem), T_MIN, T_MAX)`.
  `d ≤ 0` (DCM on the wrong side) → immediate touchdown (`_force_land`).
- **Placement**: analytic capture point `g_y = ξ_prédit(t_go) + sgn·off_lat`
  with geometric bounds `[min_sep, MAX_SEP]` relative to the actual stance foot;
  **sagittal: foothold frozen from the plan** (no `u_x` adjustment), recovery
  mode = pure lateral step, sagittal progression frozen.
- **Per-step replanning**: `d0` measured at every step entry.

## 3. Correspondence

| Element | Reference | Ours | Verdict |
|---|---|---|---|
| DCM divergence law (LIP, e^{ωt}) | yes (via τ) | yes (closed form) | ✅ identical |
| Timing shortened when the DCM escapes | dynamic τ_min | T_k = τ + t_rem, floor T_MIN | ✅ equivalent |
| Timing lengthened when the DCM is well behaved | up to τ_max | up to T_MAX | ✅ equivalent |
| Lateral switching value | by_nom (l_p, v_des) | d* = ly + ξ_s | ✅ same role, simplified parametrisation |
| Stance-side asymmetry | by_in/by_out bounds | anti-crossing min_sep (geometric) | ⚠️ partial |
| Sagittal step-location adjustment u_x | yes (joint QP) | **no** (frozen plan + pure lateral recovery) | ❌ absent |
| Location↔duration coupling (τ in the equality) | yes | no (decoupled) | ❌ absent |
| Viability bounds on b | yes | no (geometric bounds) | ❌ absent |
| Joint multi-objective optimisation | 9-var QP + slacks | greedy min(2 channels) | ❌ simplification |
| Explicit recovery on degenerate entry | no (nominal fallback) | yes (pure lateral [recov]) | ➕ our own addition |

## 4. Consequences for the thesis

1. **The wording "in the lineage of Khadiv et al." (Ch5 §5.5) is
   correct and must stay** — never write "implements Khadiv2020": we
   implement the *temporal divergence law* of that lineage, not the
   joint location+duration QP.
2. **The audit CONFIRMS the architectural reading of §5.8**: the absence of
   sagittal adjustment on our side (`u_x` frozen, pure lateral recovery) — where
   the reference optimises it — is exactly the missing mechanism that the
   S5 campaign measures (sagittal I₅₀ 20–30 N·s vs lateral 45–50+). The
   "sagittal analogue of the lateral timing law" prescription of the
   criticalinsight §5.8 amounts, in the reference's terms, to reintroducing
   `u_x` and the coupling through τ.
3. **Validity of the `--no-timing` ablation**: `use_timing=False` freezes
   only the T_k update (T_k = T_nom); `use_cp_swing` (capture-point
   placement) stays active → the ablation cleanly isolates the contribution
   of *timing* alone, separated from *placement* — cleaner than in the
   reference, where the two sit in the same QP and are not separable.
4. **Quantifiable future-work candidate**: replace the 2-channel greedy rule with
   the reference's 9-variable QP (quadprog is already in the pipeline) = a
   bounded-effort extension, directly comparable.

## 5. Limitations of the audit

Comparison made against `dcm_vrp_planner.cpp` (master, accessed 2026-07-15);
the `DcmReactiveStepper` wrapper (phase / end-effector management) was not
audited — out of scope: our stepping machine differs by design.
The reference runs on Bolt/Solo (not TALOS); no direct numerical
comparison is therefore possible, only a structural one.
