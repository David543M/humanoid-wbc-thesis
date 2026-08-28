# Conditioning of the WBC QP — soft contact vs hard equality

## Problem
The contact no-slip task was a **soft cost** with a very large weight
(`W_CONTACT = 1e4`). Added to the Hessian as `1e4·Jcᵀ Jc`, alongside the other tasks
(weights 1–100), it spreads the spectrum of `G` enormously → **poor conditioning**,
measured at `κ(G) ≈ 2.1×10¹⁰`. The solver (quadprog) coped (0 fallbacks) but the QP
was numerically fragile.

The soft cost was originally chosen because of **rank deficiency**: imposing the
24 contact-corner constraints as hard equalities (4 corners × 3 DoF × 2 feet) is
redundant — a rigid flat foot has only 6 DoF, so 12 of the 24 rows are dependent, and
quadprog rejects an equality that is not full rank.

## Correction tested (opt-in: `wbc.hard_contact = True`)
No-slip is expressed as a **hard equality on the 6D foot-body pose** (3 translation +
3 rotation per foot via `mj_jac`), giving **12 independent rows** in double support —
full rank, no redundancy. The soft task `1e4·Jcᵀ Jc` is removed from the cost; the
contact forces `f` remain (friction pyramid unchanged).

## Result (standing, otherwise identical)

| Formulation | median κ(G) | QP fallbacks | upright | CoM err | push 150/400 N |
|---|---:|---:|---|---:|---|
| soft cost (default) | **2.1×10¹⁰** | 0 | UPRIGHT | 0.01 mm | OK / OK |
| **hard equality** (`hard_contact`) | **1.0×10⁷** | 0 | UPRIGHT | 0.01 mm | OK / OK |

**Conditioning improved ×2000** (2.1e10 → 1.0e7), with no loss of performance or
robustness, and still 0 fallbacks (the 6D formulation avoids the rank deficiency).

## Limitation / extension
`hard_contact` freezes **every** foot in `self.feet`: valid in **double support**
(standing). For **walking** in single support, the constraint should be imposed only on
the foot or feet actually in contact (the stance foot), not the swing foot — since the
`DCMWalk` walker has its own `control()`, that extension has to be made there separately.

## Recommendation
Keep `hard_contact` as the recommended option for balancing and push recovery (better
conditioning, cleaner formulation). To be documented in the Methodology as the reference
formulation, the soft cost being the historical one. Enable with:
`wbc = WBC(m, d); wbc.hard_contact = True`.

---

## Update — hard formulation adopted as the default (balancing and walking)

`hard_contact` (balancing) and `use_hard_contact` (walker, on the stance foot alone) are
now the **defaults**. Measured results:

| Scenario | κ(G) before (soft) | κ(G) after (hard) | performance |
|---|---:|---:|---|
| Standing balance | 2.1×10¹⁰ | **1.0×10⁷** | UPRIGHT, push 150/400 N OK, CoM 0.01 mm |
| Walking (`--walk-cl`) | 2.08×10¹⁰ | **2.07×10⁷** | **4 clean steps** (identical), 0 fallbacks |

A conditioning gain of roughly ×1000–2000 with no change in performance. In single
support, only the **stance foot** constraint is imposed (the swing foot stays free),
through the walker's `active` list.

**Reproducibility note**: changing the default contact formulation shifts the open-loop
baseline *trajectory* slightly *after the fall* (the fallen robot slides differently;
for example x≈1.5 m instead of 0.82 m at 8 s). The **meaningful** metrics are preserved:
it still falls (open-loop), 0 QP fallbacks, balance and push recovery unchanged, and
4 clean steps in walk-cl. To return to the historical behaviour:
`wbc.hard_contact=False` / `c.use_hard_contact=False`.
