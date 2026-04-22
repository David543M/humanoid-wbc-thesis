# Research Gaps — Consolidated
*Source: WBC Council Synthesis 2026-04-22 (§2.9 of 03_Synthesized_Literature_Review.md)*
*Confidence markers: [H] multi-source peer-reviewed | [M] single peer-reviewed / preprints | [L] contested*

---

## Methodological Gaps

| ID | Gap | Evidence | Relevance to thesis | In scope? | Conf. |
|----|-----|----------|---------------------|-----------|-------|
| MG1 | Coupling centroidal MPC (50–100 Hz) with full-body QP-WBC (1 kHz) introduces interface errors — no standardised method | Dantec 2022; Kuindersma 2016 | Directly concerns the thesis's planning-execution architecture (Ch. 3) | Yes — document the interface protocol | [M] |
| MG2 | Contact-mode transition handling in QP-WBC is hand-tuned per platform; no generalised strategy exists | Koolen 2016; Ramuzat 2022 | Affects scenarios S1–S3 — must be addressed in Ch. 3 | Partially — hand-tuned approach must be justified | [M] |
| MG3 | Choice between weighted single-QP and lexicographic HQP under infeasibility lacks a principled selection criterion | Escande 2014 §6; Bouyarmane–Kheddar | Resolving PQ3 (formulation choice) | Yes — Ch. 3 must justify the choice | [M] |
| MG4 | Flexibility-aware WBC (SEA, link flexibility) is not generalised | Romualdi et al. Humanoids 2022 | Low relevance — ideal torque sources assumed in sim | No — out of scope | [M] |

## Evaluation Gaps

| ID | Gap | Evidence | Relevance to thesis | In scope? | Conf. |
|----|-----|----------|---------------------|-----------|-------|
| EG1 | No standardised scenario-and-metric set across humanoid WBC platforms | Ramuzat 2022 is best-known partial remedy | Directly motivates the thesis's evaluation protocol as a contribution | Yes — extends Ramuzat 2022 | [H] |
| EG2 | Perturbation-robustness protocols are non-reproducible paper-to-paper | master_memory.md §7.3 | Scenario S5 (perturbation robustness) — thesis can propose a reproducible protocol | Yes — S5 design | [H] |
| EG3 | Simultaneous loco-manipulation benchmarks are rare | Sleiman 2023; HumanoidBench 2024 partial remedies | Directly maps to scenario S4 (loco-manipulation) | Yes — S4 design | [M] |
| EG4 | Quantitative sim-to-real degradation is rarely reported for model-based WBC | Systematically reported only on RL side | Acknowledged limitation in Ch. 6 Discussion | No — sim-only thesis; flag as limitation | [M] |

## Data & Infrastructure Gaps

| ID | Gap | Evidence | Relevance to thesis | In scope? | Conf. |
|----|-----|----------|---------------------|-----------|-------|
| DG1 | No public multi-contact force dataset for humanoid loco-manipulation | Survey remarks only; no existing remedy | Prevents empirical grounding of contact force bounds | No — no hardware experiments | [L] |
| DG2 | URDF / MJCF / SDF cross-simulator portability unsolved; cross-sim comparisons require hand-tuning | Common practitioner knowledge | Affects MuJoCo ↔ Drake cross-validation plan | Partially — document conversion effort | [M] |

## Theoretical / Guarantee Gaps

| ID | Gap | Evidence | Relevance to thesis | In scope? | Conf. |
|----|-----|----------|---------------------|-----------|-------|
| TG1 | Lyapunov / passivity guarantees for mainstream QP-WBC do not exist beyond Henze–Roa–Ott 2016 | Henze–Roa–Ott IJRR 2016 | WBC stability analysis is out of scope; flag as a known theoretical limitation | No — acknowledge in Ch. 6 | [H] |
| TG2 | Safety guarantees for learned whole-body policies are empirical only; CBF integration emerging | Ames et al. CBF literature | Not relevant — RL is baseline only | No | [M] |

---

## Gaps Directly Addressed by This Thesis

| Gap ID | How addressed |
|--------|---------------|
| EG1 | Extended evaluation protocol (S1–S5) building on Ramuzat 2022 |
| EG2 | Reproducible perturbation-robustness protocol (S5) with standardised impulse distribution and recovery criterion |
| EG3 | Loco-manipulation scenario (S4) with dual metric tracking (CoM error + end-effector error) |
| MG1 | Explicit documentation of the centroidal MPC ↔ QP-WBC timing interface in Ch. 3 and Ch. 4 |

---

*Last updated: 2026-04-22 — migrated from 04_council_analysis/WBC_Council_2026-04-22/03_Synthesized_Literature_Review.md §2.9*
