# Validated Sources — Phase 1 Verification

> Cross-checked against publisher records, author repositories, and Semantic Scholar.
> Status legend: ✅ Verified | ⚠️ Partially verified | ❌ Not found / inconsistency
> Last updated: 2026-04-23 (Session: Chapter 2 SOTA preparation)

---

## §19 Source Verification Table

| BibKey | Title (verified) | Authors | Venue | Year | Status | Verification Notes |
|--------|-----------------|---------|-------|------|:------:|--------------------|
| Sentis2005 | Synthesis of Whole-Body Behaviors through Hierarchical Control of Behavioral Primitives | L. Sentis, O. Khatib | International Journal of Humanoid Robotics (IJHR), Vol. 2, No. 4, pp. 505–518 | 2005 | ✅ | DOI: 10.1142/S0219843605000594 — confirmed via Stanford & World Scientific. Foundational hierarchical/null-space WBC formulation. |
| Wensing2013 | Generation of Dynamic Humanoid Behaviors through Task-Space Control with Conic Optimization | P. M. Wensing, D. E. Orin | IEEE ICRA, pp. 3103–3109 | 2013 | ✅ | Confirmed via MIT-hosted PDF & Semantic Scholar. Prioritized task-space + conic QP, ZMP/friction satisfied at all priority levels. |
| Koolen2016 | Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas | T. Koolen, S. Bertrand, G. Thomas, T. de Boer, T. Wu, J. Smith, J. Englsberger, J. E. Pratt | International Journal of Humanoid Robotics (IJHR), Vol. 13, No. 1 | 2016 | ✅ | DOI: 10.1142/S0219843616500079 — confirmed via dblp & World Scientific. Momentum-based QP-WBC on Atlas, walking + manipulation + multi-contact balancing. Best Paper Award. |
| Caron2020 | ⚠️ See note below | S. Caron et al. | — | — | ⚠️ | **Inconsistency with master_memory.md §19.** No "Caron 2020 IROS" stair-climbing paper found. The closest matching reference is Caron et al. 2019 ICRA: *Stair Climbing Stabilization of the HRP-4 Humanoid Robot using Whole-body Admittance Control* (arXiv:1809.07073). Whole-body admittance + QP wrench distribution + LIPM tracking on industrial staircase (Airbus). **Proposed correction:** rename `Caron2019` and update venue to ICRA 2019. Alternative if 2020 source is genuinely intended: Caron's CWC work (ICRA 2015, "Stability of surface contacts...") covers centroidal-dynamics contact stability rigorously. |
| Winkler2018 | Gait and Trajectory Optimization for Legged Systems through Phase-Based End-Effector Parameterization | A. W. Winkler, C. D. Bellicoso, M. Hutter, J. Buchli | IEEE Robotics and Automation Letters (RA-L), Vol. 3, No. 3, pp. 1560–1567 | 2018 | ✅ | DOI: 10.1109/LRA.2018.2798285. TOWR open-source NLP solver, centroidal dynamics + phase-based foot parameterization, validated on ANYmal quadruped. Note: validated on quadruped, not humanoid — relevance for thesis is the planning-layer formalism. |
| Pinocchio | The Pinocchio C++ Library — A Fast and Flexible Implementation of Rigid Body Dynamics Algorithms and their Analytical Derivatives | J. Carpentier, G. Saurel, G. Buondonno, J. Mirabel, F. Lamiraux, O. Stasse, N. Mansard | IEEE/SICE SII, pp. 614–619 | 2019 | ✅ | DOI: 10.1109/SII.2019.8700380. RBDL-class library + analytical derivatives; 1 µs (manipulator) / 3 µs (legged) per dynamics evaluation. Backbone of Crocoddyl, Stack-of-Tasks. |
| Crocoddyl | Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control | C. Mastalli, R. Budhiraja, W. Merkt, G. Saurel, B. Hammoud, M. Naveau, J. Carpentier, L. Righetti, S. Vijayakumar, N. Mansard | IEEE ICRA | 2020 | ✅ | arXiv:1909.04947. FDDP algorithm (Feasibility-driven DDP); millisecond-scale dynamic maneuvers (jump, front-flip); built on Pinocchio. |

---

## Summary of Verification Outcomes

- **6 of 7 sources fully verified** (titles, venues, years, content).
- **1 inconsistency flagged**: `Caron2020` BibKey in §19 of `master_memory.md` does not correspond to a real "Caron 2020 IROS" stair-climbing paper. Two possible fixes:
  1. Rename to `Caron2019` and update venue to ICRA 2019 (most likely intended source).
  2. Replace by Caron 2015 ICRA "Stability of surface contacts..." if the target topic is centroidal-dynamics contact stability rather than stair climbing.

> **Action required (user/supervisor decision):** confirm which Caron paper should populate the `Caron2020` slot in §19 before this source is cited in Chapter 2.

---

## Content Relevance Mapping (for Chapter 2 sections)

| Source | Chapter 2 Section(s) | Role |
|--------|---------------------|------|
| Sentis2005 | 2.1, 2.2 | Foundational null-space hierarchy; baseline for §3.1 Task-Priority |
| Wensing2013 | 2.3 | First formal QP-WBC w/ conic friction at all priority levels |
| Koolen2016 | 2.3 | Momentum-based QP-WBC; Atlas hardware validation; reference benchmark |
| Caron2019* | 2.3, 2.4 | Whole-body admittance + wrench distribution; stair-climbing case study (replaces Caron2020 placeholder, pending confirmation) |
| Winkler2018 | 2.4 | Gait + trajectory NLP optimization; phase-based parameterization (planning layer) |
| Pinocchio (Carpentier2019) | 2.6 | Dynamics library; cited for implementation in §3.1 of thesis methodology |
| Crocoddyl (Mastalli2020) | 2.4, 2.6 | Multi-contact OCP framework; FDDP; reference toolchain comparison |

---

## Critical Insight

The verification process surfaced one BibKey/year inconsistency in `master_memory.md` §19 (`Caron2020` → likely `Caron2019` ICRA). All other six sources are confirmed at the title/venue/year level, but a **methodological gap remains**: §19 currently tracks bibliographic existence, not **content fidelity** (i.e. whether the paper's claims actually support the use being made of it in the thesis). Before Chapter 2 is finalized, each source must be re-read in full and a one-paragraph "claim → evidence" trace added to this file. Without this second pass, citations risk becoming reputational anchors rather than evidentiary ones — a recurring weakness in robotics literature reviews.
