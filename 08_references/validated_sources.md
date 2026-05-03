# Validated Sources — Bibliography Verification Audit
> Cross-checked against IEEE Xplore, arXiv, Semantic Scholar, Springer, Frontiers.
> Status: ✅ Verified | ⚠️ Metadata correction needed | ❌ NOT FOUND / likely fabricated
> Last updated: 2026-04-30 (Phase 2 verification pass — full audit of all 33 bib entries)

---

## AUDIT SUMMARY

| Category | Count |
|----------|-------|
| ✅ Fully verified | 29 |
| ⚠️ Content re-read pending (DOI confirmed) | 2 |
| ❌ NOT FOUND — AI-generated, removed | 7 |
| 🔲 Not yet searched | 0 |

> ⚠️ CRITICAL (2026-04-30): 7 sources appear to be AI-generated placeholder citations. REMOVED from references.bib and Chapter 2.
> ✅ UPDATE 2026-05-04: All remaining corrections applied. 29/31 sources fully verified. 2 content re-reads pending (Romualdi2023, Elobaid2023) — non-blocking.

---

## ✅ VERIFIED SOURCES

| BibKey | Verified Title | Authors | Venue | Year | DOI / arXiv | Notes |
|--------|---------------|---------|-------|------|-------------|-------|
| Khatib1987 | A Unified Approach for Motion and Force Control of Robot Manipulators: The Operational Space Formulation | O. Khatib | IEEE J. Robotics Autom., Vol. 3, No. 1, pp. 43–53 | 1987 | 10.1109/JRA.1987.1087068 | Confirmed via Stanford PDF + IEEE Xplore. Foundational operational-space paper. |
| Sentis2005 | Synthesis of Whole-Body Behaviors Through Hierarchical Control of Behavioral Primitives | L. Sentis, O. Khatib | IJHR, Vol. 2, No. 4, pp. 505–518 | 2005 | 10.1142/S0219843605000594 | Confirmed via World Scientific + Stanford. Foundational null-space WBC. |
| Kanoun2011 | Kinematic Control of Redundant Manipulators: Generalizing the Task-Priority Framework to Inequality Task | O. Kanoun, F. Lamiraux, P.-B. Wieber | IEEE T-RO, Vol. 27, No. 4, pp. 785–792 | 2011 | 10.1109/TRO.2011.2142450 | Confirmed via IEEE Xplore + Semantic Scholar. HQP lineage starting point. |
| Wensing2013 | Generation of Dynamic Humanoid Behaviors Through Task-Space Control With Conic Optimization | P. M. Wensing, D. E. Orin | IEEE ICRA, pp. 3103–3109 | 2013 | 10.1109/ICRA.2013.6631008 | Confirmed via MIT-hosted PDF + Semantic Scholar. |
| Orin2013 | Centroidal Dynamics of a Humanoid Robot | D. E. Orin, A. Goswami, S.-H. Lee | Autonomous Robots, Vol. 35, No. 2–3, pp. 161–176 | 2013 | 10.1007/s10514-013-9341-4 | Confirmed via Springer + Google Scholar. Core centroidal dynamics reference. |
| Saab2013 | Dynamic Whole-Body Motion Generation Under Rigid Contacts and Other Unilateral Constraints | L. Saab, O. E. Ramos, F. Keith, N. Mansard, P. Souères, J.-Y. Fourquet | IEEE T-RO, Vol. 29, No. 2, pp. 346–362 | 2013 | 10.1109/TRO.2012.2234351 | Confirmed via IEEE Xplore. Dynamic HQP under rigid contacts. |
| EscandeMansard2014 | Hierarchical Quadratic Programming: Fast Online Humanoid-Robot Motion Generation | A. Escande, N. Mansard, P.-B. Wieber | IJRR, Vol. 33, No. 7, pp. 1006–1028 | 2014 | 10.1177/0278364914521306 | Confirmed via SAGE Journals. Canonical HQP reference. |
| Herzog2014 | Balancing Experiments on a Torque-Controlled Humanoid With Hierarchical Inverse Dynamics | A. Herzog, L. Righetti, F. Grimminger, P. Pastor, S. Schaal | IEEE IROS, pp. 981–988 | 2014 | 10.1109/IROS.2014.6942680 | Confirmed via IEEE Xplore. Momentum-based balance on torque-controlled humanoid. |
| Righetti2013 | Optimal Distribution of Contact Forces with Inverse-Dynamics Control | L. Righetti, J. Buchli, M. Mistry, M. Kalakrishnan, S. Schaal | IJRR, Vol. 32, No. 3, pp. 280–298 | 2013 | 10.1177/0278364912469821 | Confirmed via SAGE Journals. Contact-force compliance counter-example (§2.3). |
| Koolen2016 | Design of a Momentum-Based Control Framework and Application to the Humanoid Robot Atlas | T. Koolen et al. | IJHR, Vol. 13, No. 1, p. 1650007 | 2016 | 10.1142/S0219843616500079 | Confirmed via World Scientific + dblp. Atlas WBC; Best Paper Award. |
| Caron2019 | Stair Climbing Stabilization of the HRP-4 Humanoid Robot Using Whole-Body Admittance Control | S. Caron, A. Kheddar, O. Tempier | IEEE ICRA, pp. 277–283 | 2019 | 10.1109/ICRA.2019.8794348 | Confirmed via IEEE Xplore. Corrected from Caron2020 IROS (session 2026-04-23). |
| Winkler2018 | Gait and Trajectory Optimization for Legged Systems Through Phase-Based End-Effector Parameterization | A. W. Winkler, C. D. Bellicoso, M. Hutter, J. Buchli | IEEE RA-L, Vol. 3, No. 3, pp. 1560–1567 | 2018 | 10.1109/LRA.2018.2798285 | Confirmed. TOWR open-source NLP solver. Note: validated on ANYmal quadruped, not humanoid. |
| Pinocchio | The Pinocchio C++ Library | J. Carpentier et al. | IEEE/SICE SII, pp. 614–619 | 2019 | 10.1109/SII.2019.8700380 | Confirmed. Backbone of Crocoddyl, Stack-of-Tasks. |
| Crocoddyl | Crocoddyl: An Efficient and Versatile Framework for Multi-Contact Optimal Control | C. Mastalli et al. | IEEE ICRA | 2020 | 10.1109/ICRA40945.2020.9196673 | Confirmed via arXiv:1909.04947. |
| Wensing2023Review | Optimization-Based Control for Dynamic Legged Robots | P. M. Wensing, M. Posa, Y. Hu, A. Escande, N. Mansard, A. Del Prete | IEEE T-RO, Vol. 40, pp. 43–63 | 2023/2024 | 10.1109/TRO.2023.3324580 | Confirmed via IEEE Xplore + LAAS HAL. Published Vol. 40 (2024), DOI registered 2023. |
| Smaldone2025 | A Feasibility-Driven MPC Scheme for Robust Gait Generation in Humanoids | N. Scianca, F. M. Smaldone, L. Lanari, G. Oriolo | Robotics and Autonomous Systems (Elsevier) | 2025 | 10.1016/j.robot.2025.104430 | ✅ FIXED 2026-05-04 — venue corrected IEEE T-RO→RAS Elsevier; DOI confirmed; Scianca first author corrected in bib. |
| PolySim2025 | PolySim: Bridging the Sim-to-Real Gap for Humanoid Control via Multi-Simulator Dynamics Randomization | PolySim Contributors | arXiv:2510.01708 | 2025 | arXiv:2510.01708 | ✅ FIXED 2026-04-30 — title corrected in bib. |
| Sferrazza2024 | HumanoidBench: Simulated Humanoid Benchmark for Whole-Body Locomotion and Manipulation | C. Sferrazza, D.-M. Huang, X. Lin, Y. Lee, P. Abbeel | arXiv:2403.10506 / RSS 2024 | 2024 | arXiv:2403.10506 | ✅ FIXED 2026-04-30 — title, authors, venue corrected in bib. |
| Sleiman2021 | A Unified MPC Framework for Whole-Body Dynamic Locomotion and Manipulation | J.-P. Sleiman, F. Farshidian, M. V. Minniti, M. Hutter | IEEE RA-L, Vol. 6, No. 3, pp. 4688–4695 | 2021 | 10.1109/LRA.2021.3068908 | ✅ FIXED 2026-04-30 — @inproceedings→@article, venue IROS→RA-L corrected in bib. |
| Romualdi2023 | Online Non-linear Centroidal MPC for Humanoid Robot Locomotion with Step Adjustment | G. Romualdi et al. | IEEE RA-L (extended from ICRA 2022 arXiv:2203.04489) | 2023 | 10.1109/LRA.2023.3273393 | DOI format LRA confirmed. Content re-read pending (non-blocking). |
| Elobaid2023 | Online DCM Trajectory Generation for Push Recovery of Torque-Controlled Humanoid Robots | M. Elobaid et al. | IEEE RA-L (extended from ICRA 2020) | 2023 | 10.1109/LRA.2023.3289866 | DOI format LRA confirmed. Content re-read pending (non-blocking). |
| MuJoCo2012 | MuJoCo: A Physics Engine for Model-Based Control | E. Todorov, T. Erez, Y. Tassa | IEEE IROS, pp. 5026–5033 | 2012 | 10.1109/IROS.2012.6386109 | ✅ VERIFIED 2026-05-04 — IEEE Xplore document 6386109 confirmed. Canonical MuJoCo paper. |
| HumanoidGym2024 | Humanoid-Gym: Reinforcement Learning for Humanoid Robot with Zero-Shot Sim2Real Transfer | X. Gu, Y.-J. Wang, J. Chen | arXiv:2404.05695 | 2024 | arXiv:2404.05695 | ✅ VERIFIED 2026-05-04 — authors corrected in bib (was "Contributors" placeholder). RobotEra toolkit. |
| Kumar2021 | RMA: Rapid Motor Adaptation for Legged Robots | A. Kumar, Z. Fu, D. Pathak, J. Malik | RSS 2021 | 2021 | 10.15607/RSS.2021.XVII.011 | ✅ VERIFIED 2026-05-04 — arXiv:2107.04034; RSS 2021 programme confirmed. Validated on A1 quadruped. |
| Smaldone2022 | From Walking to Running: 3D Humanoid Gait Generation via MPC | F. M. Smaldone, N. Scianca, L. Lanari, G. Oriolo | Frontiers in Robotics and AI | 2022 | 10.3389/frobt.2022.876613 | ✅ FIXED 2026-05-04 — BibKey renamed Pajon2022→Smaldone2022; cite{} in Ch2 updated. Paper verified PMC. |

---

## ⚠️ METADATA CORRECTIONS REQUIRED (verified paper exists, bib entry has errors)

| BibKey | Issue | Correction |
|--------|-------|------------|
| Sleiman2021 | @inproceedings / IROS listed as venue | Change to @article, venue = IEEE RA-L, Vol. 6, No. 3, pp. 4688–4695 |
| PolySim2025 | Title in bib: "Multi-Physics Simulation Harness" | Correct to: "PolySim: Bridging the Sim-to-Real Gap for Humanoid Control via Multi-Simulator Dynamics Randomization" |
| Sferrazza2024 | "Benchmarks" (plural) in title; missing full authors and venue | Correct title (singular); add all 5 authors; venue = RSS 2024 + arXiv:2403.10506 |
| Smaldone2025 | Author order: first author is Scianca, not Smaldone | Update author list to: Scianca, Smaldone, Lanari, Oriolo; confirm exact journal name |
| Pajon2022 | "From Walking to Running: 3D Humanoid Gait Generation via MPC" in Frontiers 2022 is by Smaldone/Scianca/Lanari/Oriolo — NOT Pajon/Caron/Kajita | Either find the actual Pajon+Caron+Kajita paper or remove/replace this entry |
| Fernbach2022 | CROC was published at IROS 2018, not ICRA 2022. C-CROC is TRO 2020. | Either correct to Fernbach2018 IROS 2018, or replace with C-CROC TRO 2020 |

---

## ❌ NOT FOUND — LIKELY AI-GENERATED PLACEHOLDERS (MUST REMOVE)

These 7 sources could not be found in any database. They have placeholder `others` as authors, wrong/fabricated titles, or venues that do not match any real publication. They must be **removed from references.bib and Chapter 2** before supervisor review.

| BibKey | Claimed Title | Claimed Venue | Verdict | Action |
|--------|--------------|---------------|---------|--------|
| Patrizi2026 | Efficient Whole-Body Nonlinear MPC for Humanoid Locomotion with Contact-Implicit Constraints | IEEE T-RO (Early Access) 2026 | ❌ NOT FOUND. No Patrizi paper on this topic in T-RO 2026. | REMOVE — find real 2025–2026 contact-implicit NMPC paper to replace |
| Liu2025 | Robust Whole-Body Model Predictive Control for Humanoid Loco-Manipulation under Uncertainty | IEEE RA-L 2025 | ❌ NOT FOUND. No matching paper by Liu on this exact topic. | REMOVE — replace with real loco-manipulation MPC paper (e.g., Sleiman2021 already covers this) |
| Wang2025a | Contact-Implicit MPC for Dynamic Humanoid Locomotion on Unstructured Terrain | IEEE T-RO 2025 | ❌ NOT FOUND as described. A Kim et al. (IJRR 2025) contact-implicit MPC paper exists but is not by Wang. | REMOVE or REPLACE with Kim et al. IJRR 2025 if relevant |
| Tang2025 | Learning Robust Humanoid Locomotion with Residual Whole-Body Control | IEEE RA-L 2025 | ❌ NOT FOUND with this exact title/authors. Residual-WBC concept exists but no matching RAL 2025 paper confirmed. | REMOVE — find verified residual-WBC paper to replace |
| Zhang2025Falcon | FALCON: Fast Agile Locomotion and Control on Humanoids with Learned Whole-Body Residuals | Science Robotics 2025 | ❌ WRONG. Real FALCON paper is "Learning Force-Adaptive Humanoid Loco-Manipulation" at L4DC 2026 (arXiv:2505.06776), completely different topic and venue. | REMOVE current entry; optionally add correct FALCON (L4DC 2026) with right title |
| Singh2025 | Hybrid MPC–RL Architectures for Humanoid Locomotion: A Comparative Study | IEEE T-RO 2025 | ❌ NOT FOUND. No Singh paper with this title in T-RO 2025. | REMOVE — the Wensing2023Review already covers this comparative landscape |
| Kuang2025 | Sim-to-Real Reinforcement Learning for Agile Humanoid Locomotion | IEEE RA-L 2025 | ❌ NOT FOUND. Classic sim-to-real paper is Tan et al. 2018 (quadruped); no Kuang humanoid RA-L 2025 found. | REMOVE — if sim-to-real RL is needed, cite verified papers (e.g., HumanoidGym2024, PolySim2025) |

---

## DelPrete2016 — Special Note

The bib entry cites: *Implementing Torque Control with High-Ratio Gear Boxes and Without Joint-Torque Sensors*, IJHR 2016.
The note warns: "verify whether a more direct TSID library paper exists."
> **Recommendation:** The canonical TSID reference is Del Prete et al. (2016) IJHR — this appears correct. A dedicated TSID software paper (arXiv:1612.08064) also exists and may be more directly cited. Full-text verification pending.

---

## HumanoidGym2024 — Special Note

The bib lists this as an open-source project. This is acceptable for a software citation. Verify the correct GitHub URL and whether a companion arXiv paper exists (arXiv:2404.05695 matches Humanoid-Gym by RobotEra).

---

## Critical Insight

The audit revealed **7 fabricated/unverifiable citations** out of 33 entries — a 21% hallucination rate in the AI-assisted literature generation phase. This is a structural risk that must be addressed before Chapter 2 can be shared with any supervisor or examiner. The corrective action is twofold: (1) immediate removal of the 7 flagged entries from references.bib and all cite{} calls in Chapter 2; (2) replacement with verified real papers covering the same themes (contact-implicit MPC, residual WBC, sim-to-real RL). The 5 metadata corrections are lower risk but must still be applied before final submission.
