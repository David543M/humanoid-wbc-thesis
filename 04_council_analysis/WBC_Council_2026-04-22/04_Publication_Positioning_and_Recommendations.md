# Publication Positioning and Final Recommendations

> *Council verdict.* This file operationalizes the synthesized review into an actionable plan. It defines what the thesis must *contain*, *remove*, and *deepen* to become publishable at ICRA, RSS, or IJRR, and provides an explicit prioritized edit list.

---

## 1. Publication Positioning by Venue

Each venue has a different acceptance criterion. The following table states what the *same* underlying thesis work would need to look like at each venue, what counts as novelty, and what the realistic submission target is.

### 1.1 ICRA (IEEE International Conference on Robotics and Automation)

| Dimension | ICRA expectation |
|---|---|
| **Acceptance criterion** | Working system with rigorous comparison against at least one state-of-the-art baseline, demonstrated on a humanoid in simulation or hardware. |
| **Novelty bar** | A *new evaluation protocol*, a *new controller configuration*, or a *new sensitivity analysis* suffices if documented and reproduced. |
| **Typical format** | 6–8 page paper + 1–2 page appendix; video attachment; one or two key results figures. |
| **Thesis mapping** | Submit the evaluation-protocol + contact-sensitivity analysis as a stand-alone ICRA paper. |
| **Required deliverables** | (a) Published URDF / MJCF; (b) open-sourced scenario code; (c) quantitative comparison of ≥ 2 WBC formulations on ≥ 3 scenarios; (d) at least one sensitivity axis (contact stiffness, friction coefficient, or integrator step). |
| **Realism of target** | Achievable at master's scale if the thesis commits to the evaluation-protocol framing by month 3. |

**ICRA paper title (proposed).**
> *A Reproducible Simulation-Phase Benchmark for QP-based Whole-Body Control of Humanoid Robots: Contact-Model Sensitivity and Loco-Manipulation Scenarios.*

### 1.2 RSS (Robotics: Science and Systems)

| Dimension | RSS expectation |
|---|---|
| **Acceptance criterion** | A principled scientific contribution — formal analysis, novel methodology, or rigorous empirical study that reshapes how practitioners think. Smaller venue, stricter review. |
| **Novelty bar** | Must present a *conceptual* or *methodological* insight, not merely a working system. |
| **Typical format** | 8 pages; one or two headline results with deep analysis. |
| **Thesis mapping** | Reposition the contact-model–controller consistency analysis as a *methodological contribution*: introduce a formal framework for *controller-feasibility transfer across contact models* and demonstrate it on multiple simulators (MuJoCo, Drake) and multiple WBC formulations. |
| **Required deliverables** | (a) Formal statement of the consistency-gap problem (§5.2 of synthesized review); (b) sensitivity results that *generalize* across ≥ 2 simulators and ≥ 2 WBC formulations; (c) a predictive metric for sim-to-real degradation derived from the sensitivity analysis. |
| **Realism of target** | Ambitious for a single master's thesis; feasible only if the thesis commits to this framing from the start and the supervisor is aligned on methodology-first framing. |

**RSS paper title (proposed).**
> *The Contact-Model–Controller Consistency Gap in Whole-Body Control: A Formal Framework and Cross-Simulator Sensitivity Study.*

### 1.3 IJRR (International Journal of Robotics Research)

| Dimension | IJRR expectation |
|---|---|
| **Acceptance criterion** | Archival-quality journal contribution, typically extending a conference paper with deeper analysis, full experimental protocols, and exhaustive related work. |
| **Novelty bar** | A *definitive* treatment of a question, not an incremental contribution. Reviewers expect consolidated positioning against the entire field. |
| **Typical format** | 20–30 pages; extensive reference list (80–150 citations); deep ablation. |
| **Thesis mapping** | Combine the RSS methodological framework with the ICRA empirical protocol into a single IJRR submission: a *reproducible benchmarking methodology for simulation-phase WBC*, with a cross-simulator cross-formulation cross-scenario study and an open-source release. |
| **Required deliverables** | (a) All RSS + ICRA deliverables; (b) hardware validation or explicit rigorous sim-to-hardware correlation study *on at least one scenario* — typically the thesis's scope-limit; (c) rebuttals of competing explanations for observed sensitivities; (d) 100+ reference bibliography. |
| **Realism of target** | Typically a post-master's extension, executed with the supervisor after thesis defense — a 6-month publication effort. Not in the master's scope directly. |

**IJRR paper title (proposed — long-horizon).**
> *Towards Reproducible Whole-Body Control Evaluation: A Cross-Simulator, Cross-Formulation Benchmarking Methodology for Torque-Controlled Humanoid Robots.*

### Mini Critical Insight — Positioning
The *same underlying thesis work* publishes at all three venues if framed correctly: ICRA rewards the system-and-protocol; RSS rewards the conceptual framing of the consistency gap; IJRR rewards the consolidated cross-cutting study. The thesis should be *structured from day one* with this three-venue trajectory in mind — the master_memory.md chapter plan already permits this, but the literature review must be written to support the RSS-style framing, which is what §5 of `03_Synthesized_Literature_Review.md` delivers.

---

## 2. Strengths That Must Be Kept and Deepened

Drawing from all three source reports, these are the elements the synthesized thesis must preserve and extend:

1. **Report A's taxonomy and reading order.** Keep A's §10 taxonomy (kinematics vs. dynamics, priority handling, optimization back-end, model-based vs. learning, actuation interface). Extend it to seven axes per §4 of the synthesized review.
2. **Report A's gap–thesis-angle mapping.** A's §11 is the best backbone for master_memory.md §7; keep the one-to-one mapping structure and extend it with confidence scores.
3. **Report B's simulator comparison table.** Keep the column structure (contact model, integrator, strengths, limitations); rebuild the cells on primary sources per §5.1 of the synthesized review.
4. **Report B's MPC-vs-WBC layer table.** Useful pedagogically; keep in §3.4 or §4 of the thesis.
5. **Report B's solver taxonomy.** qpOASES / OSQP / HPIPM / ProxQP mapping to problem regime is correct; deepen with a pragmatic test protocol.
6. **Report C's 2024–2026 citation spine.** Dantec, Sleiman, Radosavovic, Haarnoja, HOVER, HumanoidBench, MuJoCo Playground — all verified and should be the core modern-SOTA section of the thesis.
7. **Report C's emerging-consensus architecture.** The privileged-teacher / distilled-student / low-level-PD pattern is the single most important synthesis across the three reports; keep it verbatim in Chapter 2.
8. **Report C's argumentation style.** Explicit positions, explicit limitations, explicit predictions. This tone is what an IJRR reviewer expects. A thesis that reads neutrally surveys the field; a thesis that takes positions *on* the field is the one that gets cited.

---

## 3. What Must Be Removed

These are elements of the three source reports that, if carried into the thesis, would damage its publishability:

1. **Report B's non-peer-reviewed citations.** PatSnap, RoboCloud Hub, Moonlight (themoonlight.io), MANUS marketing copy, SciSpace paraphrases, ResearchGate figure fetches. *All must be replaced with primary sources.*
2. **Report B's broken LaTeX.** Any `\\in`, `\\mathbb`, `\\dot` double-escapes must be rewritten in valid LaTeX.
3. **Report B's OSC-for-floating-base formula without support-consistent Jacobian.** *Technically incorrect* as stated; replace with Mistry 2010 formulation.
4. **Report B's RaiSim speed claim** ("5–7× faster than MuJoCo") *without the conflict-of-interest disclosure* (SimBenchmark is a Hutter-lab artefact). If kept at all, must be disclosed and contextualized.
5. **Report A's conflation of TSID and HQP.** TSID is a weighted single-QP; HQP is a lexicographic cascade. The thesis must state this distinction in §3.5 / §4.3.
6. **Report A's unrendered numbered citations.** Every `[^N]` marker must resolve to a verifiable archival citation; the thesis cannot carry footnotes whose targets are undocumented.
7. **Report A's "development of a GPU-accelerated WBC solver in 12 months" suggestion.** Not feasible at master's scale; omit from thesis opportunities.
8. **Report C's implicit positions without confidence markers.** Replace "the central research object is… the training pipeline" with an explicitly hedged claim ("we argue that…", "evidence suggests that…").
9. **All three reports' VLA / BFM / foundation-model hype.** LeVERB / GR00T / BumbleBee should be mentioned as emerging developments, *not* treated as established SOTA until peer-reviewed demonstrations consolidate.
10. **Any claim not supported by ≥ 2 primary sources per master_memory.md §5 "source integrity rule".** The thesis must enforce this.

---

## 4. What Must Be Deepened

These are the elements none of the three reports treats adequately and that the thesis must strengthen:

### 4.1 Mathematical rigor (current state: below publication threshold)

- **Required additions in Chapter 3.**
  - Explicit floating-base EoM with support-consistent Jacobian (Mistry 2010).
  - Explicit CMM identity $h_G = A_G(q)\dot q$ (Orin-Goswami 2013).
  - Explicit friction-cone statement and polyhedral linearization with facet-count discussion.
  - Explicit distinction between weighted single-QP (TSID) and lexicographic HQP (Escande 2014).
  - Mistry's underactuation-decoupling impossibility result as a named limitation.
  - The passivity-based multi-contact formulation of Henze-Roa-Ott [IJRR 2016] as a named alternative.
  - MPC cost structure with centroidal or SRBD reduction.

### 4.2 Contact-model–controller consistency (current state: absent)

This is the thesis's *publishable angle*. Deepening required:

- Formal statement of the gap: a controller optimizing a QP with polyhedral friction cones produces torques whose resulting contact forces, simulated in MuJoCo's compliant contact or Drake's relaxed complementarity, need not satisfy the controller's own feasibility constraints.
- A **sensitivity protocol** across contact stiffness, friction coefficient, and integrator step size. Measure (i) fraction of time the true contact force violates the controller-assumed friction cone, (ii) CoM tracking error, (iii) task success rate.
- Cross-simulator validation: MuJoCo primary, Drake secondary verification on one scenario.

### 4.3 Real solver-performance characterization (current state: surface-level)

- Measure QP solve time distributions for OSQP vs. eiquadprog vs. qpOASES on the same TALOS URDF across scenarios S1–S5 (master_memory.md §9.4).
- Document warm-start failure rates under contact-switching events.
- Report worst-case solve time (not mean) — real-time feasibility depends on the tail.

### 4.4 Perturbation-robustness protocol (current state: non-reproducible in the literature)

- Define a standardized *impulse distribution* (direction × magnitude × timing) per scenario.
- Define a *recovery criterion* (time-to-CoM-within-tolerance, no contact-feasibility violation).
- Report recovery rate over ≥ 100 randomized trials.
- This protocol alone is publishable as a benchmarking contribution.

### 4.5 Learning-based baseline selection (current state: weakly framed in all three reports)

- Per master_memory.md §3.5, the thesis uses RL only as a *baseline*. The baseline choice must be justified:
  - **Recommended:** a published pre-trained policy on the same platform (Unitree H1 RL, per HumanPlus / HOVER open-source releases) *or* a ExBody-style imitation policy.
  - Not recommended: training a custom RL policy — scope inflation.
- Report comparison on identical scenarios S1–S5, identical metrics — the existing RL-vs-WBC comparisons in the literature use different metrics and are unreliable.

---

## 5. Prioritized Edit List (Actionable)

Tagged with the target thesis chapter from master_memory.md §12 and priority level. These are the *specific, actionable* edits the student must execute.

### Priority 🔴 P1 (blocking)

| # | Edit | Target chapter |
|---|---|---|
| E1 | Replace all Stack-of-Tasks / Report A foundation citations with the archival references in `§10` of `03_Synthesized_Literature_Review.md` | Ch. 2 |
| E2 | Write §3.1 (EoM with support-consistent Jacobian) and §3.5 (HQP vs. weighted QP) formally, with equations | Ch. 3 |
| E3 | Populate master_memory.md §19 (Source Integrity Tracking) with ≥ 10 verified references from `§10` of the synthesized review | — |
| E4 | Commit to a simulator (PQ2 in master_memory.md §16). Recommendation: **MuJoCo primary + Drake second-track** | Ch. 4 |
| E5 | Commit to a robot model (PQ1). Recommendation: **TALOS** (mature WBC stack, open URDF, aligns with Ramuzat 2022) or **Unitree H1** (more recent, RL baselines available) | Ch. 3, 4 |
| E6 | Commit to primary WBC formulation (PQ3). Recommendation: **weighted single-QP (TSID-style) with centroidal MPC planning layer**, with lexicographic HQP as a comparison baseline | Ch. 3 |
| E7 | Formalize the contribution claim: *methodological evaluation-protocol and contact-model sensitivity analysis*, not algorithmic novelty (PQ4) | Ch. 1 |

### Priority 🟠 P2 (blocking for strong thesis)

| # | Edit | Target chapter |
|---|---|---|
| E8 | Define perturbation-robustness protocol formally (direction × magnitude × timing distribution; recovery criterion) | Ch. 5 |
| E9 | Add contact-model sensitivity protocol to Ch. 4 | Ch. 4 |
| E10 | Remove all Report B non-peer-reviewed citations from any draft that inherits from it | Ch. 2 |
| E11 | Extend Ramuzat et al.'s TALOS protocol with scenarios S4 (loco-manipulation) and S5 (perturbation) from master_memory.md §9.4 | Ch. 5 |
| E12 | Ground the master_memory.md §9.2 metric thresholds in published numbers from Ramuzat 2022, Dantec 2022, Koolen 2016 | Ch. 5 |

### Priority 🟡 P3 (improves publishability)

| # | Edit | Target chapter |
|---|---|---|
| E13 | Add §2.6 "Motion Retargeting as the Universal Interface" from synthesized review to cover 2024–2025 SOTA | Ch. 2 |
| E14 | Add §7 (Research Gaps, structured by methodological / evaluation / data / theoretical) | Ch. 2 |
| E15 | Add confidence markers ([H] / [M] / [L]) to all non-trivial claims in Chapters 1–2 | Ch. 1, 2 |
| E16 | Draft an ICRA-format version of the sensitivity study as a side deliverable to be submitted with the supervisor after thesis completion | — |
| E17 | Add explicit anticipated-criticism rebuttals (master_memory.md §18) to Ch. 6 Discussion | Ch. 6 |
| E18 | Include energy-efficiency metric from Ramuzat 2022 in the metric table (master_memory.md §9.2) | Ch. 5 |

### Priority 🟢 P4 (optional, high value if done)

| # | Edit | Target chapter |
|---|---|---|
| E19 | Release URDF, scenario code, and solver-parameter configurations as a public GitHub repo — aligns with master_memory.md §20 and supports reproducibility claim | Appendix |
| E20 | Run one scenario on Drake in addition to MuJoCo to document cross-simulator results | Ch. 4 |
| E21 | Consolidate thesis insights into an RSS-format methodological paper (post-defense) | — |

---

## 6. Final Council Recommendations (Harsh)

1. **Do not treat the three deep-research outputs as drafts of a literature review.** They are raw materials of uneven quality. Use the synthesized review in this folder as the actual Chapter 2 scaffold.
2. **Do not cite PatSnap, RoboCloud, or themoonlight.io.** Delete these references immediately wherever they have been copied.
3. **Do not claim algorithmic novelty in WBC formulation.** The design space is saturated; the council's consensus is that the defensible novelty is *evaluation methodology*.
4. **Do not promise hardware deployment.** The thesis is simulation-phase (master_memory.md §1.5) — state this as a scope *limit* and defend it, do not apologize for it.
5. **Commit to master_memory.md §16 pending questions within the next 2 weeks.** PQ1, PQ2, PQ3, PQ4 are all blocking. The council's recommended defaults: PQ1 → TALOS, PQ2 → MuJoCo primary + Drake second-track, PQ3 → weighted QP with centroidal MPC planner, PQ4 → reproducible evaluation protocol + contact-sensitivity analysis.
6. **Write the math.** Chapters 2–3 cannot be written without writing the equations. No deep-research output substitutes for this.
7. **Adopt Ramuzat et al. (Frontiers 2022) as the *primary experimental reference* the thesis extends.** This single decision resolves scenario selection, metrics, baselines, and simulator-protocol disputes simultaneously.
8. **Commit to a three-venue publication plan on day one:** ICRA submission of the protocol and sensitivity study (month 9–10); RSS or IJRR consolidation with supervisor (post-defense, 6 months later). Do not defer publication planning to the end.
9. **Preserve scholarly honesty.** Every claim about 2024–2026 SOTA must be verifiable at primary venue level; no surveys or blog-level citations should support a factual claim.
10. **Re-read `master_memory.md` before each chapter draft.** Per the project instructions, it is the source of truth; the council's synthesis is meant to *feed it*, not replace it.

---

## Mini Critical Insight — Closing
The council's strongest recommendation is the one most likely to be resisted: **the thesis's *contribution* is not the controller, it is the evaluation protocol and the sensitivity analysis**. Accepting this reframing is the single decision that unifies feasibility, publishability, and coherence with the logical chain in master_memory.md §13. Every other recommendation in this file follows from that reframing. If the supervisor disputes it, the thesis can fall back to the narrower framing of "a working QP-WBC implementation validated on standard scenarios" — competent master's work, but not publishable beyond a workshop venue. The framing advocated here is the *only* one that routes the same work to ICRA, RSS, and eventually IJRR.
