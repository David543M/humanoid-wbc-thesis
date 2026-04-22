# Council Deliberation Transcript — RERUN
## WBC Deep-Research Triangulation for Thesis Chapter 2 — 2026-04-22

> *Orchestration disclosure.* Fallback mode: four personas reason sequentially but blindly in Round 1, cross-examine in Round 2, and restate final positions in Round 3. Framing objective changed from the first run: the deliverable is **Chapter 2 of the thesis**, not a publishable article.

---

## ROUND 1 — Independent Analyses (Blind, Lit-Review Lens)

### Σ — Control Theory Expert (mathematical rigor for a thesis chapter)

**Re-framing note.** For a literature-review *chapter*, my job is not to demand T-RO-level derivations. It is to check whether the student, after reading the chapter, will (i) know the standard notation, (ii) know the canonical equations and their provenance, and (iii) understand why the field made each mathematical choice. Thesis-chapter mathematical rigor is *pedagogical* — it should *teach*.

**Report A.** The mathematical foundations section (A §3) lists the correct ingredients — EoM, floating-base coordinates, task Jacobians, contact and friction cone, inverse-dynamics QP, null-space and HQP, centroidal dynamics, capture point — but teaches none of them. Every paragraph is a summary, not a derivation. As a *map of what the student needs to learn*, A's §3 is excellent; as a *chapter that stands alone*, it is insufficient. For the thesis, this is acceptable: A is the *outline* from which the student will expand. Also important: A §10 (Taxonomy) is precisely the kind of multi-axis breakdown a thesis chapter needs to cite and reuse.

**Report B.** This is the one report that writes the equations explicitly — EoM, centroidal momentum linear/angular form, friction cone, OSC closed form. For a student writing Chapter 3 (Methodology), these formulas are useful *drafts*. But the LaTeX is broken and the floating-base OSC extension omits the support-consistent Jacobian of Mistry (2010), which means the student who copies B's formulas into the thesis will carry a technical inaccuracy into Chapter 3. The thesis would have to *correct* B, not just adopt it.

**Report C.** Writes no equations. For a literature-review chapter, this is a *structural deficit* — a student cannot build §3 (Mathematical Foundations) of Chapter 2 from C alone. C's value is its *citation spine*: specific venues and papers (Dantec ICRA 2021 / Humanoids 2022 / RA-L 2022, Khazoom Humanoids 2024, Sleiman RA-L 2021 / Sci. Robot. 2023) that are missing from A. For mathematical content, C is a bibliography, not a source.

**Σ verdict (R1) for lit-review utility.** A provides the best *structure* to populate, B provides reusable (but needing correction) *formulas*, C provides the *citations* to anchor claims. **Rank: A ≻ B ≻ C** on the mathematical-scaffold axis — which inverts C vs. B compared to the publication-oriented run, because "readable equations the student can adapt" beats "correct citations with no equations" when the goal is a lit review that teaches. **Confidence:** high.
**Where I may be wrong:** If the thesis supervisor penalises technical inaccuracies more than missing detail, B's floating-base OSC misstatement is a larger problem than I have scored it.

---

### Ω — Robotics Systems Engineer (implementation realism for the specific thesis)

**Re-framing note.** For a lit-review chapter, "implementation realism" means: does the chapter give the student the vocabulary and tool-landscape they need to defend their implementation choices in Chapter 3? master_memory.md §2 targets a humanoid (TBD: TALOS / Unitree H1 / custom), §8.2 selects Pinocchio + OSQP + (MuJoCo or Isaac Sim), and §16 lists simulator / robot / WBC-formulation as blocking pending questions. The chapter must equip the student to close all of these.

**Report A.** Names every software library a thesis committee will expect to see — Stack-of-Tasks, OpenSoT, TSID, SEIKO, Pinocchio is absent but Crocoddyl-adjacent work is implied. The software-ecology treatment (A §8.4) is the *single most thesis-actionable* section in any of the three reports because it directly feeds the thesis's tool-selection discussion. The platform enumeration (A §6) — TALOS, HRP-2/4/5P, COMAN/COMAN+, iCub — is exactly the platform vocabulary the student needs. The gap: no explicit QP-solver comparison (qpOASES vs. OSQP vs. ProxQP) and no concrete treatment of simulation-side tools beyond Gazebo and MuJoCo.

**Report B.** The solver taxonomy (qpOASES / OSQP / HPIPM / ProxQP) is the best of the three on the solver axis, and the simulator-comparison table (RaiSim / MuJoCo / Isaac Sim / Drake) is the single most useful decision-artefact across the three reports *even with* its sourcing problems. For a literature review chapter that will include a "tool selection" subsection, B's tables are directly copyable (after primary-source verification). B's platform coverage is also good: Atlas, Unitree H1/G1, Valkyrie, TALOS, Robotis OP3, Berkeley Humanoid.

**Report C.** Names the most *recent* platforms (Unitree G1/H1, Booster T1, Agility Digit, Berkeley Humanoid) and the right modern simulators (Isaac Lab, MJX, MuJoCo Playground RSS 2025). For a thesis positioning itself in the 2024–2026 landscape, C's platform-and-sim vocabulary is essential. But C does not teach the solver landscape, does not name the classical frameworks (Stack-of-Tasks / OpenSoT / TSID as libraries), and implicitly assumes the reader already knows what Pinocchio / Crocoddyl are.

**Ω verdict (R1) for lit-review utility.** A for the scaffold, B for the tables, C for the modern vocabulary. **Rank: A ≻ B ≻ C** on scaffold value for Chapter 2; **B ≻ C ≻ A** on tool-comparison-table value for Chapter 3; **C ≻ B ≻ A** on 2024–2026 SOTA vocabulary. The lit-review chapter needs *all three*, because the thesis must cover classical model-based (A-strength), tool-selection (B-strength), and modern SOTA (C-strength). **Confidence:** high.
**Where I may be wrong:** If the thesis's implementation plan locks in Pinocchio / Crocoddyl / OSQP / MuJoCo (per master_memory.md §8.2), the solver-table depth B provides matters less than I suggested — its value is *decision-support*, and the decision is nearly made.

---

### Φ — Simulation & Physics Expert (simulation-chapter readiness)

**Re-framing note.** For a lit-review chapter, simulation coverage must support two downstream chapters: Chapter 4 (Simulation Framework) and Chapter 5 (Experiments and Results). The lit-review section on simulation must teach the student (i) the simulator landscape as of 2026, (ii) the contact-modelling options and their trade-offs, (iii) the benchmark culture (or lack thereof) in humanoid WBC, and (iv) the sim-to-real discussion so the thesis can justify simulation-only scope (master_memory.md §1.5).

**Report A.** Dedicates §5 to simulation and is the only report that cites Ivaldi et al.'s IROS 2014 simulator survey — a foundational citation. The coverage is thesis-appropriate: engines (Gazebo, ODE, Bullet, XDE, MuJoCo), contact modelling (penalty / complementarity / impulse), solver choices (HQP/TSID real-time), and sim-to-real (ASAP, residual learning). The weakness: A is *behind the 2023–2026 curve* on GPU-parallel simulation. No MJX, no MuJoCo Playground, no HumanoidBench, no Isaac Lab. For a 2026 thesis, this is an incomplete picture.

**Report B.** The simulator comparison table is the only structured head-to-head artefact in the three reports. Claims about contact-model families are roughly correct (MuJoCo soft / RaiSim bisection / Drake relaxed complementarity / Isaac PhysX PGS). The gaps: no discussion of contact-model-to-controller consistency, no benchmark culture analysis, and sourcing is weak (SimBenchmark conflict-of-interest, undisclosed). For a lit-review chapter, B's table is copyable (with sourcing fixed); its narrative is thin.

**Report C.** The strongest on 2024–2026 simulation tooling — MuJoCo Playground (RSS 2025) and HumanoidBench (RSS 2024) are named and contextualized. Sferrazza et al.'s diagnostic finding ("flat end-to-end RL fails on long-horizon high-DOF whole-body tasks; hierarchical architectures with robust low-level skills succeed") is a *single-sentence synthesis* of enormous pedagogical value — a lit-review chapter that cites this finding gives the thesis an immediate justification for model-based WBC with learning only as a baseline, which matches master_memory.md §1.5. But C provides no contact-model comparison, no physics-engine discussion, no benchmark-protocol analysis.

**Φ verdict (R1) for lit-review utility.** A for the simulation-culture framing, B for the engine table, C for the 2024–2026 benchmark tooling. **Rank: A ≻ C ≻ B** for the simulation section of Chapter 2; but the chapter must incorporate B's table (rebuilt on primary sources) and C's benchmark findings (especially Sferrazza 2024 and the Playground release). **Confidence:** high.
**Where I may be wrong:** If master_memory.md PQ2 resolves toward Isaac Sim (not MuJoCo), C's coverage of Isaac Lab / Isaac Gym becomes more important than I have scored.

---

### Ψ — Thesis Supervisor / Committee Examiner (scholarly quality as a chapter)

**Re-framing note.** A master's-thesis Chapter 2 is not a publishable survey. It is a chapter that (i) demonstrates the student has read the relevant literature, (ii) defines the terminology used in the rest of the thesis, (iii) organizes prior work thematically (not chronologically as a list), (iv) ends with a critical positioning of the thesis within the field. It must serve the student's *argument*, not stand alone.

**Report A.** This is the document *most like what Chapter 2 should be*. Explicit structure (15 sections), explicit taxonomy (§10), explicit reading order (§14), explicit thesis niches (§12), explicit gap articulation mapped to methodology (§11). A thesis supervisor reading A would say "yes, you've read the field". The weaknesses are correctable: the TSID/HQP conflation can be fixed in one paragraph; the missing 2024–2026 SOTA (Dantec, Radosavovic, HOVER) can be added from C; the math can be strengthened from B; the non-rendered footnotes can be resolved to primary venues.

**Report B.** This reads like a consulting deliverable or a tutorial, *not* a literature review chapter. Executive summary, tables, keyword index, concluding opportunity list — these are features of a briefing, not of a scholarly chapter. The sourcing problem (PatSnap, Moonlight, RoboCloud, MANUS marketing copy) is fatal in a thesis context: a committee will reject Chapter 2 at first read if such citations appear. The LaTeX is corrupted. The "thesis opportunities" section over-promises (GPU-accelerated WBC solver in 12 months). For a lit-review chapter, B is *least* usable as-is — *but* its tables are the best engineering artefact across the three.

**Report C.** This is an *editorial*, not a literature review. It takes positions, argues for them, predicts consolidation — all good writing, all wrong genre. A master's-thesis Chapter 2 should not open by stating "the field has pivoted decisively from hand-crafted optimization stacks to learning-based whole-body controllers". Even if true, this is a contribution statement in a chapter that is supposed to *survey*, not *argue*. C's contents, rewritten into a survey voice, would be highly useful; the raw C text is not.

**Ψ verdict (R1) for lit-review utility.** **Rank: A ≻ C ≻ B** on chapter-structural fit. A is the scaffold, C is the supplement (modern SOTA content), B is the table source. The ranking is wider than in the publication-oriented run because lit-review quality depends on *survey-genre discipline*, which is A's dominant strength and C's dominant weakness. **Confidence:** high.
**Where I may be wrong:** If the supervisor values modern-SOTA coverage over structural discipline, C's ranking rises — but I have seen no thesis committee that forgives weak structure for strong opinion.

---

## ROUND 2 — Cross-Examination

### Σ responds

Φ's point that A is behind on 2024–2026 simulation tooling is the correction I most needed. I had treated A's simulation section as adequate because it cites Ivaldi; it is not adequate on its own for a 2026 thesis. I accept Ω's position that B's solver-and-simulator tables are the most directly *copyable artefacts* for the thesis's tool-selection discussion. I disagree most with **Ψ's treatment of C**: while C *is* an editorial, the student can rewrite C's content into survey voice in a few hours — this is much cheaper than replacing what A and B provide. My position after the exchange: unchanged on A ≻ B ≻ C for the mathematical scaffold, but I now view C as *strictly complementary* to A rather than competing with it.

### Ω responds

Φ's observation that HumanoidBench (Sferrazza RSS 2024) provides a *chapter-defining* single-sentence synthesis (hierarchical > flat RL) changes my view of C's value. That finding alone justifies the thesis's model-based framing in master_memory.md §1.5 and should anchor the chapter's learning-based-baseline discussion. I agree with **Σ** that B's formulas are correctable; I agree with **Ψ** that B's sourcing problem is fatal. What I disagree with is the implicit framing that the three reports must each be evaluated as a whole — the correct evaluation is *per asset*: A's structure, B's tables, C's 2024–2026 citations. Each asset is either usable or not; none of the three reports is usable as a unit. My position after the exchange: synthesize per-asset, as Ψ said in the first council run.

### Φ responds

Σ's point about the pedagogical role of mathematical rigor in a thesis chapter is well-taken. I had implicitly been scoring rigor as a T-RO-level axis; the correct axis is *does the chapter teach the reader the canonical equations*. Under that axis, A's maps-without-derivations is less of a deficit than I said, because the student can derive the equations in Chapter 3. I disagree most with **Ω's** claim that B's tables can be "directly copied (after sourcing fix)". The conflict-of-interest on SimBenchmark and the non-peer-reviewed sources are not cosmetic — the tables *as they stand* cannot be copied; they must be rebuilt from Todorov 2012 (MuJoCo), Carpentier 2019 (Pinocchio), Makoviychuk 2021 (Isaac Gym), Castro 2023 (Drake SAP), and Hwangbo 2018 (RaiSim). That is more work than a copy-paste, closer to a rewrite. Position after the exchange: the simulator comparison should be rewritten from primary sources, not copied.

### Ψ responds

Three peers converge on the same finding: **the three reports together provide complementary assets for a thesis chapter, and no single one is usable alone**. This is the correct reading. My Round-1 "A ≻ C ≻ B" ranking understates how much of A's text is directly reusable in Chapter 2 with minor edits. I revise upward: A is not just the scaffold, it is the *near-draft* of §2.1–§2.4 of Chapter 2; C is the mandatory supplement for §2.5–§2.6 (2024–2026 SOTA); B contributes tables for §2.7 (tool landscape) only. My final position: A's structural ranking is *higher* than my R1 score; C's is *approximately* as I scored; B's is *lower* — its tables are necessary but must be rebuilt, which in scholarly-utility terms is close to zero direct contribution.

---

## ROUND 3 — Final Positions (Lit-Review Lens)

### Σ (final)

A is the mathematical scaffold to populate. B's formulas are reusable drafts with one correction (support-consistent Jacobian). C is a citation-only asset for the mathematical layer. **Rank: A (0.75) > B (0.50) > C (0.45)** on mathematical-scaffold axis for Chapter 2.

### Ω (final)

A is the implementation-vocabulary scaffold. B's solver/simulator tables are high-value once rebuilt on primary sources. C's 2024–2026 platform vocabulary is mandatory for a 2026 thesis. **Rank: A (0.75) > B (0.50) > C (0.55)** on implementation-chapter-support axis — note B and C are close; they support different thesis sections.

### Φ (final)

A is the simulation-culture scaffold. B's engine comparison table is the core artefact but must be rebuilt. C's modern benchmark findings (HumanoidBench, MuJoCo Playground) are mandatory. **Rank: A (0.70) > C (0.60) > B (0.45)** on simulation-chapter-support axis.

### Ψ (final)

A is the chapter scaffold, full stop. C is the modern supplement. B contributes tables only. **Rank (lit-review utility): A (0.80) > C (0.55) > B (0.35)**.

---

### Unified Council Ranking (Lit-Review Lens)

| Axis | A | B | C |
|------|----|----|----|
| Mathematical scaffold (Σ) | 0.75 | 0.50 | 0.45 |
| Implementation vocabulary (Ω) | 0.75 | 0.50 | 0.55 |
| Simulation-chapter support (Φ) | 0.70 | 0.45 | 0.60 |
| Scholarly-chapter fit (Ψ) | 0.80 | 0.35 | 0.55 |
| **Equal-weight mean** | **0.75** | **0.45** | **0.54** |

### Consensus

**Report A is the primary scaffold for Chapter 2.** It is survey-style, structured, taxonomically explicit, and maps cleanly onto master_memory.md §12. **Report C is the mandatory 2024–2026 SOTA supplement** — without it, the chapter would read as if the field stopped developing in 2022. **Report B contributes only the simulator and solver tables**, which must be rebuilt from primary sources before appearing in the thesis. The synthesized Chapter-2 review (`03_Synthesized_Literature_Review.md`) is structured around A's skeleton with C's modern coverage injected and B's tables rewritten.
