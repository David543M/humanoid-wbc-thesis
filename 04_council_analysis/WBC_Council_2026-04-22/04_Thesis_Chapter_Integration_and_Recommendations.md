# Thesis Chapter Integration and Recommendations
## How to Integrate `03_Synthesized_Literature_Review.md` into Chapter 2 of the Thesis

> **Scope of this file (re-run framing).** Per `00_README.md`, the 2026-04-22 re-run of the council was reframed from *publishability* to *literature-review integration into the thesis defined in `master_memory.md` §12*. The previous deliverable (`04_Publication_Positioning_and_Recommendations.md`, now legacy) organised recommendations around ICRA / RSS / IJRR venues. That framing is *deprecated* for a thesis Chapter 2, which synthesises rather than contributes. This file replaces it with an integration-focused plan: how the synthesized review maps onto the chapter plan, what must be edited, what must be preserved, and what cross-chapter consistency obligations follow.

---

## 1. Integration Objective

The synthesized review in `03_Synthesized_Literature_Review.md` (hereafter **SLR**) is designed to be *lifted, with author editing*, into **Chapter 2 — State of the Art — Whole-Body Control for Humanoids** of the thesis. Lifting does not mean copy-paste: the SLR is a scaffold whose twelve sub-sections (§2.0 – §2.12) map to, but are not identical to, the chapter plan in `master_memory.md` §12. The integration must:

1. Preserve the SLR's confidence-marker discipline (**[H] / [M] / [L]**) and the mini-critical-insight closing of every section (per `master_memory.md` §17 rule 8).
2. Resolve terminology and notation to the canonical forms in `master_memory.md` §10 and §17.
3. Route every claim through the Problem → Gap → Solution → Validation → Contribution chain of `master_memory.md` §13.
4. Close four of the six blocking pending questions in `master_memory.md` §16 (PQ1–PQ4) whose default answers the SLR has already argued for.

### Mini Critical Insight — 1
The SLR is *not* a draft of Chapter 2. It is a *scaffold at the correct level of abstraction for a Master's thesis*. The author's integration work is the editorial layer: trimming to the thesis's exact formulation, resolving stylistic inconsistencies, inserting transition paragraphs, and cross-linking to Chapter 3 notation. Treating the SLR as a finished chapter will produce a Chapter 2 that is too long, too general, and under-committed to the specific formulation that Chapters 3–5 will defend.

---

## 2. Mapping from Synthesized Review to `master_memory.md` §12 Chapter Plan

The SLR has twelve sub-sections; `master_memory.md` §12 allocates a single chapter (Ch. 2) to the state of the art. The recommended mapping is as follows.

| SLR section | SLR purpose | Target thesis location | Integration verdict |
|---|---|---|---|
| §2.0 Chapter Overview | Framing convergence-not-succession | Ch. 2 §1 (opening) | **Lift as-is**; rewrite opening paragraph in the author's voice |
| §2.1 Definition, scope, positioning | WBC vs. related control programs | Ch. 2 §2 | **Lift**; align every term with `master_memory.md` §10 |
| §2.2 Historical evolution (sub-§§2.2.1–2.2.6) | Five-wave historical arc | Ch. 2 §3 | **Condense** to ~60% of SLR length; keep all citations |
| §2.3 Mathematical foundations | EoM, CMM, QP, priority, stability metrics | **Split**: Ch. 2 §4 (overview level); Ch. 3 §1–§3 (formal level) | Do **not** duplicate — reference Ch. 3 from Ch. 2 |
| §2.4 Architectural taxonomy (7-axis) | Reusable classification | Ch. 2 §5 | **Lift**; will be reused in Ch. 3 to locate the thesis's commitments |
| §2.5 Simulators and sim-to-real | Simulator comparison + consistency gap | **Split**: Ch. 2 §6 (survey); Ch. 4 §1–§2 (simulator selection) | Keep the comparison table in Ch. 2; move the contact-consistency discussion to Ch. 4 or Ch. 6 |
| §2.6 Learning-based WBC (baseline) | RL coverage with diagnostic finding | Ch. 2 §7 | **Lift**; flag explicitly as baseline coverage per `master_memory.md` §3.5 |
| §2.7 Implementation tooling | Libraries and solvers | **Split**: Ch. 2 §8 (short landscape); Ch. 3 §4 (chosen stack) | Keep survey in Ch. 2; move choice-justification to Ch. 3 |
| §2.8 Evaluation and benchmarking | Ramuzat 2022 + HumanoidBench + gaps | Ch. 2 §9 | **Lift**; direct link to Ch. 5 (experiments) |
| §2.9 Open problems and research gaps | MG / EG / DG / TG taxonomy | Ch. 2 §10 | **Lift**; mirrors `master_memory.md` §7 one-to-one |
| §2.10 Thesis positioning | Thesis's seven commitments | Ch. 2 §11 | **Lift**; becomes the bridge to Ch. 3 |
| §2.11 Representative-works table | Reference table | Ch. 2 appendix or summary table | **Lift**; move to end of chapter |
| §2.12 Suggested study path | Pedagogical reading order | **Delete** from Chapter 2 (move to Appendix A or README of the repo) | Not appropriate inside the chapter body |

### Mini Critical Insight — 2
The SLR's §2.3 (mathematical foundations) is the single most ambiguous mapping target. Two failure modes are possible: (a) reproducing the equations in Chapter 2 creates a chapter that pre-empts Chapter 3's methodology; (b) removing them from Chapter 2 leaves a reader unable to understand the priority-handling discussion in §2.3.5. The recommended compromise — Chapter 2 states the equations at the *informational* level (one line per object) and Chapter 3 states them at the *formal* level (definition, derivation, constraint form) — preserves pedagogical completeness without duplication.

---

## 3. Section-by-Section Integration Plan for Chapter 2

Each entry below specifies the target thesis section, source SLR sub-section, minimum-edit list, and cross-references to other chapters.

### 3.1 Ch. 2 §1 — Chapter Overview
- **Source:** SLR §2.0.
- **Edits:** Rewrite the first paragraph in the author's voice; retain the *three-wave convergence* framing as the chapter's organising thesis; retain the confidence-marker explanation.
- **Cross-reference:** `master_memory.md` §1.2 (research question), §1.4 (hypotheses), §13 (logical chain).

### 3.2 Ch. 2 §2 — Definition and Positioning
- **Source:** SLR §2.1.
- **Edits:** Enforce the canonical term *Whole-Body Control (WBC)* per `master_memory.md` §17 rule 1; re-align the kinematic / dynamic / hierarchical tripartition against `master_memory.md` §10; explicitly state the thesis's placement in the taxonomy (*Dynamic × Hierarchical × torque-controlled × simulation-only*).
- **Cross-reference:** Chapter 3 §1 (formal WBC problem statement); Chapter 4 §1 (simulator choice rationale).

### 3.3 Ch. 2 §3 — Historical Evolution
- **Source:** SLR §§2.2.1–2.2.6.
- **Edits:** Condense to roughly 60% of SLR length — remove two or three of the cited follow-up papers per era and keep only the foundational citations plus the one-or-two highest-impact follow-ups. Verify every citation against `master_memory.md` §19 before inclusion.
- **Cross-reference:** SLR §2.10 (positioning) — every historical wave the thesis builds on is one the author must justify in §2.10.

### 3.4 Ch. 2 §4 — Mathematical Foundations (informational level)
- **Source:** SLR §§2.3.1–2.3.6 (reduced).
- **Edits:** State each object — EoM, task kinematics, centroidal momentum, contact constraints, QP formulation, priority mechanism, ZMP / capture point / DCM — in one or two lines, with the full derivation deferred to Chapter 3. Include Mistry 2010's *limitation result* here because it is historically important and motivates the optimization turn; the *formal* proof belongs in Chapter 3.
- **Cross-reference:** Chapter 3 §§1–3 (full formulation); `master_memory.md` §3.1 (WBC formulation axes).

### 3.5 Ch. 2 §5 — Architectural Taxonomy (7 axes)
- **Source:** SLR §2.4.
- **Edits:** Lift the seven-axis table verbatim. Mark each axis's thesis-commitment column as **TBD** or as the SLR §2.10 recommendation (Dynamic / TBD-priority / single-QP / full-RBD / hybrid horizon / model-based / torque). The TBDs close when `master_memory.md` §16 PQ1–PQ3 are resolved.
- **Cross-reference:** Chapter 3 §1 (the axes become Chapter 3's section structure).

### 3.6 Ch. 2 §6 — Simulators (survey level)
- **Source:** SLR §§2.5.1 and part of §2.5.3 (sim-to-real families).
- **Edits:** Keep the six-simulator comparison table. Rebuild each cell from primary sources per the SLR. Keep the conflict-of-interest disclosure on RaiSim. Move the contact-consistency discussion (SLR §2.5.2) out of Chapter 2 — see §3.13 below.
- **Cross-reference:** Chapter 4 §1 (simulator selection); Chapter 6 §2 (consistency-gap acknowledgement).

### 3.7 Ch. 2 §7 — Learning-Based WBC (Baseline Coverage)
- **Source:** SLR §2.6.
- **Edits:** Lift unchanged. Open the section with an explicit one-sentence statement that the material is surveyed per `master_memory.md` §3.5 as baseline coverage and is not a proposed approach. Keep the diagnostic finding about HumanoidBench's flat-vs-hierarchical result, which *justifies* the thesis's model-based design.
- **Cross-reference:** `master_memory.md` §3.5, §9.3 (baselines); Chapter 5 baselines.

### 3.8 Ch. 2 §8 — Implementation Tooling (short landscape)
- **Source:** SLR §2.7 (trim to one short paragraph per category).
- **Edits:** Keep the dynamics-library / QP-solver / OCP-library / WBC-framework four-way split. Name canonical references for each category; defer *choice justification* to Chapter 3.
- **Cross-reference:** Chapter 3 §4 (chosen stack); `master_memory.md` §8.2.

### 3.9 Ch. 2 §9 — Evaluation and Benchmarking
- **Source:** SLR §2.8.
- **Edits:** Lift unchanged. Explicit link to Chapter 5's experimental protocol as an extension of Ramuzat 2022.
- **Cross-reference:** Chapter 5 §§1–2 (scenarios S1–S5); `master_memory.md` §9.

### 3.10 Ch. 2 §10 — Open Problems and Research Gaps
- **Source:** SLR §2.9.
- **Edits:** Lift unchanged. Maintain the MG / EG / DG / TG labelling; this mirrors `master_memory.md` §7 one-to-one.
- **Cross-reference:** `master_memory.md` §7; Chapter 3 §1 (which gaps the proposed approach addresses).

### 3.11 Ch. 2 §11 — Thesis Positioning
- **Source:** SLR §2.10.
- **Edits:** Lift. This is the bridge paragraph that Chapter 3 opens against. Every commitment stated in §2.10 must be defended in Chapter 3 (priority scheme, solver choice, simulator, robot model, baseline strategy).
- **Cross-reference:** Chapter 3 (entire); `master_memory.md` §§8.1–8.3 (proposed approach).

### 3.12 Ch. 2 §12 — Summary Table
- **Source:** SLR §2.11.
- **Edits:** Move to end of chapter as a reference table. Optionally cut to ≤ 15 rows if the chapter is over length.
- **Cross-reference:** `master_memory.md` §19 (source integrity tracking) — every row must be a verified entry.

### 3.13 Material that does **not** go into Chapter 2

- **SLR §2.5.2 (contact-model–controller consistency problem).** Belongs in Chapter 4 §3 as a *methodological acknowledgement* of the simulator limitation, and revisited in Chapter 6 §2 as a *discussion of results' external validity*. Putting this in Chapter 2 confuses it for a reviewable open problem; it is in fact an *assumption boundary* of the thesis.
- **SLR §2.12 (study path).** Move to the repository README or an appendix. It is pedagogical metadata, not chapter content.

### Mini Critical Insight — 3
The integration pattern above enforces a strict *no-duplication* rule: equations live in Chapter 3, choices live in Chapter 3 / Chapter 4, and Chapter 2 is a *survey with explicit thesis placement*. This separation is the single most effective way to keep Chapter 2 below its natural bloat threshold (typical master's Chapter 2s inflate to 50+ pages without this discipline).

---

## 4. Cross-Chapter Consistency Obligations

Integrating the SLR into Chapter 2 creates downstream obligations that the other chapters must honour. These are not optional edits; they are consistency requirements per `master_memory.md` §9 (Thesis Coherence & Memory File Awareness).

### 4.1 Chapter 1 (Introduction)
- Must motivate the thesis using gaps **EG1 / EG2 / EG3 / MG1** from SLR §2.9, mapped to `master_memory.md` §7.
- Must state the research question as in `master_memory.md` §1.2 and the three hypotheses (§1.4) in their canonical form.
- Must adopt the SLR §2.0 *convergence-not-succession* framing when narrating the field.

### 4.2 Chapter 3 (Methodology)
- Must pick up exactly the seven axes from SLR §2.4 and state the thesis's commitment on each, closing PQ1–PQ4 from `master_memory.md` §16.
- Must formalise the equations stated informationally in Ch. 2 §4 — EoM with support-consistent Jacobian, CMM identity, friction-cone linearisation, QP formulation, priority mechanism.
- Must cite Mistry 2010's limitation result *as the formal motivation* for the optimization-based WBC form.

### 4.3 Chapter 4 (Simulation Framework)
- Must inherit the six-simulator comparison from Ch. 2 §6 and justify the thesis's choice against the row-by-row evidence.
- Must address the contact-model–controller consistency problem (SLR §2.5.2) as a methodological assumption, including any sensitivity analysis performed.

### 4.4 Chapter 5 (Experiments and Results)
- Must extend Ramuzat 2022's protocol per SLR §2.8, adding scenarios S4 (loco-manipulation) and S5 (perturbation) per `master_memory.md` §9.4.
- Must ground the metric thresholds in `master_memory.md` §9.2 against published values in Ramuzat 2022, Dantec 2022, Koolen 2016.
- Must report a published RL baseline (e.g., a Unitree H1 open-source policy), not a policy trained in-house, per `master_memory.md` §1.5 (scope) and SLR §2.6 discussion.

### 4.5 Chapter 6 (Discussion)
- Must explicitly acknowledge the contact-model–controller consistency gap (SLR §2.5.2) as a threat to the external validity of simulation claims.
- Must address the anticipated criticisms in `master_memory.md` §18 — each one has a cited response in the SLR.

### Mini Critical Insight — 4
The SLR was written with cross-chapter coherence in mind: every section's mini-critical-insight paragraph names the chapter(s) that will later carry the full argument. This is not decorative. A Chapter 2 draft that ignores these forward references will detach from the thesis's logical chain and require rework at defence time.

---

## 5. Required Updates to `master_memory.md`

The SLR and this integration plan imply changes to `master_memory.md` that must be committed before Chapter 2 drafting begins. These are consistency updates, not new content.

| § | Current content | Required update | Source |
|---|---|---|---|
| §4.1 Method categories | Placeholder list | Populate with the 7-axis SLR taxonomy entries | SLR §2.4 |
| §5 Accepted knowledge | 5 entries, all confidence-tagged | Add: *Hierarchical RL succeeds where flat RL fails on long-horizon WBC* [Sferrazza 2024, **[H]**] | SLR §2.6 diagnostic finding |
| §6 Controversies | 3 controversies listed | Add: *Contact-model–controller consistency gap* (currently absent from `master_memory.md`) | SLR §2.5.2 |
| §7 Research gaps | Populated but unlabelled | Re-label entries as MG / EG / DG / TG per SLR §2.9 | SLR §2.9 |
| §16 PQ1 | Robot model undecided | Commit: **TALOS primary, Unitree H1 alternative** per SLR §2.10; close PQ1 | SLR §2.10 |
| §16 PQ2 | Simulator undecided | Commit: **MuJoCo primary, Drake secondary-verification** per SLR §2.5.1 / §2.10 | SLR §2.5 / §2.10 |
| §16 PQ3 | WBC formulation undecided | Commit: **weighted single-QP (TSID-style) with centroidal MPC planner, HQP comparison baseline** per SLR §2.3.5 / §2.10 | SLR §2.3 / §2.10 |
| §16 PQ4 | Novel contribution undefined | Reframed: the thesis's contribution is a *reproducible simulation-phase evaluation framework* extending Ramuzat 2022, with contact-model sensitivity; not a new WBC formulation | SLR §2.10 |
| §19 Source integrity tracking | 7 references listed | Add the ≥ 20 additional verified references from SLR §2.11 | SLR §2.11 |
| §23 Session log | Latest 2026-04-22 entry | Append: *WBC Council rerun (literature-review framing) — SLR produced; integration plan committed* | This file |

> **Note on PQ4.** `master_memory.md` §16 currently frames PQ4 as a question about *novelty*. Under the literature-review framing of this rerun, *novelty at the controller level is not claimed*. The thesis's contribution is framed as a **reproducible evaluation protocol + contact-model sensitivity analysis extending Ramuzat 2022** (SLR §2.10). This is the resolution that reconciles `master_memory.md` §13's logical chain (Problem → Gap → Solution → Validation → Contribution) with the field's saturation.

### Mini Critical Insight — 5
Closing PQ1–PQ4 is the *precondition* for Chapter 3 drafting. Without these commitments the Chapter 2 positioning (SLR §2.10) has no target for Chapter 3 to defend, and the logical chain collapses at the *Solution* node. The SLR's recommended defaults are defensible against the literature surveyed; they are safer than waiting for an external event to force the choice.

---

## 6. Prioritized Edit List (Thesis Integration)

Tagged by target chapter and priority. These are the specific actions the author must execute to integrate the SLR.

### Priority 🔴 P1 — blocking for Chapter 2 draft

| # | Edit | Target | Estimated effort |
|---|---|---|---|
| I1 | Close `master_memory.md` §16 PQ1–PQ4 with the SLR §2.10 defaults (or justified alternatives) | `master_memory.md` | 0.5 day |
| I2 | Update `master_memory.md` §§4.1, 5, 6, 7, 19 per §5 above | `master_memory.md` | 1 day |
| I3 | Create Ch. 2 skeleton file under `06_writing/chapters/ch2_state_of_the_art.md` with the 12 sections mapped in §2 above | Ch. 2 | 0.5 day |
| I4 | Lift SLR §§2.0, 2.1, 2.4, 2.10, 2.11 into Ch. 2 §§1, 2, 5, 11, 12 with editorial pass | Ch. 2 | 2 days |
| I5 | Lift SLR §2.2 into Ch. 2 §3 with ~40% condensation | Ch. 2 | 2 days |
| I6 | Lift SLR §§2.6, 2.8, 2.9 into Ch. 2 §§7, 9, 10 with minimal editing | Ch. 2 | 1 day |

### Priority 🟠 P2 — blocking for a coherent chapter

| # | Edit | Target | Estimated effort |
|---|---|---|---|
| I7 | Write Ch. 2 §4 (math overview, one line per object) using SLR §2.3 as source; keep Chapter 3 derivations out of Ch. 2 | Ch. 2 | 1.5 days |
| I8 | Write Ch. 2 §6 (simulator survey) from SLR §2.5.1 table; move SLR §2.5.2 consistency discussion into Ch. 4 skeleton | Ch. 2, Ch. 4 | 1 day |
| I9 | Write Ch. 2 §8 (tooling landscape, short) from SLR §2.7; defer choice justification to Ch. 3 | Ch. 2 | 0.5 day |
| I10 | Insert transition paragraphs between every pair of sections; ensure each ends with a mini-critical-insight per `master_memory.md` §17 rule 8 | Ch. 2 | 1 day |
| I11 | Audit every citation in Ch. 2 against `master_memory.md` §19; any source not yet marked *Verified* must be fetched, read, and checked before appearing in the final manuscript | Ch. 2, `master_memory.md` §19 | 3–5 days (concurrent) |

### Priority 🟡 P3 — improves chapter quality

| # | Edit | Target | Estimated effort |
|---|---|---|---|
| I12 | Replace any numbered-footnote references `[^N]` from Report A with `master_memory.md` §19 BibKey-style entries | Ch. 2 | 0.5 day |
| I13 | Remove all non-peer-reviewed Report B citations (PatSnap, RoboCloud, Moonlight, ResearchGate figure fetches) | Ch. 2 | 0.5 day |
| I14 | Add confidence markers **[H] / [M] / [L]** to every non-trivial claim carried over from source reports | Ch. 2 | 1 day |
| I15 | Insert forward references ("see Chapter N §M") at every SLR cross-reference point | Ch. 2 | 0.5 day |
| I16 | Write Ch. 2 opening (SLR §2.0) and closing paragraphs in the author's own voice; do not lift these verbatim | Ch. 2 | 0.5 day |

### Priority 🟢 P4 — optional, high value

| # | Edit | Target | Estimated effort |
|---|---|---|---|
| I17 | Move SLR §2.12 (study path) to repository README under `03_processed/` as an onboarding document | Repo | 0.25 day |
| I18 | Build a BibTeX file from SLR §2.11 for use in the Overleaf template (`humanoid_wbc_thesis_overleaf.zip`) | `08_references/` | 0.5 day |
| I19 | Cross-check every Chapter 2 claim against the Chapter 3 skeleton to detect duplication before it ossifies | Ch. 2, Ch. 3 | 0.5 day |

### Mini Critical Insight — 6
The total effort estimate for P1 + P2 is roughly 13–15 working days. This is the realistic lower bound for a Chapter 2 that passes a supervisor review on first pass; under-budgeting it produces a chapter that requires a second full pass after supervisor feedback, doubling the total effort. The student should plan accordingly and defer Chapter 3 drafting until Chapter 2 is stable.

---

## 7. Continuity and Coherence Checks

Before merging the Chapter 2 draft into the thesis main document, the author must verify the following consistency checks (per `master_memory.md` §9 and §17).

### 7.1 Terminology
- Every instance of "whole-body control" is written as **Whole-Body Control (WBC)** — with hyphen, with acronym on first use per chapter (`master_memory.md` §17 rule 1).
- "QP" is spelled out as *Quadratic Program* on first use per chapter (`master_memory.md` §17 rule 3).
- "Humanoid robot" is used; "bipedal robot" only when the bipedal structure is the load-bearing reference (`master_memory.md` §17 rule 2).
- Canonical definitions from `master_memory.md` §10 are *reused*, not redefined.

### 7.2 Logical chain (`master_memory.md` §13)
- The chapter opens on *Problem* (unsolved / underspecified humanoid WBC), moves to *Gaps* (SLR §2.9 / `master_memory.md` §7), builds toward *Solution* positioning (SLR §2.10), and foreshadows *Validation* (SLR §2.8 and Ch. 5) and *Contribution* (Ch. 2 §11 closing).
- Every claim can be traced back to one of the five nodes of the chain. Orphan claims are deleted or reframed.

### 7.3 Hypothesis alignment (`master_memory.md` §1.4)
- The chapter's closing positioning (Ch. 2 §11) must leave the reader able to state the three hypotheses H1, H2, H3 in one sentence each, with the relevant open problem.
- If a hypothesis cannot be motivated by material already in Chapter 2, either the hypothesis or the literature selection is miscalibrated.

### 7.4 Source integrity (`master_memory.md` §19)
- Every citation in Chapter 2 appears in `master_memory.md` §19 with a Verified flag.
- No peer-reviewed claim is supported by a blog, aggregator, or marketing source.
- Non-peer-reviewed citations (arXiv, preprint) are explicitly marked with **[M]** or **[L]**.

### 7.5 Style (`master_memory.md` §17)
- No use of "clearly", "obviously", "simply" (rule 4).
- No uncited factual statement (rule 5).
- Every section closes with a *mini-critical-insight* paragraph (rule 8).
- Equations use consistent notation with Chapter 3 (rule 6).

### Mini Critical Insight — 7
These five checks are independent of content quality — they are the hygiene layer. A Chapter 2 draft that passes a supervisor review on content but fails on terminology or source-integrity hygiene will still require a full rewrite pass. Automating these checks with a simple lint script (grep for banned phrases, cross-reference BibKeys against §19) pays for itself within the first review cycle.

---

## 8. Risks and Mitigations in the Integration

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Chapter 2 exceeds natural length (50+ pages) due to verbatim lifting | High | Medium | Enforce the no-duplication rule in §3.13 and the 40% condensation of §2.2 (edit I5) |
| Chapter 2 and Chapter 3 repeat equations, producing reader fatigue | High | Medium | Strict split per §3.4: informational in Ch. 2, formal in Ch. 3; cross-link explicitly |
| The contact-consistency discussion (SLR §2.5.2) drifts into Chapter 2 and pre-empts Chapter 6 | Medium | High | Move to Chapter 4 §3 and Chapter 6 §2 per §4.3, §4.5 above |
| PQ1–PQ4 remain open when Chapter 2 is drafted, leaving §2.10 positioning abstract | High | High | Close PQ1–PQ4 in edit I1 *before* Chapter 2 drafting begins |
| Author's voice is lost by verbatim lifting of SLR paragraphs | Medium | Medium | Rewrite openings and closings of each section in the author's voice (edit I16) |
| Source integrity slips during rapid drafting, non-peer-reviewed citations leak in | Medium | High | Apply hygiene lint (§7) before every commit; audit at end of P2 (edit I11) |
| Supervisor rejects the "evaluation-protocol as contribution" reframing (PQ4 resolution) | Low | Very high | Raise in first supervisor meeting after this rerun; fall-back: narrower "working WBC implementation" framing per `master_memory.md` §18 |
| 2025–2026 SOTA citations age before thesis defence | Low | Low | Timestamp each SLR citation (*as of 2026-04-22*); refresh at the start of thesis final review |

### Mini Critical Insight — 8
The highest-impact risk is PQ4 — if the supervisor rejects the evaluation-protocol framing, every downstream commitment in Chapters 2–5 shifts. The mitigation is to surface this framing explicitly in the next supervisor meeting, armed with the SLR §2.10 argument, rather than inferring alignment from silence. A short (≤ 2-page) framing note, extracted from SLR §2.10 and §5 of this file, is an appropriate artefact for that meeting.

---

## 9. Closing Recommendations

1. **Treat the SLR as a scaffold, not a finished chapter.** The editorial pass — condensation, author-voice rewriting, cross-linking, hygiene lint — is 30% of the integration effort and the difference between a defensible and a rejected Chapter 2.

2. **Close `master_memory.md` §16 PQ1–PQ4 before Chapter 2 drafting.** The SLR §2.10 defaults are defensible and internally consistent. Deferring the commitments defers Chapter 3 indefinitely.

3. **Enforce the Chapter 2 / Chapter 3 split strictly.** Chapter 2 is *informational*; Chapter 3 is *formal*. Duplicated equations are the most common failure mode in master's theses and easily avoided with discipline from the start.

4. **Move the contact-consistency discussion out of Chapter 2.** SLR §2.5.2 is a *methodological assumption* of the thesis — it belongs to Chapters 4 and 6, not to the state-of-the-art survey.

5. **Apply the hygiene lint (§7) continuously.** Terminology, logical-chain traceability, and source-integrity checks are cheaper to enforce at draft time than at defence time.

6. **Use Ramuzat 2022 as the anchor.** It is the single most thesis-actionable reference in the entire surveyed literature; every simulation choice, metric, scenario, and baseline selection is simplified by taking Ramuzat as the reference extension target (per SLR §2.8, §2.10).

7. **Do not pursue venue-positioning decisions in this chapter.** The previous `04_Publication_Positioning_and_Recommendations.md` is now legacy. Publication planning belongs to a later stage, after Chapter 5 results exist.

8. **Commit the updated `master_memory.md` and the Chapter 2 skeleton to Git on the same day.** Per `master_memory.md` §20.3, use commit prefixes `[mem]` for the memory update and `[chapter-2]` for the skeleton. Traceability of the integration decisions matters for the thesis defence.

9. **Re-read `master_memory.md` at the start of every Chapter 2 drafting session.** Per the project instructions, it is the single source of truth; the SLR feeds it, does not replace it.

10. **Defer Chapter 3 drafting until Chapter 2 §11 (positioning) is stable.** Chapter 3 opens against §11; drafting in parallel produces interface inconsistencies that require a rewrite of both.

### Mini Critical Insight — Closing
The literature-review-framing rerun's core insight is that *Chapter 2 is a scholarly survey, not a contribution*. The synthesized review in `03_Synthesized_Literature_Review.md` and the integration plan in this file together reduce Chapter 2's remaining work to an *editorial and hygiene task* rather than a research task. This is by design: the research effort should be concentrated in Chapters 3–5, where the thesis's actual contribution — a reproducible simulation-phase evaluation framework — lives. The integration plan here is the lightest-weight path from the council's synthesis to a defensible Chapter 2 draft, consistent with `master_memory.md` at every step.
