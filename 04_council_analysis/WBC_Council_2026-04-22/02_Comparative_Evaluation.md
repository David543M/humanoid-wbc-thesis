# Comparative Evaluation of the Three Deep-Research Outputs (Lit-Review Lens)

> *Re-evaluation scope.* This file re-ranks the three outputs against the axes that matter for **Chapter 2 (State of the Art — Whole-Body Control for Humanoids)** of the thesis defined in `master_memory.md` §12 — not for publishability. The synthesized Chapter-2 scaffold is in `03_Synthesized_Literature_Review.md`.

## 1. Ranked Comparative Table (Re-run)

Scores are 0–1 qualitative grades from the council. Axes are weighted by their relevance to the *literature-review chapter* objective.

| # | Lit-review axis (for Chapter 2) | 1st | 2nd | 3rd | Numerical A / B / C |
|---|---|---|---|---|---|
| 1 | Survey-genre discipline (thematic, not a list) | A | C | B | 0.80 / 0.35 / 0.50 |
| 2 | Pedagogical coverage of classical WBC (OSC, HQP, TSID, momentum) | A | B | C | 0.80 / 0.55 / 0.40 |
| 3 | Coverage of modern WBC (2023–2026: WB-MPC, RL, retargeting) | C | B | A | 0.80 / 0.55 / 0.40 |
| 4 | Explicit taxonomy and classification of methods | A | B | C | 0.85 / 0.55 / 0.25 |
| 5 | Mathematical foundations layout (equations, notation) | B | A | C | 0.55 / 0.50 / 0.20 |
| 6 | Simulation section readiness | A | C | B | 0.65 / 0.55 / 0.45 |
| 7 | Simulator / solver comparison artefacts (tables) | B | A | C | 0.65 / 0.45 / 0.40 |
| 8 | Research-gap articulation suitable for thesis gap-chapter | A | C | B | 0.75 / 0.45 / 0.30 |
| 9 | Bibliographic density on 2024–2026 SOTA | C | B | A | 0.85 / 0.55 / 0.40 |
| 10 | Source integrity (peer-reviewed venues, no marketing/blog links) | C | A | B | 0.70 / 0.55 / 0.20 |
| 11 | Alignment with `master_memory.md` terminology and scope | A | C | B | 0.80 / 0.55 / 0.45 |
| 12 | Reading order / study-path for a student | A | B | C | 0.85 / 0.40 / 0.20 |
| 13 | Reusability as Chapter 2 draft (with minor edits) | A | C | B | 0.80 / 0.45 / 0.30 |
| 14 | Critical-synthesis quality (compares approaches, not just lists) | C | A | B | 0.70 / 0.55 / 0.35 |
| 15 | Pedagogical usefulness to a WBC newcomer | A | B | C | 0.75 / 0.55 / 0.45 |
| **Equal-weight mean (lit-review utility)** | **A** | **C** | **B** | **0.69 / 0.47 / 0.49** |

**Headline shift from the publication-oriented council run.** Under the lit-review lens:
- **Report A moves up** (0.63 → 0.69). Its survey-genre discipline, taxonomy, reading order, and thesis-niche enumeration are exactly what Chapter 2 needs.
- **Report C moves down** (0.56 → 0.49). Its editorial voice is a *genre mismatch* for a lit-review chapter; its contents remain valuable, but as supplements, not as a scaffold.
- **Report B moves up slightly** (0.41 → 0.47). Its tables become more relevant when the thesis must justify tool selection in Chapter 3.

---

## 2. Strengths per Output — What Each Report Contributes to Chapter 2

### Report A (Perplexity) — the scaffold of Chapter 2

1. **Survey-genre structure** — 15 sections organized by (definition → history → math → architectures → simulation → tasks → challenges → papers → trends → taxonomy → gaps → thesis angles → reading order → keywords). This is the cleanest match in the three reports to the standard shape of a master's thesis Chapter 2.
2. **Explicit multi-axis taxonomy** — kinematic vs. dynamic, null-space vs. HQP vs. weighted QP, single-level vs. hierarchical vs. MPC back-ends, model-based vs. learning vs. hybrid, torque vs. position vs. SEA.
3. **Software-framework coverage** — Stack-of-Tasks, OpenSoT, SEIKO, TSID, Ramuzat's TALOS benchmark. These are the names a supervisor expects to see.
4. **Research-gap articulation mapped to methodology** — A §11 identifies six gaps that map one-to-one onto master_memory.md §7.
5. **Reading order for a master's student (§14)** — a *thesis-supervisory* artefact; no other report provides this.
6. **Thesis-angle enumeration (§12)** — three feasible master's-level niches, with direct relevance to master_memory.md §8.
7. **Ivaldi et al. simulator survey cited** — foundational for Chapter 4.

### Report C (Claude) — the mandatory 2024–2026 supplement

1. **Precise citations to venue-level resolution** for the modern SOTA: Dantec *ICRA 2021 / RA-L 2022 / Humanoids 2022*, Khazoom *Humanoids 2024*, Sleiman *RA-L 2021 / Sci. Robot. 2023*, Radosavovic *Sci. Robot. 2024*, Haarnoja *Sci. Robot. 2024*, Fu *CoRL 2024* (HumanPlus), He *CoRL 2024* (OmniH2O) and *arXiv 2410.21229* (HOVER), Sferrazza *RSS 2024* (HumanoidBench), Zakka *RSS 2025* (MuJoCo Playground).
2. **Emerging-consensus architecture pattern** — privileged teacher in sim / distilled student / low-level PD at 1–2 kHz / history-conditioned observations replacing system ID. This is the single clearest synthesis across the three reports of what 2024–2025 humanoid WBC systems actually look like.
3. **Diagnostic finding from HumanoidBench** — "flat end-to-end RL fails on long-horizon high-DOF whole-body tasks; hierarchical RL with robust low-level skills succeeds". A one-sentence citation that justifies the thesis's model-based framing.
4. **Venue-level bibliographic discipline overall** — the cleanest of the three on avoiding preprint-only or blog-level citations.

### Report B (Gemini) — tables only

1. **Explicit equations** — EoM, centroidal momentum $\dot L, \dot k$, friction cone, OSC closed form — usable as drafts in Chapter 3 *after* correcting the floating-base OSC to use the support-consistent Jacobian (Mistry 2010).
2. **Simulator comparison table** — RaiSim / MuJoCo / Isaac Sim / Drake on contact model, best-use, limitations. Structure is good; sources must be rebuilt.
3. **QP solver taxonomy** — qpOASES / OSQP / HPIPM / ProxQP mapping to problem regime.
4. **Two-layer MPC+WBC table** — useful pedagogy for Chapter 3.

---

## 3. Critical Weaknesses per Output (Harsh, Lit-Review Lens)

### Report A — weaknesses as a Chapter-2 scaffold

1. **Unresolved numbered footnotes.** 60+ `[^N]` markers inline; the reference list itself is not rendered in the source. Every citation must be resolved manually to a verifiable archival venue before transplanting into the thesis.
2. **Conflation of TSID and HQP.** A §4.3 blurs Del Prete's weighted-single-QP TSID with Escande's lexicographic HQP cascade. The thesis must state this distinction explicitly; Chapter 2 cannot inherit A's conflation.
3. **Behind the 2023–2026 curve on SOTA.** No Dantec whole-body MPC series, no Radosavovic / Haarnoja *Science Robotics*, no HOVER / HumanPlus / OmniH2O, no MuJoCo Playground / HumanoidBench, no Sleiman *Science Robotics*. For a 2026 thesis Chapter 2, this is a disqualifying gap — which is why C is mandatory as a supplement.
4. **Shallow learning-based treatment.** A §2.5 and §8.6 name ExBody / MHC / H2O without distinguishing their supervision signal, reward structure, or validation platforms.
5. **Non-rendered bibliography = citation-integrity risk.** The student cannot distinguish arXiv preprints from peer-reviewed venues in A's footnotes without reconstruction.
6. **Prose is occasionally *summary-flavoured* rather than *synthesis-flavoured*.** Some paragraphs read as "paper X did A; paper Y did B" without comparison. Chapter 2 must *compare*, not list.

### Report B — weaknesses as a Chapter-2 scaffold

1. **Non-peer-reviewed sources throughout.** PatSnap (twice), RoboCloud Hub (Vercel dashboard), themoonlight.io (Moonlight), MANUS marketing copy, SciSpace paraphrased abstracts, ResearchGate figure fetches. Carrying any of these into a thesis bibliography is a fail; four is systemic. This alone rules B out as a direct scaffold.
2. **Broken LaTeX.** Double-escaped backslashes (`\\in`, `\\mathbb`, `\\dot`) throughout. Would not compile in the thesis's Overleaf template.
3. **Technically incorrect floating-base OSC.** Eq. $\tau = J^T\Lambda\ddot x_\text{des} + N^T\tau_0$ written for humanoids without the support-consistent Jacobian — silently misrepresents Mistry 2010.
4. **Undisclosed conflict-of-interest citation.** "RaiSim 5–7× faster than MuJoCo" cited to SimBenchmark, maintained by the RaiSim author lab (ETH Hutter). Must be disclosed or removed.
5. **Shallow SOTA descriptions.** BumbleBee / ExBody / HOVER paraphrased from NeurIPS abstracts; specifics of training data, reward design, ablations absent.
6. **No explicit research-gap enumeration.** B §10 ("Research Gaps and Challenges") is three bullet paragraphs.
7. **Thesis-opportunities over-promise.** "Development of a GPU-accelerated WBC solver in 12 months" — not achievable at master's scale; should be deleted.
8. **Genre mismatch.** Reads as a consulting briefing, not a scholarly survey — the executive-summary / keyword-index / opportunities shape is wrong for Chapter 2.

### Report C — weaknesses as a Chapter-2 scaffold

1. **Genre mismatch: editorial, not survey.** C opens with *"The field has pivoted decisively…"* — this is an op-ed, not a literature review. Chapter 2 must adopt a neutral survey voice.
2. **No mathematical content.** No EoM, no friction cone, no task Jacobian, no QP formulation. Chapter 2's §3 (Mathematical Foundations) cannot be built from C.
3. **Foundational classical WBC under-covered.** Khatib 1987, Sentis & Khatib 2005/2006, Mistry 2010 get one-line mentions. Chapter 2 must treat these in ~2 pages, which C does not support.
4. **No explicit taxonomy or classification.** Taxonomy is implicit in prose. Chapter 2 should be able to lift a figure or table directly; C provides neither.
5. **No reading order for the student.** Nothing equivalent to A §14.
6. **No thesis-angle enumeration.** A student trying to choose a research niche gets no help from C.
7. **Position-controlled platforms (NAO, Pepper, SEIKO workflow) essentially absent.** Chapter 2 should cover these; C does not.
8. **Confidence markers absent.** Strong claims asserted without explicit uncertainty.

---

## 4. Missing Elements Across ALL Three (Must Be Added in the Synthesis)

These are gaps none of the three reports addresses and that Chapter 2 must fill from primary sources.

| # | Missing element | Where to place in Chapter 2 |
|---|---|---|
| M1 | **Formal distinction between weighted-QP (TSID) and lexicographic HQP** | §2.3 (Optimization-based WBC) |
| M2 | **Mistry 2010 impossibility result for task / null-space decoupling under underactuation** | §2.1 (Classical OSC) |
| M3 | **Henze–Roa–Ott passivity-based multi-contact balancer (IJRR 2016)** — the only passivity-certified WBC | §2.3 (Optimization-based WBC) |
| M4 | **Explicit QP-solver landscape comparison** — qpOASES / OSQP / ProxQP / eiquadprog / HPIPM with real-time regimes | §2.7 (Implementation Tooling) |
| M5 | **Contact-model–controller consistency discussion** | §2.5 (Simulation and Sim-to-Real) |
| M6 | **State-estimation pipeline** for floating-base humanoids (IMU-EKF + contact probability) | §2.6 (State Estimation, new) |
| M7 | **Quantitative sim-to-real error figures** for model-based WBC | §2.5 (Simulation and Sim-to-Real) |
| M8 | **Explicit evaluation-protocol discussion** citing Ramuzat et al. *Frontiers* 2022 | §2.8 (Evaluation and Benchmarking) |
| M9 | **Failure-mode catalog** for QP-based WBC (contact chattering, infeasibility, rank-deficient task Jacobians, torque saturation) | §2.9 (Limitations and Open Problems) |
| M10 | **Dantec-series whole-body MPC on TALOS** with explicit 10 ms solve time / 2 kHz Riccati-feedback discussion | §2.4 (Whole-body MPC) |
| M11 | **Sleiman *RA-L 2021 / Science Robotics 2023* loco-manipulation** as the model-based benchmark for simultaneous locomotion + manipulation | §2.4 (Whole-body MPC) |
| M12 | **Sferrazza RSS 2024 HumanoidBench finding** — hierarchical RL succeeds, flat RL fails | §2.6 (Learning-based WBC) |
| M13 | **HOVER / OmniH2O / HumanPlus trio** with explicit architectural comparison | §2.6 (Learning-based WBC) |
| M14 | **MuJoCo Playground / MJX (RSS 2025)** and Isaac Lab / Isaac Gym as the modern GPU-parallel training stack | §2.5 (Simulation) |

These 14 items form the concrete edit list against which the synthesized review was written.

---

## 5. Cross-Cutting Observations

1. **The three reports are complementary, not redundant.** A covers the survey structure, C covers the modern SOTA, B contributes only the tables. This is *fortunate* — any two of them overlap would force an arbitrary selection; as it stands, the synthesis has a clear merge recipe.
2. **The literature-review chapter is structurally close to done.** If the student adopts A as scaffold, integrates C's 2024–2026 citations into the modern-SOTA section, rebuilds B's tables from primary sources, and fills the 14 missing elements from primary sources, the chapter draft can be complete within 3–4 weeks.
3. **Pattern observation on AI deep-research outputs.** The three are classic complements: one is *structured*, one is *opinionated*, one is *tabular-engineering*. A thesis student using any of them in isolation would get a deficient result for different reasons. The council exercise is informative precisely because the failure modes are non-overlapping.

---

## Mini Critical Insight
Under a publication-oriented lens, the three outputs are ranked roughly A ≻ C ≻ B. Under a **literature-review-chapter lens**, the ranking becomes **A ≻ C ≻ B with a wider gap at both ends**: A's genre discipline is exactly what the chapter needs and it moves decisively ahead; C's editorial voice is a structural mismatch despite its citation quality and it drops; B's sourcing problem is fatal for a thesis bibliography and its utility collapses to "tables-only". The synthesized Chapter-2 review executes this re-ranking as a concrete merge.
