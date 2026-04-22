# WBC Deep-Research Council — 2026-04-22 (Re-run: Literature Review Framing)

**Session purpose.** Compare three AI-generated deep-research outputs on *Whole-Body Control (WBC) and Simulation for Humanoid Robots*, and produce a synthesized scaffold for **Chapter 2 (State of the Art — Whole-Body Control for Humanoids)** of the master's thesis defined in `master_memory.md`.

**Important scope change from the first council run.** This deliverable is a **literature review for the thesis**, *not* a publishable standalone article. The evaluation axes shifted accordingly:
- "Publishability / novelty" is deprecated — a literature review synthesizes, it does not contribute.
- "Pedagogical completeness / coverage breadth" is promoted — the chapter must serve a reader encountering WBC for the first time.
- "Fit with `master_memory.md` §12 (thesis chapter plan)" becomes the dominant axis.
- "Internal consistency with the thesis's research question, hypotheses, and logical chain (master_memory.md §1, §13)" is enforced throughout.

**Inputs under review (unchanged).**

| Label | Source | File |
|-------|--------|------|
| **A** | Perplexity Deep Research | `Whole-Body Control and Simulation for Humanoid Robots A Literature Review.md` |
| **B** | Gemini Deep Research | `Humanoid Robot Whole-Body Control Review.md` |
| **C** | Claude Deep Research | `compass_artifact_wf-3e16c830-2a29-4fc5-901e-a9982fecead8_text_markdown(1).md` |

**Panel (4 experts, unchanged roles, new lens).**

1. **Σ — Control Theory Expert** — mathematical rigor *at thesis-chapter level* (not T-RO level).
2. **Ω — Robotics Systems Engineer** — implementation realism *for the specific system in master_memory.md §2*.
3. **Φ — Simulation & Physics Expert** — simulation-chapter readiness (fit with planned Chapter 4).
4. **Ψ — Thesis Supervisor / Committee Examiner** — scholarly quality of Chapter 2 as a *literature-review chapter* in a master's thesis.

**Protocol.** Single-agent fallback, disclosed: Round 1 blind independent analyses → Round 2 cross-examination → Round 3 final positions → meta-synthesis and deliverables.

**Deliverables (overwritten in this folder).**

| File | Purpose |
|------|---------|
| `01_Deliberation_Transcript.md` | Full three-round council transcript (lit-review framing) |
| `02_Comparative_Evaluation.md` | Re-ranked on lit-review axes; strengths, weaknesses, missing elements |
| `03_Synthesized_Literature_Review.md` | Thesis-Chapter-2-ready synthesized review, aligned with `master_memory.md` §12 |
| `04_Thesis_Chapter_Integration_and_Recommendations.md` | How to drop the synthesized review into the thesis; specific edit list |

**Note on ranking shift.** In the first (publication-oriented) run, Report C's bibliographic precision and argumentative spine pushed it close to Report A. In this (literature-review-oriented) rerun, Report A's explicit survey structure, taxonomy, and reading order pull ahead more clearly, because those are the artefacts Chapter 2 needs. Report C becomes a *supplement* (for 2024–2026 SOTA coverage). Report B remains the weakest overall, contributing only the simulator-comparison table.
