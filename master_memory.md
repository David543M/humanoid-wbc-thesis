# 🧠 Thesis Master Memory File
**Single Source of Truth — Updated Continuously**
*Last updated: 2026-04-22 | Status: Active*

> ⚠️ **Protocol**: Before generating any new content, Claude must read this file and align with its terminology, structure, and logical chain. Any detected inconsistency must be flagged before proceeding.

---

## 1. 🎯 Research Framing

### 1.1 Research Title (Working)
> Development and Simulation of a Whole-Body Control Framework for a Humanoid Robot

### 1.2 Research Question
> To be formalized — must be precise, testable, and non-vague. Suggested draft:
>
> *"How can a whole-body control framework be designed and validated in simulation to enable a humanoid robot to perform stable, multi-contact locomotion and manipulation tasks under dynamic constraints?"*

### 1.3 Sub-Questions
- **SQ1:** What are the key limitations of existing WBC formulations (task-priority, QP, MPC) when applied to humanoid full-body coordination?
- **SQ2:** How can contact dynamics be modeled and integrated in a simulation environment to faithfully replicate hardware behavior?
- **SQ3:** What metrics and scenarios constitute a rigorous simulation-based validation of a WBC framework for humanoids?

### 1.4 Hypotheses

| ID | Hypothesis | Expected Evidence | Falsification Criterion |
|----|-----------|-------------------|------------------------|
| H1 | A hierarchical QP-based WBC formulation can achieve stable whole-body coordination across locomotion and manipulation tasks simultaneously | Task success rate > X% across N scenarios | Persistent constraint violations or instability in > Y% of trials |
| H2 | Simulation-based validation (Isaac Sim / MuJoCo) is sufficient to demonstrate proof-of-concept for the proposed WBC framework before hardware deployment | Consistent task completion in sim under varied perturbations | Significant performance degradation attributable to sim-to-real gap artifacts in sim itself |
| H3 | Explicit contact modeling and task-priority hierarchies outperform naive torque control in multi-contact transitions | Lower CoM deviation and contact force variance during transitions | Equivalent or worse metrics vs. baseline torque control |

### 1.5 Scope

| Dimension | In Scope | Out of Scope |
|-----------|----------|--------------|
| **Timeframe** | Simulation-phase only (no physical hardware deployment) | Real-robot hardware experiments |
| **Domain** | Whole-body control, humanoid locomotion and loco-manipulation | Manipulation-only arms, wheeled robots |
| **Systems** | Humanoid platforms (e.g., URDF-based model in Isaac Sim / MuJoCo) | Quadrupeds, soft robots, swarm systems |
| **Control** | WBC, task-space control, QP formulations, basic MPC | Deep RL as primary controller (may appear as baseline) |
| **Learning** | RL as comparison baseline only | End-to-end learned policies as main contribution |

---

## 2. 🤖 System Context

### 2.1 Target System
- **Type:** Humanoid (bipedal, full-body)
- **DOF:** To be specified (typically 28–32 DOF for humanoids — torso + arms + legs)
- **Sensors:** IMU (base orientation/angular velocity), joint encoders, contact force sensors (feet)
- **Actuators:** Torque-controlled joints (SEA or ideal torque sources in simulation)
- **Environment:** Flat ground, stairs, obstacle fields (simulation)

### 2.2 Application Domain
Simulation-based proof-of-concept for whole-body control enabling simultaneous locomotion and manipulation — targeting eventual deployment in service robotics or industrial assistance contexts.

### 2.3 Constraints

| Constraint Type | Details |
|----------------|---------|
| **Real-time** | WBC solve time must remain < 1 ms (QP) or < 10 ms (MPC) per control cycle |
| **Energy** | Not a primary constraint in simulation phase; noted for future hardware relevance |
| **Hardware** | None in sim; URDF fidelity and actuator model accuracy are the proxies |
| **Safety** | Joint limits, torque saturation, contact force bounds must be enforced as QP constraints |

---

## 3. 🧩 Core Technical Axes

### 3.1 Whole-Body Control Formulation

| Approach | Principle | Key Pros | Key Cons |
|----------|-----------|----------|---------|
| **Task-Priority (Null-Space)** | Hierarchical task stacking via null-space projections | Simple, fast | Singularity issues, no constraint handling |
| **QP-based WBC** | Minimize cost subject to rigid-body dynamics + contact constraints | Constraint-aware, torque-level | Computationally heavier, requires contact model |
| **Model Predictive Control (MPC)** | Receding-horizon optimization over future states | Anticipatory, handles intermittent contacts | High computational cost, model dependency |
| **Learning-based (RL)** | Policy learned via reward shaping | Robust to model errors | Black-box, poor interpretability, sim-to-real gap |

> **Adopted approach:** QP-based WBC (primary) — to be confirmed.

### 3.2 Contact Modeling

- **Rigid contact:** No penetration, LCP-based; standard for simulation
- **Soft contact:** Compliant contact models (MuJoCo spring-damper); more stable numerically
- **Key challenge:** Contact mode switching (stance ↔ swing) introduces hybrid dynamics discontinuities
- **Failure cases:** Foot chattering, constraint infeasibility at contact transitions

### 3.3 State Estimation

- **Floating-base estimation:** EKF on IMU + joint encoders to estimate base pose/velocity
- **Foot contact detection:** Threshold-based or probabilistic (force/torque sensor readings)
- **Key trade-off:** Speed vs. accuracy of base velocity estimation; critical for WBC stability

### 3.4 Planning Layer

- **Footstep planning:** Simplified model (LIP / SLIP) or pre-scripted sequences
- **Task scheduling:** Priority assignment for locomotion vs. manipulation tasks
- **Coupling:** Planning output feeds reference trajectories into WBC controller

### 3.5 Learning-Based Components (Baseline Only)

- **Role in thesis:** Used as performance comparison baseline, not primary contribution
- **Representative works:** Cassie (Agility Robotics RL), Unitree H1 RL policies
- **Generalization issues:** Reward hacking, domain randomization dependency

---

## 4. 📚 Literature Taxonomy

### 4.1 Method Categories

| Category | Description | Representative Papers |
|----------|-------------|----------------------|
| **Model-based WBC** | QP/task-priority on rigid-body dynamics | Sentis & Khatib (2005), Wensing & Orin (2013), Koolen et al. (2016) |
| **MPC for humanoids** | Centroidal MPC, footstep-integrated planning | Diedam et al., Caron et al., Winkler et al. (Towr) |
| **Learning-based** | RL for locomotion | Schulman (PPO), Hejna et al., Kumar et al., Zhuang et al. |
| **Hybrid (model + learning)** | WBC as structured policy layer | Kolt et al., Merel et al. |

### 4.2 Comparison Dimensions

| Dimension | Notes |
|-----------|-------|
| **Accuracy** | Tracking error for CoM / end-effector trajectories |
| **Robustness** | Recovery from perturbations, contact changes |
| **Computational cost** | QP solve time, MPC horizon depth |
| **Scalability** | DOF scaling, task hierarchy depth |
| **Real-time capability** | Control loop frequency (typically 500 Hz–1 kHz for torque control) |

### 4.3 Benchmark Platforms & Simulators

| Platform | Type | Simulator | Notes |
|----------|------|-----------|-------|
| Atlas (Boston Dynamics) | Humanoid | Gazebo / custom | Reference platform for WBC literature |
| TALOS (PAL Robotics) | Humanoid | Pinocchio / MuJoCo | Open-source, well-documented |
| Unitree H1/G1 | Humanoid | MuJoCo / Isaac Sim | Accessible, recent RL benchmarks |
| Custom URDF model | Humanoid | To be defined | Thesis system |
| **Simulators** | | | |
| MuJoCo | Physics sim | — | Fast, stable contacts, widely used |
| Isaac Sim (NVIDIA) | Physics sim | — | GPU-accelerated, photorealistic |
| Gazebo + ROS2 | Physics sim | — | Ecosystem integration |
| Pinocchio + Crocoddyl | Dynamics lib | — | Best-in-class for WBC, optimal control |

---

## 5. 📊 Accepted Knowledge (Validated Claims)

| Claim | Sources | Confidence | Notes |
|-------|---------|-----------|-------|
| QP-based WBC enables strict constraint enforcement (torque limits, contact forces) at joint level | Sentis 2005, Wensing 2013, Koolen 2016 | High | Core consensus in model-based WBC |
| Null-space projection methods fail gracefully at kinematic singularities | Multiple | Medium | Well-known but mitigation strategies vary |
| Centroidal dynamics is a sufficient reduced model for locomotion planning | Orin 2013, Caron et al. | High | Broadly accepted for planning layer |
| RL-based controllers outperform WBC in agility but sacrifice interpretability and constraint guarantees | Kumar 2021, Zhuang 2023 | Medium | Trade-off accepted; no universal benchmark |
| Sim-to-real transfer remains a major open challenge for torque-level WBC | Multiple | High | Universally acknowledged gap |

---

## 6. ⚠️ Controversies & Open Problems

- **Problem 1: Contact model fidelity in simulation**
  - *Conflicting findings:* MuJoCo soft contacts yield more stable sim behavior but diverge from rigid hardware reality; some argue rigid LCP is more physically faithful
  - *Possible explanation:* Depends on foot sole compliance and controller bandwidth
  - *Status:* Unresolved; requires empirical validation

- **Problem 2: Task priority vs. QP — which is "better"?**
  - *Conflicting findings:* Null-space approaches are faster but the QP formulation handles constraints more rigorously; benchmarks are not standardized
  - *Possible explanation:* Context-dependence (number of tasks, real-time requirements)
  - *Status:* Ongoing debate; thesis should take a principled position

- **Problem 3: Centroidal vs. full-body dynamics for WBC**
  - *Conflicting findings:* Centroidal MPC is computationally tractable but loses arm/torso interaction effects; full-body optimization is intractable at high frequency
  - *Possible explanation:* Hierarchical decomposition is the pragmatic solution, but introduces interface errors
  - *Status:* Active research area

---

## 7. 🔍 Research Gaps (CRITICAL SECTION)

### 7.1 Data Gaps
- Absence of standardized benchmark scenarios for humanoid WBC evaluation across the literature (each paper uses custom scenarios)
- Lack of public datasets for multi-contact force distributions during humanoid loco-manipulation

### 7.2 Methodological Gaps
- No consensus framework for coupling a planning layer (footstep + task scheduling) with a QP-based WBC at different time scales
- Contact transition handling (stance ↔ swing) is typically hand-tuned per platform, with no generalized method
- Integration of arm/manipulation tasks into locomotion WBC without task-priority conflicts is underexplored for humanoids

### 7.3 Evaluation Gaps
- Most papers evaluate either locomotion OR manipulation, rarely the simultaneous loco-manipulation scenario
- Perturbation robustness is rarely reported with reproducible protocols
- Simulation-to-hardware performance degradation is seldom quantified rigorously

### 7.4 Application Gaps
- WBC frameworks are rarely tested beyond laboratory settings
- Long-horizon task sequencing under WBC is largely unstudied
- Energy efficiency under WBC has received limited attention despite hardware relevance

---

## 8. 🧪 Proposed Approach (Your Contribution)

### 8.1 Core Idea
> To be finalized. Proposed direction: Design, implement, and validate a hierarchical whole-body control framework for a humanoid robot in simulation, combining a centroidal MPC planning layer with a QP-based WBC execution layer, evaluated on simultaneous locomotion and manipulation tasks.

### 8.2 Methodology

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **Dynamics library** | Pinocchio | Industry standard, efficient rigid-body algorithms |
| **QP solver** | OSQP / qpOASES | Fast, warm-startable, suited for real-time WBC |
| **Planning layer** | Centroidal MPC (simplified) | Computationally tractable, proven for locomotion |
| **Simulator** | MuJoCo or Isaac Sim | To be decided based on contact fidelity needs |
| **Robot model** | URDF humanoid (TALOS or custom) | To be finalized |
| **ROS2 integration** | Optional | Ecosystem compatibility |

### 8.3 Expected Advantages vs. Existing Methods
- Explicit constraint satisfaction (joint limits, contact forces) vs. unconstrained baselines
- Simultaneous locomotion + manipulation without task switching overhead
- Reproducible simulation-based evaluation protocol (addresses evaluation gap)

### 8.4 Risks

| Risk | Type | Mitigation |
|------|------|------------|
| QP infeasibility at contact transitions | Technical | Soft constraint relaxation, warm-starting |
| Sim-to-real gap invalidating conclusions | Experimental | Limit claims to sim-validated proof-of-concept |
| Overly complex framework, insufficient time | Scope | Modular design, reduce horizon/DOF if needed |
| Chosen simulator not reproducing literature results | Technical | Validate against published benchmark scenarios first |

---

## 9. 📈 Experimental Design

### 9.1 Setup

| Item | Choice |
|------|--------|
| **Mode** | Simulation only |
| **Primary simulator** | To be decided (MuJoCo / Isaac Sim) |
| **Dynamics library** | Pinocchio + Crocoddyl |
| **Control frequency** | 1 kHz (torque) / 50–100 Hz (MPC) |
| **Robot model** | TALOS or custom URDF humanoid |

### 9.2 Metrics

| Metric | Definition | Target |
|--------|-----------|--------|
| **CoM tracking error** | RMSE of CoM trajectory vs. reference | < X cm |
| **Contact force compliance** | Fraction of time within friction cone | > 95% |
| **Task success rate** | % of trials completing the target task | > Y% |
| **QP solve time** | Mean / max per control cycle | < 1 ms |
| **Perturbation recovery** | Recovery rate after impulse disturbance | To define |
| **End-effector error** | Position RMSE during manipulation | < Z cm |

### 9.3 Baselines

| Baseline | Description |
|----------|-------------|
| **Null-space task-priority WBC** | Classic Sentis & Khatib formulation |
| **PD joint control** | Decentralized joint-level control, no whole-body coordination |
| **RL policy** | Pre-trained locomotion policy (e.g., Unitree H1 RL) as upper-bound reference |

### 9.4 Test Scenarios

| Scenario | Description | Primary Metric |
|----------|-------------|---------------|
| **S1: Static balancing** | Stand still under external pushes | CoM error, recovery time |
| **S2: Flat-ground walking** | Walk 3 m on flat terrain | Task success, CoM tracking |
| **S3: Stair climbing** | Ascend/descend 3 steps | Foot clearance, contact forces |
| **S4: Loco-manipulation** | Walk while carrying / reaching for object | End-effector error + CoM tracking |
| **S5: Perturbation robustness** | Random impulse disturbances during walking | Recovery rate |

---

## 10. 🧠 Key Definitions (Canonical — Do Not Redefine)

| Term | Definition |
|------|-----------|
| **Whole-Body Control (WBC)** | A control framework that simultaneously considers all degrees of freedom of a robot to execute multiple tasks while satisfying physical constraints (dynamics, contact, joint limits) |
| **Task-Priority Control** | A WBC method that resolves task conflicts via null-space projections, assigning strict hierarchical priority to tasks |
| **QP-based WBC** | Formulates WBC as a Quadratic Program minimizing a weighted task-space cost subject to rigid-body dynamics and contact constraints |
| **Centroidal Dynamics** | A reduced model describing the evolution of the robot's Center of Mass (CoM) and Angular Momentum — sufficient for locomotion planning |
| **Floating Base** | A robot model where the base body has 6 unactuated DOF relative to the world frame; characteristic of legged robots |
| **Contact Wrench** | The 6D vector (force + torque) exerted by the environment on the robot at a contact point |
| **Friction Cone** | The set of physically admissible contact forces assuming Coulomb friction; a key constraint in WBC |
| **Loco-manipulation** | The simultaneous execution of locomotion (moving the base) and manipulation (moving an end-effector) |
| **Sim-to-Real Gap** | The performance discrepancy observed when a controller validated in simulation is deployed on physical hardware |
| **MPC (Model Predictive Control)** | An optimization-based control strategy that solves a finite-horizon optimal control problem at each time step using a predictive model |

---

## 11. 🚫 Rejected Approaches

| Approach | Reason for Rejection |
|----------|---------------------|
| **End-to-end RL as main contribution** | Black-box, no constraint guarantees, out of scope for this thesis; retained as baseline only |
| **Hardware experiments** | Out of scope — simulation-only thesis |
| **Wheeled/tracked humanoids** | Not consistent with bipedal humanoid focus |
| **SLAM-centric perception** | Not relevant to WBC-focused contribution; environment assumed known |

---

## 12. ✍️ Current Thesis Structure

| Chapter | Title | Status | Key Argument |
|---------|-------|--------|-------------|
| **1** | Introduction | 🔲 Not started | Motivate WBC for humanoids; state research question |
| **2** | State of the Art — Whole-Body Control for Humanoids | 🔲 Not started | Synthesize and critically position existing WBC methods |
| **3** | Methodology | 🔲 Not started | Formalize proposed WBC framework; justify design choices |
| **4** | Simulation Framework | 🔲 Not started | Describe simulator, robot model, implementation |
| **5** | Experiments and Results | 🔲 Not started | Present metrics across S1–S5 scenarios |
| **6** | Discussion | 🔲 Not started | Interpret results, compare baselines, acknowledge limits |
| **7** | Conclusion and Future Work | 🔲 Not started | Summarize contribution, open research directions |

---

## 13. 🔗 Logical Flow (MANDATORY — Every Section Must Respect This Chain)

```
Problem (humanoid WBC remains unsolved / underspecified)
    ↓
Gap (no standard framework coupling planning + QP WBC for simultaneous loco-manipulation)
    ↓
Proposed Solution (hierarchical WBC: centroidal MPC + QP execution layer)
    ↓
Validation (simulation across S1–S5 scenarios, compared to baselines)
    ↓
Contribution (reproducible framework + evaluation protocol + critical analysis)
```

> Every claim, section, and argument must be traceable back to this chain. If it cannot, it should be removed or reframed.

---

## 14. 💡 Key Insights So Far

- **Insight 1:** The split between planning (centroidal / simplified models) and execution (full-body QP) is the dominant architectural pattern in model-based WBC — understanding this split is essential to positioning the thesis.
- **Insight 2:** The lack of standardized evaluation benchmarks for humanoid WBC is itself a contribution opportunity — proposing a reproducible evaluation protocol has standalone value.
- **Insight 3:** Simulation fidelity (contact model choice) will be the most debatable methodological decision — must be justified explicitly in Chapter 3 and acknowledged in Chapter 6.

---

## 15. ⚠️ Known Weaknesses (Self-Critique)

- **Weakness 1:** Research question not yet fully formalized — all downstream content may need revision once finalized.
- **Weakness 2:** Simulator and robot model not yet chosen — blocks Chapters 3 and 4.
- **Weakness 3:** Literature review not yet populated — taxonomy exists but has no content.
- **Weakness 4:** Proposed contribution may overlap with existing frameworks (Pinocchio/Crocoddyl ecosystem); novelty must be clearly delineated.
- **Weakness 5:** Metrics (X, Y, Z values in Section 9.2) are placeholder — must be grounded in literature benchmarks.

---

## 16. 🔄 Pending Questions

| ID | Question | Priority | Blocking? |
|----|----------|----------|---------|
| PQ1 | What is the exact humanoid URDF model to use? (TALOS, Unitree H1, custom?) | High | Yes — blocks Ch. 3, 4 |
| PQ2 | Which simulator: MuJoCo or Isaac Sim? | High | Yes — blocks Ch. 4 |
| PQ3 | Is the WBC formulation QP-only, or QP + MPC in hierarchy? | High | Yes — blocks Ch. 3 |
| PQ4 | What is the exact novel contribution vs. Crocoddyl/existing frameworks? | High | Yes — blocks Ch. 1 |
| PQ5 | Are there supervisor-defined constraints on the methodology? | Medium | Potentially |
| PQ6 | What are the target metric thresholds (CoM error < ?, success rate > ?)? | Medium | Blocks Ch. 9 |

---

## 17. 📌 Writing Consistency Rules

1. Use **"Whole-Body Control (WBC)"** as the primary term — never "whole body control" (no hyphen) or "wholebody control"
2. Use **"humanoid robot"** not "humanoid robotic system" or "bipedal robot" (unless specifically relevant)
3. **"QP"** = Quadratic Program/Programming — always spell out on first use per chapter
4. Avoid **"clearly", "obviously", "simply"** — academic tone requires demonstration, not assertion
5. Every claim must be followed by a citation — no uncited factual statements
6. All equations must use consistent notation (define in Section 3.1 and reuse everywhere)
7. **"Simulation"** not "simulations" when referring to the general method
8. Each chapter section must end with a **mini critical insight paragraph** synthesizing limitations or open questions

---

## 18. 🧪 Reviewer Mode — Anticipated Criticisms

| Criticism | Pre-emptive Response |
|-----------|---------------------|
| *"What is the novelty vs. Crocoddyl / existing WBC stacks?"* | Must clearly delineate contribution: new framework, new evaluation protocol, or novel coupling strategy — not just re-implementation |
| *"Simulation-only is insufficient validation"* | Acknowledge as limitation; justify with scope and reproducibility arguments; cite sim-validated WBC precedents |
| *"Evaluation scenarios are too simple"* | Include perturbation test + loco-manipulation; compare against literature baselines on same metrics |
| *"Metrics are arbitrary"* | Ground all thresholds in cited literature benchmarks |
| *"RL baselines would outperform your WBC"* | Frame as expected: RL trades constraint guarantees for agility — different design goals |
| *"Contact model choice is arbitrary"* | Dedicate a sub-section in Ch. 3 to justifying the choice; run sensitivity analysis if time permits |

---

## 19. 📚 Source Integrity Tracking

| BibKey | Title (short) | Venue | Year | Sections Used | Verified | Notes |
|--------|-------------|-------|------|--------------|---------|-------|
| Sentis2005 | Synthesis of whole-body behaviors | IJRR | 2005 | Ch. 2, 3 | 🔲 | Foundational null-space WBC |
| Wensing2013 | Generation of dynamic humanoid behaviors | ICRA | 2013 | Ch. 2, 3 | 🔲 | QP-WBC reference |
| Koolen2016 | Design of a momentum-based control framework | IJHR | 2016 | Ch. 2, 3 | 🔲 | Atlas WBC |
| Caron2020 | Stair climbing via centroidal dynamics | IROS | 2020 | Ch. 2 | 🔲 | Centroidal MPC |
| Winkler2018 | Gait and trajectory optimization (TOWR) | RAL | 2018 | Ch. 2 | 🔲 | Planning layer reference |
| Pinocchio | Carpentier et al. — Pinocchio library | IROS | 2019 | Ch. 3, 4 | 🔲 | Dynamics library |
| Crocoddyl | Mastalli et al. — Crocoddyl | ICRA | 2020 | Ch. 3, 4 | 🔲 | Optimal control library |

> ⚠️ Only sources marked **✅ Verified** (fully read + cross-checked) may appear in the final manuscript.

---

## 20. 🔗 Version Control & GitHub

### 20.1 Repository
- **URL:** [https://github.com/David543M/humanoid-wbc-thesis](https://github.com/David543M/humanoid-wbc-thesis)
- **Visibility:** Public
- **Default branch:** `main`
- **Created:** 2026-04-22

### 20.2 Branching Strategy

| Branch pattern | Purpose |
|---------------|---------|
| `main` | Stable state — always reflects current thesis |
| `chapter/N-title` | One branch per chapter draft |
| `sim/feature-name` | Simulation development |
| `fix/description` | Corrections and revisions |

### 20.3 Commit Conventions

| Prefix | Use |
|--------|-----|
| `[chapter-N]` | Chapter writing or update |
| `[sim]` | Simulation development |
| `[lit]` | Literature / research update |
| `[mem]` | `master_memory.md` update |
| `[fix]` | Correction or revision |

### 20.4 Token Security Rule
- **Never hardcode the GitHub PAT in any versioned file**
- Always use: `$env:GITHUB_TOKEN = "..."` (PowerShell) or `export GITHUB_TOKEN=...` (bash)
- The `setup_github_repo.py` script reads from `os.environ.get("GITHUB_TOKEN")`

---

## 21. 📁 File Structure Reference

```
IRP/
├── 00_problem_definition/     ← research_question.md, scope.md, hypotheses.md
├── 01_prompts/                ← AI query prompts (perplexity, gemini, claude, council, adversarial)
├── 02_raw_research/           ← Unprocessed literature notes
├── 03_processed/              ← taxonomy.md, summaries/, knowledge_graph/
├── 04_council_analysis/       ← contradictions.md, consensus.md, gaps.md, confidence_scores.md
├── 05_targeted_research/      ← refined_sources.md, gap_filling_reports/
├── 06_writing/                ← outline.md, draft_v1.md, draft_v2.md, final_review.md, chapters/
│                                 overleaf_template_guide.md ← NOUVEAU — doc du template LaTeX
├── 07_review/                 ← adversarial_review.md, supervisor_feedback.md, revisions.md
├── 08_references/             ← validated_sources.md, zotero_exports/
├── 09_appendices/             ← code/, figures/, datasets/
├── 10_simulation/             ← Simulation setup and results
├── 11_experiments/            ← Experiment logs and outputs
├── 12_meetings/               ← Supervisor meeting notes
├── 13_logbook/                ← Daily research log
├── humanoid_wbc_thesis_overleaf.zip  ← Template LaTeX Overleaf (prêt à importer)
└── master_memory.md           ← THIS FILE
```

---

## 21. 🚀 Next Actions

| Priority | Task | Blocking For |
|----------|------|-------------|
| 🔴 **P1** | Finalize and formalize the Research Question (PQ4) | Everything |
| 🔴 **P1** | Choose simulator (MuJoCo vs. Isaac Sim) and robot model (PQ1, PQ2) | Ch. 3, 4 |
| 🔴 **P1** | Delineate novelty vs. Crocoddyl/existing WBC frameworks | Ch. 1 intro |
| 🟠 **P2** | Populate literature taxonomy (03_processed/taxonomy.md) | Ch. 2 |
| 🟠 **P2** | Run initial literature council analysis and fill gaps.md | Ch. 2 |
| 🟡 **P3** | Define metric thresholds from literature baselines | Ch. 5 |
| 🟡 **P3** | Draft Chapter 1 — Introduction | Thesis draft |
| 🟢 **P4** | Set up simulation environment and test URDF loading | Ch. 4 implementation |

---

## 23. 🗓️ Session Log (Append After Each Work Session)

| Date | Session Focus | Key Decisions Made | Files Modified |
|------|--------------|-------------------|----------------|
| 2026-04-22 | Master memory file created | Template finalized; thesis structure confirmed | master_memory.md (created) |
| 2026-04-22 | GitHub repository initialized | Repo created at github.com/David543M/humanoid-wbc-thesis; full workspace pushed to main; token security rule established | README.md (enriched), master_memory.md (§20 added), setup_github_repo.py (token removed) |
| 2026-04-22 | Overleaf LaTeX template created | Template complet 7 chapitres (0 erreur, 41 pages) — notation canonique, environnements criticalinsight/hypothesis/researchgap, 9 sources BibTeX, structure liée à master_memory | humanoid_wbc_thesis_overleaf.zip (racine), 06_writing/overleaf_template_guide.md (nouveau), master_memory.md (§21 file structure mis à jour) |
| 2026-04-22 | WBC Council rerun (literature-review framing) — integration deliverable produced | Synthesized review (SLR) confirmed as Chapter 2 scaffold; integration plan committed; council recommends closing PQ1–PQ4 with defaults (TALOS primary, MuJoCo primary + Drake secondary, weighted single-QP + centroidal MPC, evaluation-protocol reframed as contribution); contact-consistency discussion reassigned to Ch. 4 / Ch. 6; publication-positioning framing deprecated as rerun scope change | 04_council_analysis/WBC_Council_2026-04-22/04_Thesis_Chapter_Integration_and_Recommendations.md (created); 04_Publication_Positioning_and_Recommendations.md (now legacy) |

---

*This file is the single source of truth. When in doubt, defer to it. When updating it, update all linked files for consistency.*
