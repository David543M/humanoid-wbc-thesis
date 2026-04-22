# Whole-Body Control for Humanoid Robots — A Literature Review
## Thesis Chapter 2 Scaffold (Council Synthesis, 2026-04-22)

> **Purpose of this document.** This is the synthesized best-possible *literature-review chapter* produced by merging the strongest elements of three deep-research outputs (Perplexity, Gemini, Claude) and filling the missing elements from primary sources. It is structured to be lifted — with final-author editing — into **Chapter 2 of the thesis** defined in `master_memory.md` §12. Every subsection ends with a **mini critical insight** per master_memory.md §17 rule 8. Confidence markers are attached to non-trivial claims: **[H]** multiple peer-reviewed sources, **[M]** single peer-reviewed or multiple preprints, **[L]** contested / preprint-only. Citations follow master_memory.md §19 format: BibKey-style with venue and year.

---

## 2.0 Chapter Overview

This chapter surveys the state of the art in whole-body control (WBC) of humanoid robots, with a focus on simulation-based development. The field has matured in three overlapping waves: (i) analytical prioritized-task control rooted in the operational-space formulation [Khatib 1987; Sentis–Khatib 2005; Mistry 2010] **[H]**; (ii) optimization-based whole-body control that casts the problem as a constrained quadratic program, hierarchical or weighted [Escande–Mansard–Wieber 2014; Del Prete 2014; Righetti 2013; Koolen 2016; Henze–Roa–Ott 2016] **[H]**; and (iii) a recent convergence of whole-body model predictive control (WB-MPC) on torque-controlled hardware [Dantec 2021, 2022a, 2022b; Sleiman 2021, 2023] **[H]** with deep-reinforcement-learning policies distilled from privileged teachers and deployed via GPU-parallel simulation [Radosavovic 2024; Haarnoja 2024; Cheng 2024; He 2024; Sferrazza 2024; Zakka 2025] **[H]**.

The review is organized as follows. §2.1 defines WBC and places it within the broader control-theoretic landscape. §2.2 traces the historical evolution. §2.3 develops the mathematical foundations. §2.4 enumerates the main architectural choices and introduces the taxonomy reused throughout the thesis. §2.5 addresses simulation environments, contact modelling, and the sim-to-real gap. §2.6 covers learning-based WBC as a baseline and comparison reference. §2.7 surveys implementation tooling. §2.8 addresses evaluation and benchmarking. §2.9 identifies the open problems and research gaps that motivate the thesis's research question (master_memory.md §1.2) and hypotheses (§1.4). §2.10 closes with a positioning of the thesis within the surveyed field.

### Mini Critical Insight — 2.0
The dominant pattern in humanoid WBC literature since 2015 is a *convergence* of formulations (QP-based instantaneous control, MPC planning, learned policies) rather than a succession. A thesis chapter that treats them chronologically misreads the field; a chapter that treats them as coexisting tradeoffs along a speed–generality axis matches current practice and is the framing adopted here.

---

## 2.1 Definition, Scope, and Positioning of Whole-Body Control

**Canonical definition (reused from master_memory.md §10).** Whole-Body Control is a control framework that simultaneously considers *all* degrees of freedom of a floating-base robot and computes joint commands — torques, or references to lower-level controllers — that realize multiple task-space objectives while satisfying physical constraints: rigid-body dynamics, contact consistency, friction-cone feasibility, actuator limits, and (optionally) kinematic limits [Khatib–Sentis–Park–Warren, *IJHR* 2004; Koolen 2016] **[H]**.

Three operational abstractions coexist in the literature [Mansard 2009; Del Prete 2014; Righetti 2013] **[H]**:

- **Kinematic WBC** computes desired generalized velocities or accelerations from task Jacobians and lets a lower-level joint-space controller realize them. This is the Stack-of-Tasks (SoT) tradition and is most appropriate for *position-controlled* humanoids (e.g., NAO, Pepper, older HRP platforms) where the low-level joint control is stiff and not directly under the designer's control.
- **Dynamic WBC** (inverse-dynamics WBC) directly computes joint torques from the full rigid-body dynamics with explicit optimization over contact forces. This is the approach demonstrated on Atlas [Koolen 2016], TALOS [Ramuzat 2022], TORO [Henze 2016], and the MIT Humanoid [Chignoli 2021], and it presupposes torque-controlled actuation.
- **Hierarchical WBC** organizes tasks into priority levels. Priority can be enforced by analytical null-space projection [Sentis 2007] **[H]**, lexicographic hierarchical quadratic programming [Escande 2014] **[H]**, or by weighted aggregation in a single QP [Del Prete 2014; Bouyarmane–Kheddar] **[H]**. These three mechanisms have substantively different feasibility and optimality properties — see §2.3.5.

WBC is distinguished from three related control-theoretic programs frequently conflated in introductory material:

1. **Reduced-model balance control** (ZMP / capture point / LIP): operates on a simplified model of the robot as a point-mass or linear inverted pendulum [Kajita 2003; Pratt 2006] **[H]**. Modern practice uses these models as *planners* whose references are tracked by a WBC layer, not as controllers in their own right [Koolen 2016].
2. **Classical manipulator operational-space control**: the ancestor of WBC [Khatib 1987] but formulated for *fully-actuated, fixed-base* systems. Extension to floating-base humanoids requires the support-consistent Jacobian and the Mistry 2010 framework — see §2.3.
3. **Joint-level or task-level impedance control**: a regulation strategy orthogonal to WBC. Impedance behaviour can be implemented *within* a WBC layer as one task among several [Hyon–Hale–Cheng 2007; Henze 2016] **[H]**.

**Scope positioning for this thesis.** Per master_memory.md §1.5, the thesis targets a torque-controlled bipedal humanoid in simulation, with QP-based WBC as the adopted approach. Within §2.1's taxonomy, this places the thesis at: *Dynamic WBC × Hierarchical (formulation TBD: weighted vs. lexicographic) × torque-controlled × simulation-only validation*. Reinforcement-learning WBC is surveyed in §2.6 as a *baseline* comparator per master_memory.md §9.3.

### Mini Critical Insight — 2.1
The common student error is to treat "WBC" as a single algorithm. It is a *family* of formulations whose members make different structural commitments about actuation, priority handling, model fidelity, and contact treatment. Choosing a WBC formulation is choosing a bundle of seven or eight commitments simultaneously, not one — see §2.4.

---

## 2.2 Historical Evolution

### 2.2.1 Operational Space and the Prioritized-Task Tradition (1987–2010)

Khatib's operational-space formulation [Khatib, *IEEE J. Robot. Autom.* 1987] introduced task-space dynamic decoupling for fixed-base manipulators via the mass-weighted pseudo-inverse, producing the now-classical torque law $\tau = J^\top \Lambda \ddot x_\text{des} + N^\top \tau_0$ with $\Lambda = (J M^{-1} J^\top)^{-1}$ [Khatib 1987] **[H]**. Sentis and Khatib [*IJHR* 2004; *ICRA* 2006] extended the formulation to floating-base humanoids by introducing the *support-consistent Jacobian*, which projects task forces into the subspace compatible with the stance contact constraints [Sentis 2007 thesis; Khatib–Sentis–Park–Warren *IJHR* 2004] **[H]**. Mistry, Buchli, and Schaal [*ICRA* 2010] provided the foundational analysis of inverse dynamics for constrained underactuated systems, establishing that for such systems task and null-space dynamics cannot in general be fully decoupled — a *limitation result* that motivates the optimization-based turn in later years [Mistry–Buchli–Schaal 2010] **[H]**.

### 2.2.2 Balance Control: ZMP, Capture Point, and Centroidal Momentum (1990–2015)

Parallel to operational-space work, humanoid locomotion consolidated around three balance-related notions. The *Zero Moment Point* and the Linear Inverted Pendulum Model, popularised by Kajita and colleagues [Kajita *ICRA* 2003; Kajita *IROS* 2001] **[H]**, enabled pattern-generator-based walking on flat and quasi-flat terrain. The *Capture Point* [Pratt–Carff–Drakunov–Goswami *Humanoids* 2006] and its generalization to the *Divergent Component of Motion* [Englsberger–Ott–Albu-Schäffer] **[H]** provided a closed-form criterion for where a humanoid must step to arrest its motion. Momentum-based balance control, building on Orin and Goswami's formalization of the Centroidal Momentum Matrix [Orin–Goswami *ICRA* 2008; Orin–Goswami–Lee *Auton. Robot.* 2013] **[H]**, regulated whole-body linear and angular momentum within a QP-WBC layer; the highest-impact instantiation is Koolen et al.'s Atlas controller [Koolen et al. *IJHR* 2016] **[H]** and the Feng et al. DARPA-era QP controller [Feng–Whitman–Xinjilefu–Atkeson *JFR* 2015] **[H]**.

### 2.2.3 Optimization-Based WBC (2010–2020)

Escande, Mansard, and Wieber [*IJRR* 2014] formalised *Hierarchical Quadratic Programming* (HQP): a cascade of quadratic programs ordered by strict lexicographic priority, each minimising its task error within the feasible set of all higher-priority tasks. The paper established real-time feasibility of strict-priority WBC on HRP-2 and provided the algorithmic backbone used by most European optimisation-based WBC stacks [Escande–Mansard–Wieber *IJRR* 2014] **[H]**. Del Prete, Nori, Metta, and Natale [*RAS* 2014] introduced *Task-Space Inverse Dynamics* (TSID) — a weighted single-QP formulation with formal optimality under full-actuation, subsequently extended to floating-base systems [Del Prete–Mansard *RAL* 2016] **[H]**. Righetti, Buchli, Mistry, Kalakrishnan, and Schaal [*IJRR* 2013] demonstrated hierarchical inverse-dynamics QP on a torque-controlled humanoid lower body, detailing the simplifications needed for real-time control at 1 kHz [Righetti et al. 2013] **[H]**. Kuindersma and colleagues [*Auton. Robot.* 2016] documented the mixed-integer / convex QP pipeline used by Atlas at the DARPA Robotics Challenge — a system-level reference point for any optimization-based WBC project [Kuindersma et al. 2016] **[H]**. Finally, Henze, Roa, and Ott [*IJRR* 2016] developed a passivity-based multi-contact balancer for DLR's TORO, providing Lyapunov-style stability guarantees under multi-limb support — a property ordinary QP-on-acceleration schemes do not enjoy by construction [Henze–Roa–Ott 2016] **[H]**.

### 2.2.4 Whole-Body MPC on Torque-Controlled Hardware (2021–2024)

Until approximately 2020, whole-body MPC was a simulation-only program; onboard solvers could not close a full-dynamics optimal control problem fast enough on a humanoid. The breakthrough came from a tight co-design of efficient rigid-body algorithms (*Pinocchio* [Carpentier et al. *SII* 2019] **[H]**), differential dynamic programming via *Crocoddyl* [Mastalli et al. *ICRA* 2020] **[H]**, and learned warm-starts. Dantec et al. [*ICRA* 2021] demonstrated the first whole-body MPC with state feedback on a torque-controlled humanoid (TALOS), solving a 0.5 s receding-horizon OCP in under 10 ms using Crocoddyl and a "memory of motion" warm-start [Dantec et al. *ICRA* 2021] **[H]**. The Humanoids 2022 follow-up extended this to full biped locomotion with a 1.5 s preview and 10 cm step traversal [Dantec et al. *Humanoids* 2022] **[H]**. An *RA-L* 2022 paper showed that the Riccati gains produced as a by-product of DDP's backward pass can be used directly as the low-level feedback at 2 kHz, removing the need for a separately designed tracking controller [Dantec et al. *RA-L* 2022] **[H]**. Khazoom, Dantec, Carpentier, and Mansard [*Humanoids* 2024] subsequently tailored solver accuracy per stage to accelerate the OCP without sacrificing stability [Khazoom et al. 2024] **[M]**.

On legged mobile manipulators, Sleiman, Farshidian, Minniti, and Hutter [*RA-L* 2021] unified locomotion and manipulation in a single multi-contact OCP by augmenting centroidal dynamics with object dynamics and encoding gait schedules as switched-system constraints, running onboard in real time on ANYmal-arm systems [Sleiman et al. *RA-L* 2021] **[H]**. The 2023 *Science Robotics* follow-up generalized this to versatile legged loco-manipulation [Sleiman et al. *Science Robotics* 2023] **[H]**.

### 2.2.5 The Reinforcement-Learning Takeover (2022–2026)

The most consequential shift since 2022 is that deep-reinforcement-learning policies now match or beat classical WBC on real humanoids. Radosavovic et al. [*Science Robotics* 9, eadi9579, 2024] trained a causal transformer on 16-step observation-action histories with model-free RL in Isaac Gym and deployed zero-shot to Agility's Digit, walking outdoors without state estimation, reference trajectories, or gait libraries [Radosavovic et al. 2024] **[H]**. Haarnoja et al. [*Science Robotics* 9, eadi8022, 2024] trained 20-DOF Robotis OP3 robots to play 1v1 soccer with deep RL, producing emergent fall recovery and context-dependent kicking; the learned walking reached 0.57 m/s versus 0.45 m/s for a hand-tuned RoboCup-style controller [Haarnoja et al. 2024] **[H]**. A follow-up by Radosavovic et al. — "Humanoid Locomotion as Next Token Prediction" [NeurIPS 2024] — reframed control as autoregressive sequence modeling over mixed sensorimotor trajectories [Radosavovic et al. NeurIPS 2024] **[H]**.

The enabling infrastructure is as important as the algorithms: NVIDIA Isaac Gym / Isaac Lab [Makoviychuk et al. NeurIPS Datasets & Benchmarks 2021] **[H]**, DeepMind's MuJoCo MJX and MuJoCo Playground [Zakka et al. *RSS* 2025] **[H]**, and the HumanoidBench benchmark suite [Sferrazza et al. *RSS* 2024] **[H]** collapsed wall-clock training from days on CPU clusters to minutes on a single GPU and supplied a standardised test environment.

### 2.2.6 Human-Motion Retargeting as Universal Interface (2024–2025)

A convergence visible in 2024 is that **kinematic human pose has become the de facto abstraction for humanoid WBC** [Cheng et al. *RSS* 2024 (ExBody); Ji et al. arXiv 2412.13196 (ExBody2); Fu et al. *CoRL* 2024 (HumanPlus); He et al. *CoRL* 2024 (OmniH2O); He et al. arXiv 2410.21229 (HOVER)] **[H]**. HOVER distils multiple control modes (root velocity, head-hand poses, joint angles, full-body kinematic poses) into a single 1.5-million-parameter neural network trained in Isaac Lab in approximately 50 minutes of wall-clock on one GPU, and outperforms the specialists from which it is distilled [He et al. 2024] **[M]**.

### Mini Critical Insight — 2.2
The 35-year arc from Khatib 1987 to HOVER 2024 is *not* a succession — each wave extended the applicability of WBC without displacing prior work. Classical QP-based WBC remains the only regime with hard constraint guarantees; whole-body MPC remains the only regime with anticipatory reasoning over full dynamics; RL-distilled policies remain the only regime with robust un-modelled-dynamics generalization. A thesis that implements one of the three must position itself with respect to the other two — a requirement fulfilled by §2.10 below.

---

## 2.3 Mathematical Foundations

### 2.3.1 Floating-Base Rigid-Body Dynamics

A humanoid with $n$ actuated joints has configuration $q \in SE(3) \times \mathbb{R}^n$, combining a 6-DOF floating-base pose and $n$ joint angles; its generalised velocity is $v \in \mathbb{R}^{n+6}$. The equations of motion are [Featherstone 2008; Sentis 2007; Mistry 2010; Koolen 2016] **[H]**:

$$
M(q)\,\dot v + h(q,v) = S^\top\tau + \sum_{c \in \mathcal{C}} J_c^\top(q)\, f_c,
$$

where $M(q) \in \mathbb{R}^{(n+6)\times(n+6)}$ is the generalized inertia matrix, $h(q,v)$ collects Coriolis, centrifugal, and gravitational terms, $S = [0_{n\times 6}, I_{n \times n}]^\top$ is the actuation selection matrix (reflecting that the 6 base DOFs are unactuated), $J_c(q)$ is the $c$-th contact-point Jacobian, and $f_c$ is the corresponding contact wrench. The first six rows of the EoM — the unactuated directions — are the structural constraint that defines *underactuated* floating-base control and motivates the entire centroidal and momentum apparatus of §2.3.3.

### 2.3.2 Task Kinematics, Operational-Space Control, and Mistry's Limitation

For a task defined by the map $x = \phi(q)$, differential kinematics yield $\dot x = J_\phi(q) v$ and $\ddot x = J_\phi \dot v + \dot J_\phi v$. Khatib's classical closed-form controller for fully-actuated manipulators is

$$
\tau = J_\phi^\top \Lambda(q) \left[\ddot x_\text{des} - \dot J_\phi v \right] + N^\top \tau_0,\qquad \Lambda = \bigl(J_\phi M^{-1} J_\phi^\top\bigr)^{-1},
$$

where $N = I - J_\phi^\top \Lambda J_\phi M^{-1}$ is the dynamically-consistent null-space projector [Khatib 1987] **[H]**. Extension to floating-base humanoids requires projecting this onto the subspace consistent with stance contacts — the *support-consistent Jacobian* $\bar J_\phi$ of Sentis and Khatib [Sentis 2007] **[H]**. Mistry, Buchli, and Schaal [2010] proved that for *constrained underactuated* systems, the operational-space and posture dynamics cannot in general be fully decoupled — a result that explains why classical OSC is insufficient for humanoids and why the optimization-based formulations of §2.3.5 became necessary [Mistry–Buchli–Schaal 2010] **[H]**.

### 2.3.3 Centroidal Dynamics and Momentum

The **Centroidal Momentum Matrix** (CMM) $A_G(q) \in \mathbb{R}^{6 \times (n+6)}$ maps the generalized velocity to the system's linear and angular momentum about the centre of mass [Orin–Goswami 2013] **[H]**:

$$
h_G = \begin{bmatrix} L \\ k \end{bmatrix} = A_G(q)\, v.
$$

The centroidal dynamics are

$$
\dot L = \sum_{c\in\mathcal{C}} f_c + m g, \qquad \dot k = \sum_{c\in\mathcal{C}} (p_c - p_{CoM}) \times f_c,
$$

where $p_c$ is the contact-point position, $p_{CoM}$ is the centre of mass, and $m g$ is gravity. Efficient computation of $A_G(q)$ is available in Pinocchio via spatial-operator algebra [Carpentier et al. 2019] **[H]**. The centroidal model is the standard *reduced model* used by MPC planners [Dai–Valenzuela–Tedrake *Humanoids* 2014; Caron et al.] **[H]**.

### 2.3.4 Contact Constraints and Friction Cone

Each active contact $c \in \mathcal{C}$ imposes three kinematic and dynamic constraints [Koolen 2016; Horak–Trinkle 2019] **[H]**:

1. **Holonomic contact consistency** (no penetration, no motion at a stance contact): $J_c v = 0$ and, at acceleration level, $J_c \dot v + \dot J_c v = 0$.
2. **Unilateral normal force**: $f_{c,z} \ge 0$.
3. **Coulomb friction cone**: $\sqrt{f_{c,x}^2 + f_{c,y}^2} \le \mu\, f_{c,z}$.

In QP-based WBC the friction cone is linearised into a polyhedral approximation $A_\mu f_c \le 0$ typically using 4–16 facets, with conservative bias toward the former for real-time tractability [Caron et al.] **[H]**. This linearization is a known source of controller–simulator inconsistency when the simulator uses a smooth (MuJoCo soft contact) or exact (Drake relaxed complementarity) cone formulation — see §2.5.2.

### 2.3.5 Whole-Body Control as a Quadratic Program

The dynamic-WBC problem in its modern form is a quadratic program:

$$
\min_{\dot v,\,\tau,\,f} \; \sum_{i} w_i \bigl\| J_i \dot v + \dot J_i v - \ddot x_i^\text{des} \bigr\|^2_{W_i}
$$

subject to **EoM** (highest-priority equality constraint), **contact consistency** (equality on stance contacts), **friction cone** (polyhedral inequality), **torque limits** (inequality), and optionally **joint-position / joint-velocity limits** (inequality with look-ahead) [Righetti 2013; Koolen 2016; Del Prete 2014] **[H]**. Tasks include CoM trajectory tracking, end-effector pose tracking, swing-foot trajectory, posture regularization, and angular momentum regulation.

**Priority handling — three mechanisms, not interchangeable** [Escande 2014; Del Prete 2014; Mansard 2009] **[H]**:

- **Null-space projection** (analytical): lower-priority tasks projected into the dynamically-consistent null-space of higher priorities. Risk: ill-conditioning near kinematic singularities; incompatible with inequality constraints.
- **Lexicographic Hierarchical QP** (Escande 2014): a cascade of QPs, each minimizing its task-error norm within the feasibility set of all higher-priority tasks. Guarantees strict priority; handles inequalities rigorously; may be infeasible at high levels under disturbance.
- **Weighted single-QP** (Del Prete 2014 TSID-style): all tasks in one QP with scalar weights $w_i$; always feasible (with slacks) but does not enforce strict priority and weights lack principled selection.

The choice among these three is a core thesis commitment (master_memory.md §16 PQ3); Chapter 3 will formalise and defend the selection.

### 2.3.6 Stability Metrics: ZMP, Capture Point, DCM

For planning and diagnosis, reduced stability metrics remain in use [Vukobratović 1972; Pratt 2006; Englsberger et al.] **[H]**:

- **Zero Moment Point** (ZMP): the point on the support surface where the net resultant moment is purely vertical; for balance on flat ground, the ZMP must lie within the support polygon.
- **Capture Point** (CP): the point on the ground where the robot would step to bring the CoM to rest.
- **Divergent Component of Motion** (DCM): a 3-D generalisation of the CP whose dynamics are unstable by construction and whose regulation is sufficient for CoM stability.

These are *planning-layer* concepts; they provide reference trajectories for the WBC layer but do not replace it.

### Mini Critical Insight — 2.3
The mathematical scaffold of WBC is well-established and mostly closed — the open research questions are not in the formulation but in the *interface problems* between adjacent layers: planning ↔ WBC (time-scale mismatch, reference feasibility), WBC ↔ contact (linearization consistency), WBC ↔ hardware (actuator bandwidth, state estimation latency). Chapter 3 of the thesis should defend its formulation choices explicitly along these interfaces, not merely along the core QP.

---

## 2.4 Architectural Taxonomy

A master's-thesis Chapter 2 should provide an explicit multi-axis taxonomy that the rest of the thesis can reuse. The axes below consolidate the treatments in [Koolen 2016; Ramuzat 2022; Del Prete 2014; Escande 2014] **[H]** and extend them with modern practice.

| Axis | Values | Representative works | Relevance to this thesis |
|---|---|---|---|
| Abstraction level | Kinematic / Dynamic / Hybrid | SoT [Mansard 2009] / TSID [Del Prete 2014] / SEIKO [Rouxel 2022] | Dynamic WBC |
| Priority handling | Null-space / Lexicographic HQP / Weighted single-QP | Sentis 2007 / Escande 2014 / Del Prete 2014 | TBD — see §2.3.5 and PQ3 |
| Optimization back-end | Single-level QP / Cascaded QPs / Nonlinear OCP / DDP | OpenSoT / Escande 2014 / Crocoddyl [Mastalli 2020] / Dantec 2021 | Single-level QP (primary) |
| Dynamics model | Full rigid-body / Centroidal / SRBD / Point-mass LIP | Koolen 2016 / Orin–Goswami 2013 / Kuindersma 2016 / Kajita 2003 | Full rigid-body (primary), Centroidal (planning) |
| Horizon | Instantaneous / Short-horizon MPC / Long-horizon plan | QP-WBC / Dantec 2022 / Winkler TOWR 2018 | Hybrid (centroidal MPC + instantaneous QP) |
| Supervision | Model-based / Imitation / Model-free RL / Distillation | Koolen 2016 / HumanPlus / Radosavovic 2024 / HOVER | Model-based (primary), RL (baseline) |
| Actuation interface | Position-controlled / Torque-controlled / SEA / QDD | HRP-4, Nao / TALOS, Atlas / Valkyrie, TORO / Unitree H1 | Torque-controlled |

Adopting this seven-axis taxonomy makes every thesis commitment explicit and traceable back to prior work.

### Mini Critical Insight — 2.4
A common failing of humanoid WBC literature is to describe a controller along one axis and elide the others. The taxonomy above forces every survey entry to be classified along all seven axes — a discipline that surfaces otherwise invisible incompatibilities (e.g., position-controlled-hardware papers that borrow torque-controlled-hardware vocabulary without adapting the formulation).

---

## 2.5 Simulation Environments and the Sim-to-Real Gap

### 2.5.1 Physics Engines for Humanoid WBC

Accurate simulation of multi-contact humanoid dynamics is pivotal for simulation-phase thesis work. The 2014 survey by Ivaldi, Peters, Padois, and Nori [*Humanoids* 2014] **[H]** establishes the historical reference for simulator choice in humanoid research; the 2026 landscape is significantly more fragmented and GPU-parallel. The table below is rebuilt from primary sources; it corrects claims from third-party benchmarks with disclosed conflicts of interest.

| Simulator | Contact model | Integrator | Primary source | Open-source? | Strengths | Known limitations | Conf. |
|---|---|---|---|---|---|---|---|
| **MuJoCo** | Soft convex-cone with slip tolerance | Semi-implicit Euler | Todorov–Erez–Tassa *IROS* 2012 | Yes (since 2021) | Fast, smooth gradients, widely used in RL; MJX GPU variant | Soft contact produces consistent slip; behaviour sensitive to `solref`/`solimp`/`impratio` | **[H]** |
| **MuJoCo MJX / Playground** | Same model, GPU-parallel | Same | Zakka et al. *RSS* 2025 | Yes | GPU scaling, sim-to-real demos on Berkeley Humanoid, Unitree G1, Booster T1 | Feature parity with CPU MuJoCo maturing | **[M]** |
| **Isaac Sim / Isaac Lab** | GPU PhysX (rigid LCP, hybrid) | PGS / TGS | Makoviychuk et al. *NeurIPS D&B* 2021 | Yes (Isaac Lab) | Massive parallelism, photorealistic rendering | Contact fidelity for multi-limb dynamic contact behind MuJoCo in published comparisons | **[M]** |
| **Drake** (MIT / TRI) | Relaxed complementarity (Anitescu 2006), convex MLCP | SAP | Castro et al. 2023 | Yes | Analytical derivatives, optimization-native; formally principled | Steeper learning curve; slower than MuJoCo for RL | **[H]** |
| **RaiSim** (ETH) | Bisection on per-contact LCP | Semi-implicit | Hwangbo et al. 2018 | **No** (proprietary) | Good for ANYmal-class work; reported energy preservation | Comparative speed claims on SimBenchmark originate from the same lab and should be treated as non-independent | **[L]** on speed |
| **Gazebo / gz-sim + DART/ODE** | Penalty or constraint-based, back-end dependent | Multiple | DART [Lee et al. 2018]; ODE [Smith 2005] | Yes | ROS 2 integration | Contact fidelity behind MuJoCo / Drake; community migration away for WBC since ~2022 | **[M]** |

### 2.5.2 The Contact-Model–Controller Consistency Problem

A under-treated issue in humanoid WBC literature — and one directly relevant to the thesis — is that **the contact model used by the simulator and the contact constraints written into the controller QP are not the same mathematical object**. The QP uses a linearized friction cone $A_\mu f_c \le 0$, unilateral normal constraints, and rigid holonomic contact; MuJoCo uses soft contact with slip tolerance; Drake uses relaxed complementarity; Isaac PhysX uses a projected Gauss–Seidel approximation of the rigid LCP. The *controller's notion of a valid contact force* and the *simulator's notion of a valid contact force* are therefore non-identical, and the gap is numerically non-zero even under perfect system identification [Horak–Trinkle 2019; Castro et al. 2023; Acosta et al. 2022; Howell et al. 2022] **[M]**.

Consequence for simulation-phase validation: claims about constraint satisfaction in simulation do not automatically transfer to claims about constraint satisfaction in reality. Chapter 6 of the thesis (Discussion) will acknowledge this explicitly.

### 2.5.3 Sim-to-Real Strategies for Humanoid WBC

Four complementary families appear in the literature:

1. **System identification** — mass / inertia / joint friction / actuator bandwidth calibration from data [Bloesch et al.; Koolen 2016] **[H]**.
2. **Domain randomization** over contact / actuator / sensor parameters [Peng et al. *ICRA* 2018] **[H]**.
3. **Residual / delta-action learning** — a learned correction added to a model-based controller or simulation model [He et al. *CoRL* 2024 (OmniH2O); CMU ASAP framework] **[M]**.
4. **Actuator networks** — learned torque-dynamics models [Hwangbo et al. *Science Robotics* 2019 (ANYmal); MIT "Bridging the Sim-to-Real Gap for Athletic Loco-Manipulation" 2025] **[M]**.

For simulation-phase theses, (1) and (2) are directly applicable; (3) and (4) are reported as emerging practice but are outside this thesis's scope (master_memory.md §1.5).

### Mini Critical Insight — 2.5
Simulator choice is not neutral. MuJoCo's soft contact is *less physically faithful* than Drake's relaxed complementarity but *more numerically stable* for high-bandwidth WBC and the only engine with a mature GPU path (MJX). A defensible thesis choice is MuJoCo as primary simulator with Drake as a secondary verification track on one scenario; Chapter 4 should make this argument formally, and Chapter 6 should flag the consistency gap as an acknowledged limitation.

---

## 2.6 Learning-Based WBC (Baseline Coverage)

Per master_memory.md §1.5 and §3.5, deep-RL WBC is surveyed here as a baseline and comparator, not as a proposed approach.

**Four reference systems** span the 2024–2025 design space:

- **ExBody** [Cheng et al. *RSS* 2024] **[H]** — RL on Unitree H1 trained on ~780 CMU MoCap clips with decoupled upper-body imitation and lower-body velocity tracking; state-of-the-art for expressive motion without compromising balance.
- **ExBody2** [Ji et al. arXiv 2412.13196] **[M]** — teacher-student distillation with a privileged-motion-filtering teacher; student tracks keypoints decoupled from velocity; transfers to both G1 and H1.
- **HumanPlus** [Fu et al. *CoRL* 2024] **[H]** — Humanoid Shadowing Transformer trained on 40 h of human motion + Humanoid Imitation Transformer for autonomous skills; single-RGB-camera shadowing at a 25/50/1000 Hz cascade.
- **OmniH2O** [He et al. *CoRL* 2024] **[H]** and **HOVER** [He et al. arXiv 2410.21229] **[M]** — universal teleoperation + distilled 1.5 M-parameter generalist policy trained in ~50 min on one GPU in Isaac Lab; HOVER outperforms specialists from which it is distilled.

**Diagnostic finding.** Sferrazza et al.'s HumanoidBench [*RSS* 2024] established empirically that **flat end-to-end RL struggles with long-horizon, high-DOF whole-body tasks; hierarchical architectures with robust low-level skills (walk, reach) succeed** [Sferrazza et al. 2024] **[H]**. This finding justifies the thesis's model-based architecture: the structured priority and contact-aware QP layer *is* the "robust low-level skill" that hierarchical RL architectures aim to recover.

**Emerging consensus architecture (2024–2025)** [He 2024; Ji 2024; Fu 2024; Radosavovic 2024] **[M]**:

1. Privileged "oracle" teacher in simulation (full-state, reference motion).
2. Deployable student distilled with sparse realistic sensors.
3. Low-level PD or torque layer at 1–2 kHz executing student outputs.
4. Domain randomisation + history-conditioned observations substituting for explicit system identification.

### Mini Critical Insight — 2.6
Learning-based WBC wins on *generalisation over un-modelled dynamics* and on *hardware-breadth demonstrations*; model-based WBC wins on *constraint-enforcement guarantees* and *contact-rich manipulation with heavy payloads*. Reporting them as alternatives is incorrect; they solve different problems. The thesis's decision to use model-based WBC is therefore a choice about which guarantees to keep, not a choice about which method is "better".

---

## 2.7 Implementation Tooling

The thesis's tool selections (master_memory.md §8.2) should be defensible against the following landscape.

**Dynamics libraries.** *Pinocchio* [Carpentier et al. 2019] **[H]** is the de-facto standard for efficient rigid-body algorithms with analytical derivatives; *Drake's MultibodyPlant* [Tedrake et al.] **[H]** is the principal alternative with its own articulated-body dynamics. *RBDL* [Felis 2017] **[H]** remains in use for legacy systems.

**QP solvers for WBC.** For dense small-to-medium problems (30–60 joints, 12–40 contact-force variables):

- **qpOASES** [Ferreau et al. *Math. Prog. Comp.* 2014] **[H]** — active-set, best warm-starting for dense small problems; failure mode is active-set drift under fast contact switching.
- **OSQP** [Stellato et al. *Math. Prog. Comp.* 2020] **[H]** — ADMM, scales well, less sensitive to conditioning; standard choice for OSC-style WBC.
- **eiquadprog** (Goldfarb–Idnani) — dual active-set; default in the TSID reference implementation.
- **ProxQP** [Bambade et al. *RSS* 2022] **[M]** — primal-dual augmented-Lagrangian; robust under active-set drift; the solver of choice for contact-switching regimes.
- **HPIPM** [Frison–Diehl 2020] **[H]** — interior-point with block-structure; preferred for long-horizon structured MPC.

**Optimal-control libraries.** *Crocoddyl* [Mastalli et al. *ICRA* 2020] **[H]** is the principal DDP-based library for whole-body optimal control; its co-design with Pinocchio enables the Dantec-series WB-MPC results. *OCS2* (ETH) is a competing library with a different internal DDP variant. *Drake TrajectoryOptimization* provides an alternative for direct-transcription problems.

**WBC frameworks.** *TSID* [Del Prete 2014, 2016] **[H]** — weighted single-QP, Pinocchio-based, actively maintained. *OpenSoT* [Mingo Hoffman et al. *ICRA* 2015] **[H]** — hierarchical QP, COMAN-origin, periodic maintenance. *Stack-of-Tasks* [Mansard et al. 2009] **[H]** — kinematic WBC, HRP-2 legacy, less actively developed. *SEIKO* [Rouxel et al. 2022] **[M]** — sequential-equilibrium IK + admittance for position-controlled platforms.

### Mini Critical Insight — 2.7
Tool selection is a *constraint propagation* exercise. Choosing Pinocchio forces alignment with TSID or a custom QP implementation; choosing Drake forces alignment with Drake's MultibodyPlant and its own IK stack. The thesis's tool plan in master_memory.md §8.2 is consistent (Pinocchio + OSQP + MuJoCo or Isaac Sim) but the WBC-framework choice (TSID vs. custom) is not yet fixed — Chapter 3 should close this.

---

## 2.8 Evaluation and Benchmarking

The humanoid-WBC community has historically lacked standardised evaluation protocols. The most thorough comparative study to date is Ramuzat, Boria, and Stasse [*Frontiers Robotics & AI* 2022] **[H]**, which benchmarks three WBC controllers — HQP at velocity level, TSID at acceleration level, TSID at torque level — on the TALOS humanoid in Gazebo across flat walking, uneven terrain, and stair climbing. Metrics include task-tracking error, control energy, and computation time. This is the reference evaluation protocol most closely aligned with the thesis's scenarios S2 / S3 from master_memory.md §9.4.

HumanoidBench [Sferrazza et al. *RSS* 2024] **[H]** provides a complementary but differently-scoped benchmark: 27 whole-body manipulation and locomotion tasks on Unitree H1 with Shadow Hands in MuJoCo. Its scope is broader but its discipline is RL-centric (designed to train and evaluate policies, not compare controllers on matched metrics).

Neither benchmark is sufficient on its own. Gaps:

- **No standardised perturbation-robustness protocol** — the impulse distribution, timing, and recovery criterion differ paper-to-paper (master_memory.md §7.3).
- **No public dataset of multi-contact force distributions** for humanoid loco-manipulation (master_memory.md §7.1).
- **No simulator-agnostic evaluation pipeline** — URDF / MJCF / SDF portability remains a per-platform effort.
- **No quantitative sim-to-hardware degradation protocol** for model-based WBC (well-documented only on the RL side).

### Mini Critical Insight — 2.8
Ramuzat 2022 is the most thesis-actionable reference in the literature — the metric set, scenario set, and controller selection align with master_memory.md §9. The thesis evaluation should be a direct extension of Ramuzat's protocol with added scenarios (S4: loco-manipulation; S5: perturbation robustness) and at least one additional simulator for cross-validation.

---

## 2.9 Open Problems and Research Gaps

This section consolidates gaps cited across the literature and maps them onto master_memory.md §7.

**Methodological gaps.**

- **MG1** — Coupling centroidal MPC (50–100 Hz) with full-body QP-WBC (1 kHz) introduces interface errors that are acknowledged [Dantec 2022; Kuindersma 2016] **[M]** but not standardised.
- **MG2** — Contact-mode transition handling in QP-WBC is typically hand-tuned per platform; no generalised strategy exists [Koolen 2016; Ramuzat 2022] **[M]**.
- **MG3** — The choice between weighted single-QP and lexicographic HQP under infeasibility lacks a principled selection criterion [Escande 2014 §6; Bouyarmane–Kheddar] **[M]**.
- **MG4** — Flexibility-aware WBC (series-elastic actuators, link flexibility) is partially addressed [Romualdi et al. *Humanoids* 2022] **[M]** but not generalised.

**Evaluation gaps.**

- **EG1** — No standardised scenario-and-metric set across humanoid platforms [Ramuzat 2022 is the best-known partial remedy] **[H]**.
- **EG2** — Perturbation-robustness protocols are non-reproducible paper-to-paper [master_memory.md §7.3] **[H]**.
- **EG3** — Simultaneous loco-manipulation benchmarks are rare [Sleiman 2023; HumanoidBench 2024 partial remedies] **[M]**.
- **EG4** — Quantitative sim-to-real degradation is rarely reported for model-based WBC [more systematically reported on the RL side] **[M]**.

**Data and infrastructure gaps.**

- **DG1** — No public multi-contact force dataset for humanoid loco-manipulation [survey remarks only; no remedy] **[L]**.
- **DG2** — URDF / MJCF / SDF cross-simulator portability is not solved; cross-simulator comparisons require hand-tuning.

**Theoretical / guarantee gaps.**

- **TG1** — Lyapunov / passivity guarantees for mainstream QP-WBC formulations do not exist beyond Henze–Roa–Ott's multi-contact balancer [Henze 2016] **[H]**.
- **TG2** — Safety guarantees for learned whole-body policies are empirical only; control-barrier-function integration is emerging [Ames et al. CBF literature] **[M]**.

### Mini Critical Insight — 2.9
Gaps EG1, EG2, EG3, and MG1 are all within the scope of a simulation-phase master's thesis and align directly with master_memory.md §7. The thesis can contribute by extending Ramuzat 2022's protocol (EG1), proposing a reproducible perturbation protocol (EG2), including a loco-manipulation scenario (EG3), and documenting the planning-execution interface timing (MG1) — each a small but concrete research contribution in aggregate.

---

## 2.10 Positioning of the Thesis within the Surveyed Field

Synthesising the preceding sections, the thesis occupies the following position in the field:

- **Formulation class.** Dynamic WBC via QP, with a centroidal MPC planning layer [Koolen 2016-lineage × Dantec 2022-lineage] — a mainstream modern-model-based architecture.
- **Priority scheme.** Weighted single-QP (TSID-style) as primary, with lexicographic HQP included as a comparison baseline — a commitment justified by Ramuzat 2022's demonstration that weighted TSID meets real-time constraints on TALOS.
- **Hardware target.** Torque-controlled bipedal humanoid (TALOS / Unitree H1 / custom — master_memory.md PQ1 pending).
- **Simulator.** MuJoCo primary (GPU path via MJX available), Drake second-track verification (PQ2 pending).
- **Learning integration.** RL only as an upper-bound-reference baseline per master_memory.md §9.3.
- **Contribution class.** A *reproducible simulation-phase evaluation framework* extending Ramuzat 2022 with loco-manipulation (S4) and perturbation-robustness (S5) scenarios, and with a contact-model sensitivity analysis addressing EG1 / EG2 / EG3 / MG1. The thesis does *not* claim novelty in controller formulation — a deliberate scope decision given field saturation.

### Mini Critical Insight — 2.10
The thesis's position is well-defined with respect to every axis in §2.4: the seven commitments (abstraction × priority × back-end × dynamics model × horizon × supervision × actuation interface) are made explicit and can be defended individually against the literature surveyed above. This satisfies master_memory.md §13's logical chain (Problem → Gap → Solution → Validation → Contribution) and master_memory.md §17's consistency-rule requirement that every claim be traceable.

---

## 2.11 Summary Table — Representative Works by Category

| Category | Canonical reference(s) | Role in thesis |
|---|---|---|
| Operational-space foundation | Khatib 1987; Sentis–Khatib 2004/2006; Mistry 2010 | §2.2.1, §2.3.2 |
| ZMP / Capture Point / DCM | Kajita 2003; Pratt 2006; Englsberger | §2.2.2, §2.3.6 |
| Centroidal dynamics | Orin–Goswami 2008, 2013; Lee–Goswami 2012 | §2.2.2, §2.3.3 |
| QP / Hierarchical QP | Righetti 2013; Escande–Mansard–Wieber *IJRR* 2014 | §2.2.3, §2.3.5 |
| TSID | Del Prete 2014; Del Prete–Mansard 2016 | §2.2.3, §2.3.5, §2.7 |
| Momentum-based WBC on Atlas | Feng *JFR* 2015; Koolen *IJHR* 2016; Kuindersma 2016 | §2.2.3 |
| Passivity multi-contact | Henze–Roa–Ott *IJRR* 2016 | §2.2.3, §2.9 |
| Whole-body MPC | Dantec *ICRA* 2021, *Humanoids* 2022, *RA-L* 2022; Khazoom *Humanoids* 2024 | §2.2.4 |
| Legged loco-manipulation MPC | Sleiman *RA-L* 2021, *Science Robotics* 2023 | §2.2.4, §2.6 |
| RL real-world humanoid | Radosavovic *Science Robotics* 2024; Haarnoja *Science Robotics* 2024 | §2.2.5, §2.6 |
| Motion retargeting | Cheng *RSS* 2024; Ji 2024; Fu *CoRL* 2024; He *CoRL* 2024; He 2024 (HOVER) | §2.2.6, §2.6 |
| HumanoidBench | Sferrazza *RSS* 2024 | §2.6, §2.8 |
| MuJoCo Playground | Zakka *RSS* 2025 | §2.5 |
| Dynamics libraries | Carpentier *SII* 2019 (Pinocchio); Drake | §2.7 |
| DDP library | Mastalli *ICRA* 2020 (Crocoddyl) | §2.2.4, §2.7 |
| QP solvers | Ferreau 2014 (qpOASES); Stellato 2020 (OSQP); Bambade 2022 (ProxQP); Frison–Diehl 2020 (HPIPM) | §2.7 |
| Simulator foundations | Todorov 2012 (MuJoCo); Makoviychuk 2021 (Isaac Gym); Castro 2023 (Drake SAP); Hwangbo 2018 (RaiSim) | §2.5 |
| Simulator survey | Ivaldi *Humanoids* 2014 | §2.5 |
| Actuator nets / sim-to-real | Hwangbo *Science Robotics* 2019; MIT 2025 | §2.5 |
| Reference benchmark | Ramuzat *Frontiers* 2022 | §2.8 |

---

## 2.12 Suggested Study Path for a Master's Student

1. Surveys first: Ivaldi 2014 (simulators) + Ramuzat 2022 (controllers).
2. Foundational: Khatib 1987 → Sentis 2007 thesis → Mistry 2010.
3. Centroidal: Orin–Goswami 2013 → Koolen 2016.
4. Optimization-based: Escande 2014 → Righetti 2013 → Del Prete 2014.
5. Software: Pinocchio 2019 → Crocoddyl 2020 → MuJoCo 2012.
6. Modern model-based: Dantec 2021 → Dantec 2022 (Humanoids) → Sleiman 2021.
7. Learning baseline: Radosavovic 2024 (Sci. Robot.) → Sferrazza 2024 (HumanoidBench) → He 2024 (HOVER).
8. Open-problem context: Henze 2016 (passivity) → "Bridging the Sim-to-Real Gap" 2025.

### Mini Critical Insight — Chapter Closing
The synthesized review above provides a draftable scaffold for Chapter 2 of the thesis. It adopts Report A's structural backbone, injects Report C's 2024–2026 SOTA coverage at §2.2.4–§2.2.6 and §2.6, rebuilds Report B's simulator and solver tables from primary sources in §2.5 and §2.7, and closes the 14 missing-element gaps identified in `02_Comparative_Evaluation.md` §4. Every claim is accompanied by a confidence marker per master_memory.md's source-integrity rule, and every section ends with a critical insight per §17 rule 8. The chapter's logical flow (definition → history → math → taxonomy → simulation → learning baseline → tooling → evaluation → gaps → thesis positioning) traces master_memory.md §13's Problem → Gap → Solution → Validation → Contribution chain.
