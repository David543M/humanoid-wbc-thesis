# Which code produced which number

*Traceability manifest — `pal_talos/` — 2026-08-25*

This directory contains several control loops that all write to `d.ctrl`. Nothing in their
names indicates which one is authoritative for a given scenario. This file does.

---

## 1. Torque-writing sites

`grep -n "d\.ctrl\[" *.py` — **four** distinct implementations:

| File | Line | Class / function | Used by |
|---|---|---|---|
| `talos_wbc.py` | 380 | `WBC.control()` | direct use of `WBC` (balance, probes) |
| `talos_dcm_walk.py` | 288 | `DCMWalk.control()` | **S2 and S3** |
| `talos_s4_qs_carry.py` | 270 | `S4CarryQS.control()` | **S4** |
| `s1_baselines.py` | 98, 153 | baseline controllers | S1 comparators |

> **`DCMWalk.control()` does not call `super().control()`.** It re-implements the QP in
> full. A change applied to `talos_wbc.py` is therefore **dead code for S2, S3 and S4**.
> This was established the hard way: the first energy instrumentation patch produced
> `E_mech = 0.0` while `mass`, inherited from `WBC.__init__`, came out correct.

## 2. Effective inheritance chain

```
S2 : DCMWalkT (talos_dcm_walk_timing.py:80)  -> B.DCMWalk
       control() l.445  ->  super().control()  ->  DCMWalk.control()   [QP here]

S3 : StairQS (talos_s3_stairs_qs.py:47)      -> B.DCMWalk
       control() l.269  ->  super().control()  ->  DCMWalk.control()   [QP here]

S4 : S4CarryQS (talos_s4_qs_carry.py:131)    -> QS.StairQS -> B.DCMWalk
       control() l.191  ->  QP rewritten in place, no super() call     [QP here]
```

## 3. `DCMWalk.control()` vs `S4CarryQS.control()` — measured diff

Method: extract both method bodies, strip comments, normalise all whitespace, neutralise
module prefixes (`B.`, `W.`), then `difflib.unified_diff`.

**Result: 78 lines against 87. Nine differences, all additions. No deletions, no
modifications.**

Two of the nine are functional:

```python
        t_perf = time.perf_counter()
        for s in ("L", "R"):
            J_rel, a_ee, e = self._ee_task(s); add(J_rel, a_ee, self.w_ee); e_ee[s] = e
```

The other seven are logging (`ee_L`, `ee_R`, `ee_regime`, `com_err`, `ctrl_ms`, `grf`).

**Line-for-line identical:** CoM, base-orientation, posture, contact Baumgarte, swing and
foot-orientation tasks with the landing ramp; the equality constraint
`M q̈ − Sᵀτ − Jcᵀf = −h`; the `fz_min`, friction-pyramid (`MU`) and torque-bound
inequalities; the `W.solve_qp` call; the gravity-compensation plus posture-PD fallback;
the final clip.

**No constant is redefined:** `grep -E "^(W_|KP_|KD_)[A-Z_]* *=" talos_s4_qs_carry.py`
returns nothing. Gains and task weights are **imported** from `talos_dcm_walk` and
`talos_wbc`; they are not copies that could drift.

**Conclusion.** S4 runs the same formulation as S2 and S3 with **one additional weighted
task**. The optimisation problem solved at each tick is therefore not identical: the
cross-scenario comparison is one of executor machinery, not of objective.

## 4. Energy instrumentation

Accumulators `E_mech = ∫|τ·q̇| dt` and `E_sq = ∫‖τ‖² dt`, written **after torque
saturation**, hence on the torque actually applied. Present at the three active sites:
`talos_wbc.py:378`, `talos_dcm_walk.py:291`, `talos_s4_qs_carry.py:273`.

**Inertness.** Neither attribute is read by any control path — `grep -n "E_mech\|E_sq"`
on each executor returns only the initialisation and the accumulation. They cannot
influence the dynamics. `mass` is read once through `mj_getTotalmass` (94.003 kg) and is
never hard-coded.

**Single-run validation** (seed 0, frozen S2 configuration): `E_mech = 5173.6 J` over
`dist = 3.142 m`, giving a cost of transport of about 1.79 — a plausible order of
magnitude for a 94 kg humanoid.

**Campaigns** (`s2_batch_energy/`, `s3_batch_energy/`, `s4_batch_energy/`): the
instrumentation was absent when the headline campaigns ran, so these are separate
replications. Success rates 39/50, 11/35 and 4/30 all fall inside the two-sided Fisher
indistinguishability zones of the campaigns they repeat ([25, 43], [3, 19] and [1, 14]).

> These are **commanded torques from the QP**: no actuator model, no ohmic loss, no
> gearbox efficiency, and braking counted as expenditure. The quantity is a mechanical
> cost of command, not a power draw.

## 5. Historical snapshots

`_legacy/` holds earlier versions kept as a development record:
`talos_wbc_PRE_HIP_20260717.py`, `talos_wbc_PRE_QPSTATUS_20260811.py`,
`talos_dcm_walk_timing_PRE_SAGFB_20260720.py`, `talos_s3_stairs_qs_WORKING_3of3.py`,
`validate_s1_batch_BACKUP_20260716.py`. None is imported by the active code and none
contributes to a published result — verified by enumerating every `import` in the
directory before moving them.
