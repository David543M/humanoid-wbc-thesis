"""
probe_priority_inversion.py — the hazard the HQP baseline would have arbitrated,
measured directly on the existing weighted executor.

Ch3 §3.4 (insight): "an active torque limit can divert the optimisation effort
away from a higher-band task encoded only by its weight,
producing a tracking degradation invisible in the solver's exit status."

This is testable without implementing an HQP. At each tick we read the ACTUAL QP
solution (patch on solve_qp, no reimplementation of control()), determine
which torque bounds are active, and compute the task residuals:
    r_com = ||Jcom qdd* - a_com||   (weight 100)
    r_ori = ||Jori qdd* - a_ori||   (weight  40)
    r_pos = ||Jpos qdd* - a_pos||   (weight   1)

Priority inversion = when a constraint bites, the HIGH-priority task degrades
MORE than the low-priority task. The clean test is therefore the RATIO
r_com/r_pos on constrained vs free ticks, not r_com alone.

TWO MODES, because the imposed bound is not the binding bound:
  A  bounds as implemented  : +/-300 N.m (fallback, cf. Ch4 §4.4)
  B  bounds as specified    : per-joint ctrlrange from the model
Mode B also measures what the "owed" correction of Ch7 §7.2 is worth.

USAGE (Anaconda terminal, from pal_talos/):
    python probe_priority_inversion.py                 # 3 seeds, modes A and B
    python probe_priority_inversion.py --seeds 5 --force 150
"""
import argparse
import numpy as np
import mujoco
import talos_wbc as W

TOL  = 1e-6         # activity margin of a bound (N.m)
ATOL = 1e-6         # activity margin of an inequality row (slack ~ 0)


def task_residuals(m, d, wbc, qdd):
    """Residuals of the three weighted tasks, on the SAME state as the solved QP."""
    nv = wbc.nv
    Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, wbc.base)
    com = d.subtree_com[wbc.base]
    a_com = W.KP_COM * (wbc.com_ref - com) - W.KD_COM * (Jcom @ d.qvel)

    tmp = np.zeros((3, nv)); Jori = np.zeros((3, nv))
    mujoco.mj_jacBody(m, d, tmp, Jori, wbc.base)
    q = d.qpos[3:7]; err = np.zeros(3); neg = np.zeros(4); res = np.zeros(4)
    mujoco.mju_negQuat(neg, q)
    mujoco.mju_mulQuat(res, np.array([1.0, 0, 0, 0]), neg)
    mujoco.mju_quat2Vel(err, res, 1.0)
    a_ori = W.KP_ORI * err - W.KD_ORI * (Jori @ d.qvel)

    Jpos = np.zeros((wbc.nu, nv))
    for k, dof in enumerate(wbc.act_dofs):
        Jpos[k, dof] = 1.0
    qcur = np.array([d.qpos[a] for a in wbc.act_qadr])
    vcur = np.array([d.qvel[a] for a in wbc.act_dofs])
    a_pos = W.KP_POS * (wbc.home - qcur) - W.KD_POS * vcur

    return (float(np.linalg.norm(Jcom @ qdd - a_com)),
            float(np.linalg.norm(Jori @ qdd - a_ori)),
            float(np.linalg.norm(Jpos @ qdd - a_pos)))


def run(seed, mode, force, dur=6.0, t_push=2.0, imp=0.1):
    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    wbc = W.WBC(m, d, corners=W.CORNERS_MEASURED)

    if mode == "B":                       # bounds as specified
        wbc.tau_min = m.actuator_ctrlrange[:, 0].copy()
        wbc.tau_max = m.actuator_ctrlrange[:, 1].copy()
        wbc._Aineq, wbc._bineq = wbc._build_ineq()

    if seed is not None:
        rng = np.random.default_rng(seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv); mujoco.mj_forward(m, d)

    # -- capture the ACTUAL QP solution, without reimplementing control() --
    box = {}
    orig = W.solve_qp
    def spy(G, a, Aeq, beq, Aineq, bineq):
        x = orig(G, a, Aeq, beq, Aineq, bineq); box["x"] = x; return x
    W.solve_qp = spy

    # indices of the three inequality blocks
    ncp = wbc.ncp
    uni_idx, fri_idx = [], []
    for c in range(ncp):
        b = 5 * c
        uni_idx.append(b)
        fri_idx += [b + 1, b + 2, b + 3, b + 4]
    tau_idx = list(range(5 * ncp, 5 * ncp + 2 * wbc.nu))
    uni_idx = np.array(uni_idx); fri_idx = np.array(fri_idx); tau_idx = np.array(tau_idx)

    rows = []
    try:
        while d.time < dur:
            box.pop("x", None)
            tau = wbc.control()
            x = box.get("x")
            if x is not None:
                qdd = x[:wbc.nv]
                r = task_residuals(m, d, wbc, qdd)
                # activity PER inequality BLOCK, read off the actual solution:
                # order of _build_ineq() = per contact point [fz>=FZ_MIN, 4 x pyramid],
                # then 2 torque-bound rows per actuator.
                slack = wbc._Aineq @ x - wbc._bineq
                a_uni = int(np.sum(slack[uni_idx] < ATOL))
                a_fri = int(np.sum(slack[fri_idx] < ATOL))
                a_tau = int(np.sum(slack[tau_idx] < ATOL))
                rows.append((a_uni, a_fri, a_tau, r[0], r[1], r[2]))
            d.xfrc_applied[wbc.base, :] = 0.0
            if t_push <= d.time < t_push + imp:
                d.xfrc_applied[wbc.base, 1] = force
            mujoco.mj_step(m, d)
    finally:
        W.solve_qp = orig

    upright = bool(d.qpos[2] > 0.8)
    dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
    return np.array(rows), upright, dev


def summarise(tag, rows, upright, dev):
    a_uni, a_fri, a_tau = rows[:, 0], rows[:, 1], rows[:, 2]
    rcom, rori, rpos = rows[:, 3], rows[:, 4], rows[:, 5]
    act = a_uni + a_fri + a_tau
    n = len(rows); nb = int((act > 0).sum())
    print("  %-24s ticks=%d  upright=%s  dev_fin=%.1f mm" % (tag, n, upright, 1e3 * dev))
    print("      ticks with >=1 active row     : unilat %5.1f %% | pyramid  %5.1f %% | torque %5.1f %% | all  %5.1f %%"
          % (100.0*(a_uni>0).mean(), 100.0*(a_fri>0).mean(),
             100.0*(a_tau>0).mean(), 100.0*(act>0).mean()))
    if int((a_tau > 0).sum()) == 0:
        print("      -> NO torque bound active: the inversion hazard from actuator "
              "saturation is not exercised.")
    if nb == 0:
        print("      -> no active inequality at all.")
        return
    free = act == 0
    bind = act > 0
    if free.sum() == 0:
        print("      -> all inequalities active throughout: no free population "
              "for the comparison.")
        return
    for nm, v in (("r_com (w=100)", rcom), ("r_ori (w=40)", rori), ("r_pos (w=1)", rpos)):
        print("      %-14s free  %8.3f | bound     %8.3f | x%.2f"
              % (nm, np.median(v[free]), np.median(v[bind]),
                 np.median(v[bind]) / max(np.median(v[free]), 1e-12)))
    ratio_f = np.median(rcom[free] / np.maximum(rpos[free], 1e-12))
    ratio_b = np.median(rcom[bind] / np.maximum(rpos[bind], 1e-12))
    print("      ratio r_com/r_pos : free %.4f | bound %.4f | x%.2f  <-- inversion test"
          % (ratio_f, ratio_b, ratio_b / max(ratio_f, 1e-12)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=3)
    ap.add_argument("--forces", type=float, nargs="+", default=[150.0, 350.0, 450.0])
    ap.add_argument("--dur", type=float, default=4.0)
    a = ap.parse_args()
    print("=" * 78)
    print("PRIORITY INVERSION PROBE — lateral push at t=2 s, %.0f s/trial" % a.dur)
    print("  mode A = torque bounds as IMPLEMENTED (+/-300 N.m, fallback)")
    print("  mode B = torque bounds as SPECIFIED (per-joint ctrlrange)")
    print("  S1 lateral envelope measured ~450 N; >500 N leaves the domain (unstable sim)")
    print("  inversion test = ratio r_com/r_pos on constrained vs free ticks")
    print("=" * 78)
    for F in a.forces:
        print("\n=== F = %.0f N ===" % F)
        for mode in ("A", "B"):
            for sd in range(a.seeds):
                rows, up, dev = run(sd, mode, F, a.dur)
                summarise("mode %s seed %d" % (mode, sd), rows, up, dev)


if __name__ == "__main__":
    main()
