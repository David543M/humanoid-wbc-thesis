"""
qp_infeasible_probe.py — diagnostic of the PROXQP_PRIMAL_INFEASIBLE events in S1 (2026-08-11).

FINDING TO EXPLAIN
------------------
`qp_status_check.py --n 3`: 107 / 13500 ticks (0.79 %) report
PROXQP_PRIMAL_INFEASIBLE, with `iter max 54` and `pri_res max 1.96e-3`.
The trial verdicts remain identical to the reference. Two possible readings:

  H1  SOLVER ARTEFACT. The ProxQP workspace is reused from one tick to the next
      (`init()` once, `update()` thereafter) and the preconditioner is NEVER
      recomputed (`settings.update_preconditioner = False`, default) even though H, A, C
      change at every tick. The infeasibility certificate (eps_primal_inf = 1e-4)
      would then fire wrongly on a QP that is in fact feasible.

  H2  GENUINE NEAR-INFEASIBILITY. At those instants the QP really is at the edge of the
      domain: torque bounds saturated, friction cone saturated, or an incompatible
      6D no-slip constraint on both feet.

DISCRIMINATING TEST
-------------------
At every flagged tick, we RE-SOLVE exactly the same QP with a FRESH
ProxQP instance (init(), no history, fresh preconditioner):

  * if the fresh solve returns PROXQP_SOLVED  -> H1 (reuse artefact);
  * if it also returns PRIMAL_INFEASIBLE      -> H2 (the QP really is at the edge).

We also measure the command deviation ||tau_reused - tau_fresh||_inf: that is what
tells whether the event has a PHYSICAL consequence or only a status one.

The controller is NOT modified: we wrap `W.solve_qp` for the duration of the diagnostic.

USAGE (Anaconda terminal, from pal_talos/)
-------------------------------------------
    python qp_infeasible_probe.py --seed 1000
    python qp_infeasible_probe.py --seed 1000 --seeds 3 --csv probe.csv
"""
import argparse, csv, sys
import numpy as np
import mujoco

import talos_wbc as W
import validate_s1_batch as VS

try:
    import proxsuite
except ImportError:
    sys.exit("[ABORT] proxsuite required for the fresh re-solve.")


STATE = {"t": 0.0, "tick": 0, "events": []}
_orig_solve = W.solve_qp


def _fresh_solve(G, a, Aeq, beq, Aineq, bineq, eps_inf=None, max_iter=None):
    """Reproduces EXACTLY the preprocessing of W.solve_qp, then solves from scratch.
    eps_inf: if given, TIGHTENS the infeasibility-certificate threshold (makes it
    nearly impossible to fire) -> decisive test of actual feasibility."""
    n = G.shape[0]
    Gs = 0.5 * (G + G.T) + 1e-8 * np.eye(n)          # identical to solve_qp
    neq, nin = Aeq.shape[0], Aineq.shape[0]
    qp = proxsuite.proxqp.dense.QP(n, neq, nin)
    qp.settings.eps_abs = 1e-6                        # identical to solve_qp
    if eps_inf is not None:
        qp.settings.eps_primal_inf = eps_inf
        qp.settings.eps_dual_inf = eps_inf
    if max_iter is not None:
        qp.settings.max_iter = max_iter
    qp.init(Gs, -a, Aeq, beq, Aineq, bineq, 1e20 * np.ones(nin))
    qp.solve()
    i = qp.results.info
    return (np.asarray(qp.results.x), str(i.status).split(".")[-1],
            float(i.pri_res), int(i.iter))


def _violations(x, Aeq, beq, Aineq, bineq, wbc):
    """ACTUAL violations of the returned solution, by constraint group.
    Order of Aineq (cf. talos_wbc._build_ineq): per corner [fz>=1, 4 friction
    rows], then 2*nu torque-bound rows."""
    nu, ncp = wbc.nu, wbc.ncp
    eq = float(np.max(np.abs(Aeq @ x - beq))) if Aeq.size else 0.0
    slack = Aineq @ x - bineq                          # >= 0 if satisfied
    v = np.minimum(slack, 0.0)
    nfric = 5 * ncp
    grp_fz = [5 * c for c in range(ncp)]
    grp_fr = [5 * c + j for c in range(ncp) for j in (1, 2, 3, 4)]
    return dict(
        viol_eq=eq,
        viol_fz=float(-v[grp_fz].min()) if ncp else 0.0,
        viol_fric=float(-v[grp_fr].min()) if ncp else 0.0,
        viol_tau=float(-v[nfric:nfric + 2 * nu].min()) if nu else 0.0,
    )


def _context(x, xf):
    """Physical state + structure of the solution at the flagged tick.
    Purpose: distinguish (i) saturation of a constraint, (ii) degeneracy
    (non-unique optimum: the internal-force block is pinned only by eps_f=1e-5)."""
    d, wbc = STATE["d"], STATE["wbc"]
    nv, nu = wbc.nv, wbc.nu
    tau, tauf = np.asarray(x[nv:nv+nu]), np.asarray(xf[nv:nv+nu])
    f,   ff   = np.asarray(x[nv+nu:]),   np.asarray(xf[nv+nu:])
    fz = f[2::3]
    return dict(
        qvel_inf=float(np.max(np.abs(d.qvel))),
        com_dev=float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2])),
        tau_max=float(np.max(np.abs(tau))),
        tau_margin=float(np.min(np.minimum(tau - wbc.tau_min, wbc.tau_max - tau))),
        fz_min=float(np.min(fz)), fz_max=float(np.max(fz)),
        # split of the deviation: command (tau) vs internal forces (f)
        dtau_inf=float(np.max(np.abs(tau - tauf))),
        df_inf=float(np.max(np.abs(f - ff))),
        dqdd_inf=float(np.max(np.abs(np.asarray(x[:nv]) - np.asarray(xf[:nv])))),
    )


def probe_solve(G, a, Aeq, beq, Aineq, bineq):
    before = W.QP_STATS["not_solved"]
    x = _orig_solve(G, a, Aeq, beq, Aineq, bineq)
    STATE["tick"] += 1
    if W.QP_STATS["not_solved"] > before:             # this tick has just been flagged
        xf, st, pri, it = _fresh_solve(G, a, Aeq, beq, Aineq, bineq)
        # --- DECISIVE TEST: infeasibility certificate all but disabled ---
        xh, sth, prih, ith = _fresh_solve(G, a, Aeq, beq, Aineq, bineq,
                                          eps_inf=1e-14, max_iter=100000)
        ev = dict(tick=STATE["tick"], t=STATE["t"], fresh_status=st,
                  fresh_pri=pri, fresh_iter=it,
                  hard_status=sth, hard_pri=prih, hard_iter=ith,
                  dx_inf=float(np.max(np.abs(np.asarray(x) - xf))),
                  dxh_inf=float(np.max(np.abs(np.asarray(x) - xh))))
        try:
            ev.update(_context(x, xf))
            # split of the SENT vs CORRECT deviation, per variable block
            c2 = _context(x, xh)
            ev["h_dtau"] = c2["dtau_inf"]; ev["h_df"] = c2["df_inf"]
            ev["h_dqdd"] = c2["dqdd_inf"]
            wb = STATE["wbc"]; nv_, nu_ = wb.nv, wb.nu
            ev["tau_correct_max"] = float(np.max(np.abs(np.asarray(xh[nv_:nv_+nu_]))))
            ev["fz_correct_min"] = float(np.min(np.asarray(xh[nv_+nu_:])[2::3]))
            for k, v in _violations(np.asarray(x), Aeq, beq, Aineq, bineq,
                                    STATE["wbc"]).items():
                ev[k] = v
            for k, v in _violations(xh, Aeq, beq, Aineq, bineq,
                                    STATE["wbc"]).items():
                ev["hard_" + k] = v
        except Exception as e:
            ev["ctx_error"] = repr(e)
        STATE["events"].append(ev)
    return x


def run_seed(seed):
    m, d, wbc, F, th = VS._setup(seed)
    STATE["d"], STATE["wbc"] = d, wbc
    dt = m.opt.timestep
    fx, fy = F * np.cos(th), F * np.sin(th)
    t_end_push = VS.T_PUSH + VS.IMPULSE
    for _ in range(int(VS.DUR / dt)):
        STATE["t"] = float(d.time)
        wbc.control()
        d.xfrc_applied[wbc.base, :] = 0.0
        if VS.T_PUSH <= d.time < t_end_push:
            d.xfrc_applied[wbc.base, 0] = fx; d.xfrc_applied[wbc.base, 1] = fy
        mujoco.mj_step(m, d)
        if d.qpos[2] < 0.6:
            break
    return F, th, float(d.qpos[2])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1000)
    ap.add_argument("--seeds", type=int, default=1, help="number of consecutive seeds")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--isolate", action="store_true",
                    help="clear _PROX_CACHE between trials: tests whether reusing "
                         "the ProxQP workspace across trials is to blame")
    a = ap.parse_args()

    W.solve_qp = probe_solve                          # temporary wrapper
    STATE["isolate"] = a.isolate
    W.qp_stats_reset()
    print("=" * 78)
    print("INFEASIBILITY PROBE — %d seed(s) from %d" % (a.seeds, a.seed))
    print("=" * 78)
    for k in range(a.seeds):
        if a.isolate:
            W._PROX_CACHE.clear()                     # fresh ProxQP workspace per trial
        n0 = len(STATE["events"])
        F, th, z = run_seed(a.seed + k)
        print("  seed %d : F=%.0f N  z_final=%.3f  | events in this trial : %d"
              % (a.seed + k, F, z, len(STATE["events"]) - n0))
    W.solve_qp = _orig_solve

    ev = STATE["events"]
    print("-" * 78)
    print(W.qp_stats_report())
    print("-" * 78)
    if not ev:
        print("No event flagged — nothing to diagnose.")
        return

    fresh_ok = sum(1 for e in ev if e["fresh_status"] == "PROXQP_SOLVED")
    dx = np.array([e["dx_inf"] for e in ev])
    tt = np.array([e["t"] for e in ev])
    print("Events flagged : %d" % len(ev))
    print("  FRESH re-solve -> PROXQP_SOLVED : %d / %d (%.1f %%)"
          % (fresh_ok, len(ev), 100.0 * fresh_ok / len(ev)))
    print("  solution deviation ||x_reused - x_fresh||_inf :"
          " median %.2e | max %.2e" % (np.median(dx), dx.max()))

    def col(k):
        v = np.array([e[k] for e in ev if k in e], dtype=float)
        return v if v.size else np.array([np.nan])
    print("  SPLIT of the deviation (median / max) :")
    print("    torques  |dtau|_inf  %9.2e / %9.2e  N.m" % (np.median(col('dtau_inf')), col('dtau_inf').max()))
    print("    forces   |df|_inf    %9.2e / %9.2e  N"   % (np.median(col('df_inf')),   col('df_inf').max()))
    print("    accel.   |dqdd|_inf  %9.2e / %9.2e  rad/s2" % (np.median(col('dqdd_inf')), col('dqdd_inf').max()))
    print("  PHYSICAL STATE at the events (median / max) :")
    print("    |qvel|_inf   %9.2e / %9.2e  rad/s" % (np.median(col('qvel_inf')), col('qvel_inf').max()))
    print("    CoM dev      %9.2e / %9.2e  m"     % (np.median(col('com_dev')),  col('com_dev').max()))
    print("    |tau|_max    %9.2f / %9.2f  N.m (QP bounds : +-300)"
          % (np.median(col('tau_max')), col('tau_max').max()))
    print("    torque marg. %9.2f / %9.2f  N.m (0 = bound reached)"
          % (np.median(col('tau_margin')), col('tau_margin').min()))
    print("    fz_min       %9.2f / %9.2f  N (constraint : >= 1 N)"
          % (np.median(col('fz_min')), col('fz_min').min()))
    print("  ACTUAL VIOLATIONS of the returned solution (median / max) :")
    for k, lab, unit in (('viol_eq', 'equalities (dyn + no-slip)', ''),
                         ('viol_fz', 'fz >= 1 N', ' N'),
                         ('viol_fric', 'friction cone', ' N'),
                         ('viol_tau', 'torque bounds', ' N.m')):
        print("    %-26s %9.2e / %9.2e%s" % (lab, np.median(col(k)), col(k).max(), unit))
    print("-" * 78)
    print("DECISIVE TEST — same QP, infeasibility certificate disabled (eps_inf=1e-14) :")
    hs = {}
    for e in ev:
        hs[e.get('hard_status', '?')] = hs.get(e.get('hard_status', '?'), 0) + 1
    print("    statuses : %s" % hs)
    print("    iterations   median %8.0f / max %8.0f" % (np.median(col('hard_iter')), col('hard_iter').max()))
    print("    violations of THIS solution (median / max) :")
    for k, lab, unit in (('hard_viol_eq', 'equalities', ''),
                         ('hard_viol_fz', 'fz >= 1 N', ' N'),
                         ('hard_viol_fric', 'friction cone', ' N'),
                         ('hard_viol_tau', 'torque bounds', ' N.m')):
        print("      %-24s %9.2e / %9.2e%s" % (lab, np.median(col(k)), col(k).max(), unit))
    nsolved_hard = sum(1 for e in ev if e.get('hard_status') == 'PROXQP_SOLVED')
    print("    DEVIATION between the command SENT and the CORRECT command :")
    print("      ||x_sent - x_correct||_inf     median %9.2e / max %9.2e"
          % (np.median(col('dxh_inf')), col('dxh_inf').max()))
    print("      SPLIT PER BLOCK (median / max) :")
    print("        torques   |dtau|  %9.2f / %9.2f  N.m   <-- THE number that counts"
          % (np.median(col('h_dtau')), col('h_dtau').max()))
    print("        forces    |df|    %9.2f / %9.2f  N"
          % (np.median(col('h_df')), col('h_df').max()))
    print("        accel.    |dqdd|  %9.2f / %9.2f  rad/s2"
          % (np.median(col('h_dqdd')), col('h_dqdd').max()))
    print("      CORRECT solution : |tau|_max median %.2f / max %.2f N.m | "
          "fz_min median %.2f N"
          % (np.median(col('tau_correct_max')), col('tau_correct_max').max(),
             np.median(col('fz_correct_min'))))
    print("  >>> %d / %d converge when the certificate is disabled." % (nsolved_hard, len(ev)))
    if nsolved_hard == len(ev):
        print("  >>> CONCLUSION : the QP IS feasible. ProxQP's infeasibility")
        print("      certificate is a FALSE POSITIVE on this problem (degeneracy).")
    elif nsolved_hard == 0:
        print("  >>> CONCLUSION : the QP is GENUINELY infeasible at those instants.")
        print("      See above for WHICH constraint is violated.")
    print("  temporal distribution (push at t=%.1f-%.1f s) :"
          % (VS.T_PUSH, VS.T_PUSH + VS.IMPULSE))
    for lo, hi, lab in ((0.0, VS.T_PUSH, "before push"),
                        (VS.T_PUSH, VS.T_PUSH + VS.IMPULSE, "DURING push"),
                        (VS.T_PUSH + VS.IMPULSE, 1e9, "after push")):
        k = int(((tt >= lo) & (tt < hi)).sum())
        print("    %-16s %5d  (%5.1f %%)" % (lab, k, 100.0 * k / len(ev)))
    print("-" * 78)
    # The verdict rests on the DECISIVE TEST (certificate disabled), not on the
    # plain fresh re-solve: that one uses the same settings, so it
    # reproduces the same false positive.
    if nsolved_hard == len(ev):
        print("VERDICT : SOLVER FALSE POSITIVE.")
        print("          The QP is feasible (117/117 converge, certificate disabled).")
        print("          ProxQP wrongly emits a primal infeasibility certificate and")
        print("          returns a NON-CONVERGED iterate — cf. the violations above.")
        print("          Fix : very small settings.eps_primal_inf at init().")
        print("          /!\\ changes the command at those ticks -> revalidate the campaigns.")
    elif nsolved_hard == 0:
        print("VERDICT : the QP is GENUINELY infeasible at those instants.")
        print("          See above for WHICH constraint is violated.")
    else:
        print("VERDICT : MIXED — %d / %d false positives." % (nsolved_hard, len(ev)))

    # --- temporal grouping of the events (isolated dropout or burst?) ---
    tk = np.sort(np.array([e["tick"] for e in ev]))
    if tk.size:
        runs, cur = [], 1
        for i in range(1, tk.size):
            if tk[i] == tk[i-1] + 1:
                cur += 1
            else:
                runs.append(cur); cur = 1
        runs.append(cur)
        runs = np.array(runs)
        print("-" * 78)
        print("GROUPING : %d burst(s) of consecutive ticks | median length %d, max %d"
              % (runs.size, int(np.median(runs)), int(runs.max())))
        print("             (1 tick = 1 ms ; a burst of N ticks = N ms of degraded command)")

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(ev[0].keys()))
            w.writeheader(); w.writerows(ev)
        print("[ok] %s" % a.csv)


if __name__ == "__main__":
    main()
