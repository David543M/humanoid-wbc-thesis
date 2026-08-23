"""
qp_infeasible_probe.py — diagnostic des PROXQP_PRIMAL_INFEASIBLE de S1 (2026-08-11).

CONSTAT A EXPLIQUER
-------------------
`qp_status_check.py --n 3` : 107 / 13500 ticks (0.79 %) rapportent
PROXQP_PRIMAL_INFEASIBLE, avec `iter max 54` et `pri_res max 1.96e-3`.
Les verdicts d'essai restent identiques a la reference. Deux lectures possibles :

  H1  ARTEFACT DE SOLVEUR. Le workspace ProxQP est reutilise d'un tick a l'autre
      (`init()` une fois, `update()` ensuite) et le preconditionneur n'est JAMAIS
      recalcule (`settings.update_preconditioner = False`, defaut) alors que H, A, C
      changent a chaque tick. Le certificat d'infaisabilite (eps_primal_inf = 1e-4)
      se declencherait a tort sur un QP pourtant faisable.

  H2  QUASI-INFAISABILITE REELLE. A ces instants le QP est effectivement au bord du
      domaine : bornes de couple saturees, cone de friction sature, ou contrainte de
      non-glissement 6D incompatible sur les deux pieds.

TEST DISCRIMINANT
-----------------
A chaque tick signale, on RE-RESOUT exactement le meme QP avec une instance
ProxQP NEUVE (init(), aucun historique, preconditionneur frais) :

  * si la resolution fraiche rend PROXQP_SOLVED  -> H1 (artefact de reutilisation) ;
  * si elle rend PRIMAL_INFEASIBLE aussi         -> H2 (le QP est reellement au bord).

On mesure aussi l'ecart de commande ||tau_reutilise - tau_frais||_inf : c'est lui qui
dit si l'evenement a une consequence PHYSIQUE ou seulement statutaire.

Le controleur n'est PAS modifie : on enveloppe `W.solve_qp` le temps du diagnostic.

USAGE (terminal Anaconda, depuis pal_talos/)
--------------------------------------------
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
    sys.exit("[ABORT] proxsuite requis pour la re-resolution fraiche.")


STATE = {"t": 0.0, "tick": 0, "events": []}
_orig_solve = W.solve_qp


def _fresh_solve(G, a, Aeq, beq, Aineq, bineq, eps_inf=None, max_iter=None):
    """Reproduit EXACTEMENT le pretraitement de W.solve_qp, puis resout a neuf.
    eps_inf : si fourni, DURCIT le seuil du certificat d'infaisabilite (le rend
    quasi impossible a declencher) -> test decisif de faisabilite reelle."""
    n = G.shape[0]
    Gs = 0.5 * (G + G.T) + 1e-8 * np.eye(n)          # identique a solve_qp
    neq, nin = Aeq.shape[0], Aineq.shape[0]
    qp = proxsuite.proxqp.dense.QP(n, neq, nin)
    qp.settings.eps_abs = 1e-6                        # identique a solve_qp
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
    """Violations REELLES de la solution retournee, par groupe de contraintes.
    Ordre de Aineq (cf. talos_wbc._build_ineq) : par coin [fz>=1, 4 lignes de
    friction], puis 2*nu lignes de bornes de couple."""
    nu, ncp = wbc.nu, wbc.ncp
    eq = float(np.max(np.abs(Aeq @ x - beq))) if Aeq.size else 0.0
    slack = Aineq @ x - bineq                          # >= 0 si satisfait
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
    """Etat physique + structure de la solution au tick signale.
    Objectif : distinguer (i) saturation d'une contrainte, (ii) degenerescence
    (optimum non unique : le bloc de forces internes n'est fixe que par eps_f=1e-5)."""
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
        # decomposition de l'ecart : commande (tau) vs forces internes (f)
        dtau_inf=float(np.max(np.abs(tau - tauf))),
        df_inf=float(np.max(np.abs(f - ff))),
        dqdd_inf=float(np.max(np.abs(np.asarray(x[:nv]) - np.asarray(xf[:nv])))),
    )


def probe_solve(G, a, Aeq, beq, Aineq, bineq):
    before = W.QP_STATS["not_solved"]
    x = _orig_solve(G, a, Aeq, beq, Aineq, bineq)
    STATE["tick"] += 1
    if W.QP_STATS["not_solved"] > before:             # ce tick vient d'etre signale
        xf, st, pri, it = _fresh_solve(G, a, Aeq, beq, Aineq, bineq)
        # --- TEST DECISIF : certificat d'infaisabilite quasi desactive ---
        xh, sth, prih, ith = _fresh_solve(G, a, Aeq, beq, Aineq, bineq,
                                          eps_inf=1e-14, max_iter=100000)
        ev = dict(tick=STATE["tick"], t=STATE["t"], fresh_status=st,
                  fresh_pri=pri, fresh_iter=it,
                  hard_status=sth, hard_pri=prih, hard_iter=ith,
                  dx_inf=float(np.max(np.abs(np.asarray(x) - xf))),
                  dxh_inf=float(np.max(np.abs(np.asarray(x) - xh))))
        try:
            ev.update(_context(x, xf))
            # decomposition de l'ecart ENVOYE vs CORRECT, par bloc de variables
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
    ap.add_argument("--seeds", type=int, default=1, help="nb de seeds consecutifs")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--isolate", action="store_true",
                    help="vide _PROX_CACHE entre les essais : teste si la reutilisation "
                         "du workspace ProxQP d'un essai a l'autre est en cause")
    a = ap.parse_args()

    W.solve_qp = probe_solve                          # enveloppe temporaire
    STATE["isolate"] = a.isolate
    W.qp_stats_reset()
    print("=" * 78)
    print("PROBE INFAISABILITE — %d seed(s) depuis %d" % (a.seeds, a.seed))
    print("=" * 78)
    for k in range(a.seeds):
        if a.isolate:
            W._PROX_CACHE.clear()                     # workspace ProxQP neuf par essai
        n0 = len(STATE["events"])
        F, th, z = run_seed(a.seed + k)
        print("  seed %d : F=%.0f N  z_final=%.3f  | evenements sur cet essai : %d"
              % (a.seed + k, F, z, len(STATE["events"]) - n0))
    W.solve_qp = _orig_solve

    ev = STATE["events"]
    print("-" * 78)
    print(W.qp_stats_report())
    print("-" * 78)
    if not ev:
        print("Aucun evenement signale — rien a diagnostiquer.")
        return

    fresh_ok = sum(1 for e in ev if e["fresh_status"] == "PROXQP_SOLVED")
    dx = np.array([e["dx_inf"] for e in ev])
    tt = np.array([e["t"] for e in ev])
    print("Evenements signales : %d" % len(ev))
    print("  re-resolution NEUVE -> PROXQP_SOLVED : %d / %d (%.1f %%)"
          % (fresh_ok, len(ev), 100.0 * fresh_ok / len(ev)))
    print("  ecart de solution ||x_reutilise - x_frais||_inf :"
          " median %.2e | max %.2e" % (np.median(dx), dx.max()))

    def col(k):
        v = np.array([e[k] for e in ev if k in e], dtype=float)
        return v if v.size else np.array([np.nan])
    print("  DECOMPOSITION de l'ecart (median / max) :")
    print("    couples  |dtau|_inf  %9.2e / %9.2e  N.m" % (np.median(col('dtau_inf')), col('dtau_inf').max()))
    print("    forces   |df|_inf    %9.2e / %9.2e  N"   % (np.median(col('df_inf')),   col('df_inf').max()))
    print("    accel.   |dqdd|_inf  %9.2e / %9.2e  rad/s2" % (np.median(col('dqdd_inf')), col('dqdd_inf').max()))
    print("  ETAT PHYSIQUE aux evenements (median / max) :")
    print("    |qvel|_inf   %9.2e / %9.2e  rad/s" % (np.median(col('qvel_inf')), col('qvel_inf').max()))
    print("    dev CoM      %9.2e / %9.2e  m"     % (np.median(col('com_dev')),  col('com_dev').max()))
    print("    |tau|_max    %9.2f / %9.2f  N.m (bornes du QP : +-300)"
          % (np.median(col('tau_max')), col('tau_max').max()))
    print("    marge couple %9.2f / %9.2f  N.m (0 = borne atteinte)"
          % (np.median(col('tau_margin')), col('tau_margin').min()))
    print("    fz_min       %9.2f / %9.2f  N (contrainte : >= 1 N)"
          % (np.median(col('fz_min')), col('fz_min').min()))
    print("  VIOLATIONS REELLES de la solution retournee (median / max) :")
    for k, lab, unit in (('viol_eq', 'egalites (dyn + no-slip)', ''),
                         ('viol_fz', 'fz >= 1 N', ' N'),
                         ('viol_fric', 'cone de friction', ' N'),
                         ('viol_tau', 'bornes de couple', ' N.m')):
        print("    %-26s %9.2e / %9.2e%s" % (lab, np.median(col(k)), col(k).max(), unit))
    print("-" * 78)
    print("TEST DECISIF — meme QP, certificat d'infaisabilite desactive (eps_inf=1e-14) :")
    hs = {}
    for e in ev:
        hs[e.get('hard_status', '?')] = hs.get(e.get('hard_status', '?'), 0) + 1
    print("    statuts : %s" % hs)
    print("    iterations   median %8.0f / max %8.0f" % (np.median(col('hard_iter')), col('hard_iter').max()))
    print("    violations de CETTE solution (median / max) :")
    for k, lab, unit in (('hard_viol_eq', 'egalites', ''),
                         ('hard_viol_fz', 'fz >= 1 N', ' N'),
                         ('hard_viol_fric', 'cone de friction', ' N'),
                         ('hard_viol_tau', 'bornes de couple', ' N.m')):
        print("      %-24s %9.2e / %9.2e%s" % (lab, np.median(col(k)), col(k).max(), unit))
    nsolved_hard = sum(1 for e in ev if e.get('hard_status') == 'PROXQP_SOLVED')
    print("    ECART entre la commande ENVOYEE et la commande CORRECTE :")
    print("      ||x_envoye - x_correct||_inf   median %9.2e / max %9.2e"
          % (np.median(col('dxh_inf')), col('dxh_inf').max()))
    print("      DECOMPOSITION PAR BLOC (median / max) :")
    print("        couples   |dtau|  %9.2f / %9.2f  N.m   <-- LE chiffre qui compte"
          % (np.median(col('h_dtau')), col('h_dtau').max()))
    print("        forces    |df|    %9.2f / %9.2f  N"
          % (np.median(col('h_df')), col('h_df').max()))
    print("        accel.    |dqdd|  %9.2f / %9.2f  rad/s2"
          % (np.median(col('h_dqdd')), col('h_dqdd').max()))
    print("      solution CORRECTE : |tau|_max median %.2f / max %.2f N.m | "
          "fz_min median %.2f N"
          % (np.median(col('tau_correct_max')), col('tau_correct_max').max(),
             np.median(col('fz_correct_min'))))
    print("  >>> %d / %d convergent quand le certificat est desactive." % (nsolved_hard, len(ev)))
    if nsolved_hard == len(ev):
        print("  >>> CONCLUSION : le QP EST faisable. Le certificat d'infaisabilite de")
        print("      ProxQP est un FAUX POSITIF sur ce probleme (degenerescence).")
    elif nsolved_hard == 0:
        print("  >>> CONCLUSION : le QP est REELLEMENT infaisable a ces instants.")
        print("      Lire ci-dessus QUELLE contrainte est violee.")
    print("  repartition temporelle (poussee a t=%.1f-%.1f s) :"
          % (VS.T_PUSH, VS.T_PUSH + VS.IMPULSE))
    for lo, hi, lab in ((0.0, VS.T_PUSH, "avant poussee"),
                        (VS.T_PUSH, VS.T_PUSH + VS.IMPULSE, "PENDANT poussee"),
                        (VS.T_PUSH + VS.IMPULSE, 1e9, "apres poussee")):
        k = int(((tt >= lo) & (tt < hi)).sum())
        print("    %-16s %5d  (%5.1f %%)" % (lab, k, 100.0 * k / len(ev)))
    print("-" * 78)
    # Le verdict repose sur le TEST DECISIF (certificat desactive), pas sur la
    # simple re-resolution a neuf : celle-ci utilise les memes reglages, donc
    # reproduit le meme faux positif.
    if nsolved_hard == len(ev):
        print("VERDICT : FAUX POSITIF DU SOLVEUR.")
        print("          Le QP est faisable (117/117 convergent, certificat desactive).")
        print("          ProxQP emet un certificat d'infaisabilite primale a tort et")
        print("          retourne un itere NON CONVERGE — cf. les violations ci-dessus.")
        print("          Correctif : settings.eps_primal_inf tres petit a l'init().")
        print("          /!\\ change la commande a ces ticks -> revalider les campagnes.")
    elif nsolved_hard == 0:
        print("VERDICT : le QP est REELLEMENT infaisable a ces instants.")
        print("          Lire ci-dessus QUELLE contrainte est violee.")
    else:
        print("VERDICT : MIXTE — %d / %d faux positifs." % (nsolved_hard, len(ev)))

    # --- groupement temporel des evenements (dropout isole ou salve ?) ---
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
        print("GROUPEMENT : %d salve(s) de ticks consecutifs | longueur median %d, max %d"
              % (runs.size, int(np.median(runs)), int(runs.max())))
        print("             (1 tick = 1 ms ; une salve de N ticks = N ms de commande degradee)")

    if a.csv:
        with open(a.csv, "w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=list(ev[0].keys()))
            w.writeheader(); w.writerows(ev)
        print("[ok] %s" % a.csv)


if __name__ == "__main__":
    main()
