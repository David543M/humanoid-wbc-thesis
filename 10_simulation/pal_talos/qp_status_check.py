"""
qp_status_check.py — verification du POINT 1 de l'audit QP (2026-08-11).

CONTEXTE
--------
`solve_qp` appelait `qp.solve()` puis retournait `qp.results.x` SANS jamais lire
`qp.results.info.status`. Or :
  * quadprog LEVAIT ValueError si le QP etait infaisable  -> repli declenche, compte ;
  * ProxQP NE LEVE RIEN : il retourne le dernier itere.
Le passage a ProxQP a donc supprime la detection d'echec. Le compteur `_qp_fail`
ne comptait plus que les exceptions Python -> la metrique « 0 repli / 100 % feasible »
de Ch5 etait aveugle aux non-convergences.

`talos_wbc.py` est desormais instrumente (lecture seule par defaut). Ce script
verifie DEUX choses, sans rien changer au controleur :

  A. NEUTRALITE  — les verdicts des essais rejoues sont identiques a ceux
                   enregistres dans runs/s1_corners_v2/results.csv (record 50/50).
  B. FAISABILITE — le solveur a bien retourne PROXQP_SOLVED a chaque tick,
                   avec residus primal/dual sous tolerance.

USAGE (terminal Anaconda, depuis pal_talos/)
--------------------------------------------
    python qp_status_check.py --n 3                  # 3 essais, rapide
    python qp_status_check.py --n 50                 # campagne complete
    python qp_status_check.py --n 3 --strict         # verifie que le mode strict
                                                     # ne change rien quand tout va bien

ARGUMENT SECONDAIRE (utile au viva)
-----------------------------------
Mesure sandbox 2026-08-11, sur un QP aux dimensions du votre (n=94, n_eq=50,
n_in=104) : une resolution NORMALE coute ~6 ms/iteration-set (99 iterations) ;
un QP primal-infaisable part a la limite de 10 000 iterations et n'avait pas
termine apres 200 s. Autrement dit, un evenement de non-convergence coute des
SECONDES, pas des millisecondes. Votre p99 de boucle mesure a 1.7 ms (Ch5)
est donc, a lui seul, une preuve indirecte qu'aucun tick n'a atteint la limite
d'iterations : un seul evenement aurait fait exploser la statistique de temps.
Ce script fournit la preuve DIRECTE.
"""
import argparse, csv, os, sys, time
import numpy as np

import talos_wbc as W
import validate_s1_batch as VS

CSV_REF = os.path.join("runs", "s1_corners_v2", "results.csv")
TOL = dict(z=2e-3, com_dev=2e-3)      # le CSV de reference est arrondi a 3-4 decimales


def load_ref(path):
    if not os.path.exists(path):
        return {}
    out = {}
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            out[int(row["seed"])] = row
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=3, help="nombre d'essais rejoues")
    ap.add_argument("--seed0", type=int, default=1000, help="1er seed (batch S1 = 1000)")
    ap.add_argument("--strict", action="store_true",
                    help="active WBC_QP_STRICT : une non-convergence leve et declenche le repli")
    ap.add_argument("--ref", default=CSV_REF)
    a = ap.parse_args()

    if a.strict:
        W.QP_STRICT = True
    print("=" * 78)
    print("QP STATUS CHECK — mode %s | %d essais depuis seed %d"
          % ("STRICT" if W.QP_STRICT else "observation", a.n, a.seed0))
    print("=" * 78)

    ref = load_ref(a.ref)
    if ref:
        print("[ref] %s : %d essais enregistres" % (a.ref, len(ref)))
    else:
        print("[ref] %s introuvable — controle de NEUTRALITE saute" % a.ref)

    W.qp_stats_reset()
    mismatches, rows = [], []
    t0 = time.perf_counter()
    for k in range(a.n):
        seed = a.seed0 + k
        r = VS.trial(seed)
        rows.append(r)
        tag = "ok"
        if seed in ref:
            g = ref[seed]
            bad = []
            if int(r["success"]) != int(g["success"]):   bad.append("success")
            if int(r["fell"]) != int(g["fell"]):         bad.append("fell")
            if int(r["qp_fails"]) != int(g["qp_fails"]): bad.append("qp_fails")
            for key in ("z", "com_dev"):
                if abs(float(r[key]) - float(g[key])) > TOL[key]:
                    bad.append("%s (%.4f vs %.4f)" % (key, float(r[key]), float(g[key])))
            if bad:
                mismatches.append((seed, bad)); tag = "DIVERGE: " + ", ".join(bad)
        print("  seed %d  F=%6.1f N  success=%d  fell=%d  z=%.3f  com_dev=%.4f  "
              "qp_fails=%d   [%s]"
              % (seed, r["F"], r["success"], r["fell"], r["z"], r["com_dev"],
                 r["qp_fails"], tag))
    wall = time.perf_counter() - t0

    print("-" * 78)
    print(W.qp_stats_report())
    print("-" * 78)

    s = W.QP_STATS
    ok_feas = (s["calls"] > 0 and s["not_solved"] == 0)
    ok_neut = (not ref) or (not mismatches)

    print("A. NEUTRALITE  : %s" % (
        "OK — verdicts identiques a la reference" if ok_neut else
        "ECHEC — %d essai(s) divergent : %s" % (len(mismatches), mismatches)))
    print("B. FAISABILITE : %s" % (
        "OK — %d/%d resolutions PROXQP_SOLVED, pri_res max %.2e"
        % (s["solved"], s["calls"], s["max_pri_res"]) if ok_feas else
        "ECHEC — %d non-convergence(s) sur %d : %s"
        % (s["not_solved"], s["calls"], s["statuses"])))
    print("   duree totale %.1f s pour %d essais (%.0f ticks)"
          % (wall, a.n, s["calls"]))
    print("=" * 78)
    if not ok_feas:
        print("\n>>> Le claim « 0 repli QP / 100 %% feasible » de Ch5 doit etre requalifie.")
        print(">>> Relancer en --strict pour que ces evenements comptent comme des replis.")
    sys.exit(0 if (ok_feas and ok_neut) else 1)


if __name__ == "__main__":
    main()
