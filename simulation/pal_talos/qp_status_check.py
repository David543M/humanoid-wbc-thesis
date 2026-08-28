"""
qp_status_check.py — verification of POINT 1 of the QP audit (2026-08-11).

CONTEXT
-------
`solve_qp` called `qp.solve()` then returned `qp.results.x` WITHOUT ever reading
`qp.results.info.status`. However:
  * quadprog RAISED ValueError when the QP was infeasible -> fallback triggered, counted;
  * ProxQP RAISES NOTHING: it returns the last iterate.
The move to ProxQP therefore removed failure detection. The `_qp_fail` counter
only counted Python exceptions any more -> the "0 fallbacks / 100 % feasible" metric
of Ch5 was blind to non-convergences.

`talos_wbc.py` is now instrumented (read-only by default). This script
checks TWO things, without changing anything in the controller:

  A. NEUTRALITY  — the verdicts of the replayed trials are identical to those
                   recorded in runs/s1_corners_v2/results.csv (record 50/50).
  B. FEASIBILITY — the solver did return PROXQP_SOLVED at every tick,
                   with primal/dual residuals under tolerance.

USAGE (Anaconda terminal, from pal_talos/)
-------------------------------------------
    python qp_status_check.py --n 3                  # 3 trials, quick
    python qp_status_check.py --n 50                 # full campaign
    python qp_status_check.py --n 3 --strict         # check that strict mode
                                                     # changes nothing when all is well

SECONDARY OBSERVATION
--------------------
Sandbox measurement 2026-08-11, on a QP with the dimensions of the controller's (n=94, n_eq=50,
n_in=104): a NORMAL solve costs ~6 ms/iteration-set (99 iterations);
a primal-infeasible QP runs to the limit of 10,000 iterations and had not
finished after 200 s. In other words, a non-convergence event costs
SECONDS, not milliseconds. The loop p99 measured at 1.7 ms (Ch5)
is therefore, on its own, indirect proof that no tick hit the iteration
limit: a single event would have blown up the timing statistics.
This script provides the DIRECT proof.
"""
import argparse, csv, os, sys, time
import numpy as np

import talos_wbc as W
import validate_s1_batch as VS

CSV_REF = os.path.join("runs", "s1_corners_v2", "results.csv")
TOL = dict(z=2e-3, com_dev=2e-3)      # the reference CSV is rounded to 3-4 decimals


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
    ap.add_argument("--n", type=int, default=3, help="number of trials replayed")
    ap.add_argument("--seed0", type=int, default=1000, help="first seed (S1 batch = 1000)")
    ap.add_argument("--strict", action="store_true",
                    help="enable WBC_QP_STRICT: a non-convergence raises and triggers the fallback")
    ap.add_argument("--ref", default=CSV_REF)
    a = ap.parse_args()

    if a.strict:
        W.QP_STRICT = True
    print("=" * 78)
    print("QP STATUS CHECK — %s mode | %d trials from seed %d"
          % ("STRICT" if W.QP_STRICT else "observation", a.n, a.seed0))
    print("=" * 78)

    ref = load_ref(a.ref)
    if ref:
        print("[ref] %s : %d trials recorded" % (a.ref, len(ref)))
    else:
        print("[ref] %s not found — NEUTRALITY check skipped" % a.ref)

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
                mismatches.append((seed, bad)); tag = "DIVERGED: " + ", ".join(bad)
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

    print("A. NEUTRALITY  : %s" % (
        "OK — verdicts identical to the reference" if ok_neut else
        "FAILED — %d trial(s) diverge: %s" % (len(mismatches), mismatches)))
    print("B. FEASIBILITY : %s" % (
        "OK — %d/%d solves PROXQP_SOLVED, max pri_res %.2e"
        % (s["solved"], s["calls"], s["max_pri_res"]) if ok_feas else
        "FAILED — %d non-convergence(s) out of %d: %s"
        % (s["not_solved"], s["calls"], s["statuses"])))
    print("   total duration %.1f s for %d trials (%.0f ticks)"
          % (wall, a.n, s["calls"]))
    print("=" * 78)
    if not ok_feas:
        print("\n>>> The Ch5 claim \"0 QP fallbacks / 100 %% feasible\" must be requalified.")
        print(">>> Re-run with --strict so these events count as fallbacks.")
    sys.exit(0 if (ok_feas and ok_neut) else 1)


if __name__ == "__main__":
    main()
