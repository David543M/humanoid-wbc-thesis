"""
S2 — Determinism test (P1d, reproducibility caveat Ch5 §5.5 / Ch6).

Question: does the per-seed non-determinism observed between the N=20 and N=50 batches
(same frozen config, same seed -> different outcomes; e.g. seed 3 SUCCESS 3.59 m
then FELL 1.34 m) come from the floating-point non-associativity of the multi-threaded BLAS,
or is it intrinsic (thin-margin chaos, another entropy source)?

Protocol: for each seed, re-run the walk R times (default 2) under TWO
threading regimes, in SUBPROCESSES (the env must be set BEFORE the child's
numpy/mujoco import):
  - MULTI  : default threading (what the batches used)
  - SINGLE : BLAS forced single-threaded (OMP/MKL/OPENBLAS/NUMEXPR/VECLIB = 1,
             PYTHONHASHSEED=0)
Then compare the com_err (and xi_err) trajectories tick by tick between repetitions
of the same (seed, regime).

Reading the results:
  - MULTI  identical   AND SINGLE identical  -> already deterministic (surprising;
                                               the non-determinism came from elsewhere)
  - MULTI  divergent   AND SINGLE identical  -> HYPOTHESIS CONFIRMED: the multi-threaded
                                               BLAS is the cause;
                                               single-thread restores determinism
  - MULTI  divergent   AND SINGLE divergent  -> INTRINSIC non-determinism
                                               (thin-margin chaos / other entropy)
                                               -> Ch6 argument even stronger

The divergence is characterised by: bit-for-bit identical? first divergence tick,
max deviation, and macro outcome (dist / fell_at / success). A very early first
tick ~ machine epsilon that grows = chaotic amplification; a
divergence from tick 0 = initialisation entropy.

Usage (Anaconda terminal, from pal_talos/):
    python s2_determinism.py                       # seeds 0 2 3, R=2, ~10 min
    python s2_determinism.py --seeds 0 1 2 3 --repeats 3
Outputs: determinism_out/  (npz per run + determinism_report.md)
"""
import argparse, os, subprocess, sys, time
import numpy as np

FROZEN = ["--steps", "70", "--offlat", "0.08", "--dsovl", "0.12", "--tmin", "0.24"]
SCRIPT = "talos_dcm_walk_timing.py"
TIMEOUT = 900

# Variables read at import time by numpy / MKL / OpenBLAS / OpenMP: set in
# the CHILD's env (setting them in this process would be too late).
SINGLE_ENV = {
    "OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1", "OPENBLAS_NUM_THREADS": "1",
    "NUMEXPR_NUM_THREADS": "1", "VECLIB_MAXIMUM_THREADS": "1",
    "MKL_DYNAMIC": "FALSE", "OMP_DYNAMIC": "FALSE", "PYTHONHASHSEED": "0",
}


def run(seed, regime, rep, outdir):
    npz = os.path.join(outdir, "det_s%02d_%s_r%d.npz" % (seed, regime, rep))
    log = os.path.join(outdir, "det_s%02d_%s_r%d.log" % (seed, regime, rep))
    env = dict(os.environ)
    if regime == "single":
        env.update(SINGLE_ENV)
    cmd = [sys.executable, SCRIPT] + FROZEN + ["--seed", str(seed), "--save", npz]
    t0 = time.time()
    with open(log, "w", encoding="utf-8") as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, env=env,
                       timeout=TIMEOUT, check=False)
    return npz, time.time() - t0


def outcome(npz):
    if not os.path.exists(npz):
        return None
    d = np.load(npz, allow_pickle=True)
    return dict(com=d["com_err"], xi=d["xi_err"], dist=float(d["dist"]),
                fell=float(d["fell_at"]), succ=bool(d["success"]))


def compare(a, b):
    """Compare two com_err trajectories: (identical?, 1st divergent tick, max deviation)."""
    if a is None or b is None:
        return dict(identical=False, first=-1, maxabs=float("nan"), note="missing run")
    na, nb = len(a["com"]), len(b["com"])
    m = min(na, nb)
    ca, cb = a["com"][:m], b["com"][:m]
    if na == nb and np.array_equal(a["com"], b["com"]):
        return dict(identical=True, first=-1, maxabs=0.0, note="bit-identical")
    diff = np.abs(ca - cb)
    nz = np.nonzero(diff > 0)[0]
    first = int(nz[0]) if nz.size else -1
    note = "different lengths (%d vs %d)" % (na, nb) if na != nb else "different values"
    return dict(identical=False, first=first, maxabs=float(diff.max()), note=note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 2, 3])
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--outdir", type=str, default="determinism_out")
    ap.add_argument("--regimes", type=str, nargs="+", default=["multi", "single"])
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    print("DETERMINISM  seeds=%s  repeats=%d  regimes=%s  config=%s"
          % (a.seeds, a.repeats, a.regimes, " ".join(FROZEN)))

    data = {}       # (seed, regime, rep) -> outcome
    for regime in a.regimes:
        for seed in a.seeds:
            for rep in range(a.repeats):
                npz, wall = run(seed, regime, rep, a.outdir)
                o = outcome(npz)
                data[(seed, regime, rep)] = o
                tag = ("dist=%.3f fell=%.1f %s" % (o["dist"], o["fell"],
                       "SUCCESS" if o["succ"] else "fail")) if o else "CRASH (no npz)"
                print("  seed %2d %-6s r%d : %-32s (%.0f s)" % (seed, regime, rep, tag, wall))

    # ---------- comparison rep0 vs rep1.. ----------
    lines = ["# S2 — Determinism test", "",
             "Frozen config: `%s`" % " ".join(FROZEN),
             "Repetitions compared against rep0. `identical` = com_err equal bit-for-bit.", ""]
    lines += ["| seed | regime | rep | identical | 1st div. tick | max deviation (m) | outcome | note |",
              "|---|---|---|---|---|---|---|---|"]
    regime_identical = {r: True for r in a.regimes}
    for regime in a.regimes:
        for seed in a.seeds:
            base = data.get((seed, regime, 0))
            b_iss = ("%.3f/%s" % (base["dist"], "S" if base["succ"] else "F")) if base else "—"
            lines.append("| %d | %s | 0 | (ref) | — | — | %s | — |" % (seed, regime, b_iss))
            for rep in range(1, a.repeats):
                cur = data.get((seed, regime, rep))
                c = compare(base, cur)
                if not c["identical"]:
                    regime_identical[regime] = False
                iss = ("%.3f/%s" % (cur["dist"], "S" if cur["succ"] else "F")) if cur else "—"
                lines.append("| %d | %s | %d | %s | %s | %.2e | %s | %s |" % (
                    seed, regime, rep, "YES" if c["identical"] else "**NO**",
                    c["first"] if c["first"] >= 0 else "—", c["maxabs"], iss, c["note"]))

    # ---------- verdict ----------
    lines += ["", "## Verdict"]
    multi_det = regime_identical.get("multi", None)
    single_det = regime_identical.get("single", None)
    if multi_det and single_det:
        v = "Both regimes are deterministic -> the non-determinism observed between batches came from ELSEWHERE (different config between runs? to be investigated)."
    elif (multi_det is False) and single_det:
        v = "**HYPOTHESIS CONFIRMED**: multi-thread divergent, single-thread identical -> the multi-threaded BLAS (floating-point non-associativity) is the cause. Per-seed determinism is recoverable by forcing OMP/MKL=1. -> Recommend single-threaded execution for any reported campaign (Ch5) + document it (Ch6)."
    elif (multi_det is False) and (single_det is False):
        v = "**INTRINSIC NON-DETERMINISM**: divergence persists even single-threaded -> the entropy is not (only) the BLAS. Likely cause: chaotic sensitivity of the thin-margin limit cycle. -> The reproducible quantity remains the AGGREGATE RATE; Ch6 argument reinforced (the config operates at the edge of stability)."
    else:
        v = "Mixed / incomplete result — see the table."
    lines += [v, "",
              "Reminder: divergent macro outcome already observed between N=20 and N=50 "
              "(seed 3 SUCCESS 3.59 m -> FELL 1.34 m; seed 2 3.72 -> 3.16 m; seed 0 3.37 -> 3.42 m)."]

    report = "\n".join(lines)
    print("\n" + report)
    with open(os.path.join(a.outdir, "determinism_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    print("\n[ok] %s/determinism_report.md" % a.outdir)


if __name__ == "__main__":
    main()
