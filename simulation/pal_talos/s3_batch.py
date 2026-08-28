"""
S3 — Batch of N seeds on the quasi-static sequencer (frozen config QS #5c + polish 6a).

MOTIVATION (2026-08-06): Ch5 par.5.6 reported the 3/3 climb from a SINGLE
run. Since the thesis's declared contribution is the batch protocol with a Wilson
interval, a scenario reported from a single run does not satisfy it. This batch applies the
same protocol to S3, whether the result confirms the capability or turns it into a
quantified limit.

Runs talos_s3_stairs_qs.py in a SUBPROCESS per seed. Per-process isolation is
NOT a convenience detail: the scripts mutate shared globals (talos_wbc.MODEL,
gains) and any chaining within a single interpreter is invalid (cross-cutting finding
2026-07-30, inter-scenario state contamination). Same pattern as s2_batch.py /
s4_batch.py, which are therefore unaffected.

PRE-REGISTERED VERDICT (identical to the script's criterion, no latitude after the fact):
    success = UPRIGHT  AND  3/3 steps reached  AND  final state DONE
The initial noise is the same as S2 (qvel sigma=0.01 via default_rng(seed)), so the
aggregate rate is directly comparable to the 70 % of S2.

Outputs in OUTDIR: s3_seed<k>.npz + s3_seed<k>.log, s3_batch_summary.npz,
s3_batch_report.md, s3_batch_rows.csv.

Usage (Anaconda terminal, from pal_talos/):
    python s3_batch.py                       # 20 seeds (0..19), frozen S3-QS config
    python s3_batch.py --n 30 --start 0      # confirmatory (N >= 35 for a PASS verdict)
    python s3_batch.py --outdir s3_batch_h15 -- --hriser 0.15   # variant: args after --
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np

# Frozen config = the defaults of the 3/3 success (QS #5c + w_foot_stance=1400), passed
# EXPLICITLY so the report is self-documenting and replayable without reading the script.
FROZEN = ["--rate", "0.18", "--tswing", "0.65", "--ttrmax", "3.0",
          "--postol", "0.035", "--clear", "0.14", "--hriser", "0.10"]
SCRIPT = "talos_s3_stairs_qs.py"
TIMEOUT = 1800         # s per trial (the QS climb is slow: ~25 s of sim, wide margin)
Z = 1.959963984540054  # 95 %


def wilson(k, n, z=Z):
    """95 % Wilson interval for k successes out of n trials -> (low, high)."""
    if n == 0:
        return 0.0, 1.0
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def run_seed(seed, outdir, extra):
    npz = os.path.join(outdir, "s3_seed%02d.npz" % seed)
    log = os.path.join(outdir, "s3_seed%02d.log" % seed)
    cmd = [sys.executable, SCRIPT] + FROZEN + ["--seed", str(seed), "--save", npz] + extra
    t0 = time.time()
    try:
        with open(log, "w", encoding="utf-8") as f:
            subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=TIMEOUT, check=False)
        status = "ok"
    except subprocess.TimeoutExpired:
        status = "timeout"
    return npz, log, status, time.time() - t0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=20, help="number of trials (Table 3.1: N >= 20 pilot)")
    ap.add_argument("--start", type=int, default=0, help="first seed")
    ap.add_argument("--outdir", type=str, default="s3_batch")
    ap.add_argument("--thresh", type=float, default=0.90,
                    help="criterion: Wilson lower bound > thresh (Table 3.1)")
    ap.add_argument("extra", nargs=argparse.REMAINDER,
                    help="args passed to the sequencer after -- (e.g.: -- --hriser 0.15)")
    a = ap.parse_args()
    extra = [e for e in a.extra if e != "--"]
    # cmd.exe guard (taken from s2_batch.py): a comment '# ...' appended to the
    # command is NOT ignored by cmd -> it ends up in extra and crashes the child
    # script (batch 2026-07-08: artificial 0/20, trials of 0-1 s).
    for i, e in enumerate(extra):
        if e.startswith("#"):
            extra = extra[:i]; break
    junk = [e for i, e in enumerate(extra)
            if not e.startswith("-") and (i == 0 or not extra[i - 1].startswith("-"))]
    if junk:
        raise SystemExit("[ABORT] suspicious extra arguments (not options): %s" % junk)
    os.makedirs(a.outdir, exist_ok=True)
    seeds = list(range(a.start, a.start + a.n))
    print("S3 BATCH  N=%d seeds=%d..%d  frozen config %s  extra=%s  -> %s/"
          % (a.n, seeds[0], seeds[-1], " ".join(FROZEN), extra or "-", a.outdir))
    print("  pre-registered verdict: UPRIGHT and 3/3 steps and final state DONE")
    # Ch5 par.5.2 rule: the verdict is the Wilson LOWER BOUND, not the point estimate.
    # Arithmetic consequence: at N=20, even 20/20 only gives LB=0.839 -> a pilot
    # batch can FAIL but never PASS. N=35 is the first N at which 35/35 clears
    # 0.90 (LB=0.901). Running N<35 hoping for a PASS is a waste of time.
    if a.n < 35 and a.thresh >= 0.90:
        print("  [!] N=%d: a PASS verdict is ARITHMETICALLY IMPOSSIBLE (35/35 -> LB=0.901"
              " is the first favourable case). This batch is a PILOT: it can falsify,"
              " not validate. Use --n 35 (or more) for a confirmatory batch." % a.n)

    rows = []
    for s in seeds:
        npz_path, log_path, status, wall = run_seed(s, a.outdir, extra)
        row = dict(seed=s, status=status, wall_s=wall, success=False, upright=False,
                   reached_top=False, state="?", n_climbed=-1, mi=-1, n_moves=-1,
                   dist=np.nan, climb=np.nan, fell_at=np.nan, grf_pic=np.nan,
                   clear_min=np.nan, ctrl_mean=np.nan, ctrl_p99=np.nan, qp_feas=np.nan,
                   e_mech=np.nan, e_sq=np.nan, cot=np.nan)
        if status == "ok" and os.path.exists(npz_path):
            try:
                d = np.load(npz_path, allow_pickle=False)
                cm = d["ctrl_ms"]
                # purge timer artefacts (cf. methodology 2026-07-15: outlier values
                # up to 1e7 ms pollute the stats if not filtered)
                cmv = cm[np.isfinite(cm) & (cm < 1000.0)]
                nq, nt = int(d["qp_fail"]), max(int(d["n_ticks"]), 1)
                row.update(success=bool(d["success"]), upright=bool(d["upright"]),
                           reached_top=bool(d["reached_top"]), state=str(d["state"]),
                           n_climbed=int(d["n_climbed"]), mi=int(d["mi"]),
                           n_moves=int(d["n_moves"]), dist=float(d["dist"]),
                           climb=float(d["climb"]), fell_at=float(d["fell_at"]),
                           grf_pic=float(np.nanmax(d["grf"])),
                           clear_min=float(np.nanmin(d["clear"])),
                           ctrl_mean=float(cmv.mean()) if cmv.size else np.nan,
                           ctrl_p99=float(np.percentile(cmv, 99)) if cmv.size else np.nan,
                           qp_feas=100.0 * (1.0 - nq / nt))
                _em, _dd = float(d["e_mech"]), float(d["dist"])
                _ms = float(d["mass"])
                row.update(e_mech=_em, e_sq=float(d["e_sq"]),
                           cot=_em / (_ms * 9.81 * _dd) if _dd > 0.1 else np.nan)
            except Exception as e:
                row["status"] = "npz_error:%s" % e
        elif status == "ok":                     # process exited without an npz = CRASH
            row["status"] = "crash"
        rows.append(row)
        print("  seed %2d : %-7s %s  %d/3 steps  mi=%2d/%2d  dist=%5.2f m  (%.0f s)"
              % (s, row["status"], "SUCCESS" if row["success"] else "fail   ",
                 max(row["n_climbed"], 0), row["mi"], row["n_moves"], row["dist"], wall))
        if row["status"] == "crash":
            try:
                tail = [l.rstrip() for l in open(log_path, encoding="utf-8", errors="replace")][-3:]
            except OSError:
                tail = ["(log unreadable)"]
            for l in tail:
                print("      | %s" % l)
            if s == seeds[0]:
                raise SystemExit("[ABORT] the 1st trial crashed — fix before re-running (log: %s)" % log_path)

    # ---------- aggregation ----------
    n = len(rows); k = sum(r["success"] for r in rows)
    lo, hi = wilson(k, n)
    ok = [r for r in rows if r["success"]]
    def mstd(key, sel):
        v = np.array([r[key] for r in sel], dtype=float)
        v = v[~np.isnan(v)]
        return (v.mean(), v.std()) if v.size else (np.nan, np.nan)
    verdict = "PASS" if lo > a.thresh else "FAIL"
    lines = []
    lines.append("# S3 batch — %d essais (seeds %d..%d)%s" %
                 (n, seeds[0], seeds[-1], "  extra: " + " ".join(extra) if extra else ""))
    lines.append("")
    lines.append("Config gelee : `%s`" % " ".join(FROZEN))
    lines.append("")
    lines.append("Verdict pre-enregistre : UPRIGHT et 3/3 marches et etat final DONE.")
    lines.append("")
    lines.append("| Metrique | Valeur |")
    lines.append("|---|---|")
    lines.append("| Success rate | **%d/%d = %.1f %%** |" % (k, n, 100 * k / n))
    lines.append("| Wilson 95 %% CI | [%.3f, %.3f] |" % (lo, hi))
    lines.append("| Critere Table 3.1 (borne basse > %.2f) | **%s** |" % (a.thresh, verdict))
    for key, label, unit, sel in (
            ("dist", "Avancee", "m", ok), ("climb", "Gain CoM z", "m", ok),
            ("grf_pic", "Pic GRF", "N", rows),
            ("ctrl_mean", "Ctrl loop mean", "ms", rows),
            ("ctrl_p99", "Ctrl loop p99", "ms", rows),
            ("qp_feas", "QP feasible", "%", rows)):
        m, sd = mstd(key, sel)
        lines.append("| %s (%s, %s) | %.2f +/- %.2f |"
                     % (label, "succes seuls" if sel is ok else "tous", unit, m, sd))
    # distribution of the number of steps: distinguishes "fails early" from "fails at the top",
    # which do not read the same way (the failure mode of the 3/3 run was the final settle)
    lines.append("")
    lines.append("Marches atteintes : " + ", ".join(
        "%d/3 -> %d essai(s)" % (v, sum(1 for r in rows if r["n_climbed"] == v))
        for v in sorted({r["n_climbed"] for r in rows if r["n_climbed"] >= 0})))
    fails = [r for r in rows if not r["success"]]
    if fails:
        lines.append("")
        lines.append("Echecs : " + ", ".join(
            "seed %d (%s, %d/3, etat=%s, t=%.1f s)"
            % (r["seed"], r["status"], max(r["n_climbed"], 0), r["state"], r["fell_at"])
            for r in fails))
    lines.append("")
    lines.append("> WARNING on the clearance metric: the sequencer's `_clear_tick` logger")
    lines.append("> reports 0 crossings over a complete run (backlog polish item P4,")
    lines.append("> not fixed - touching the swing had regressed the climb three times). Swing")
    lines.append("> clearance is therefore NOT quantifiable from this batch; do not report it.")
    report = "\n".join(lines)
    print("\n" + report)

    with open(os.path.join(a.outdir, "s3_batch_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    with open(os.path.join(a.outdir, "s3_batch_rows.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    np.savez(os.path.join(a.outdir, "s3_batch_summary.npz"),
             seeds=np.array(seeds), success=np.array([r["success"] for r in rows]),
             n_climbed=np.array([r["n_climbed"] for r in rows]),
             k=k, n=n, wilson_low=lo, wilson_high=hi, thresh=a.thresh,
             verdict=verdict, extra=" ".join(extra))
    print("\n[ok] %s/s3_batch_report.md  +  s3_batch_rows.csv  +  s3_batch_summary.npz" % a.outdir)


if __name__ == "__main__":
    main()
