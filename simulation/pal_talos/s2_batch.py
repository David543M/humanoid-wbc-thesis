"""
S2 — Batch of N seeds on the frozen config (P1, Ch5 lock).

Runs talos_dcm_walk_timing.py in a SUBPROCESS per seed (isolation: a fall
or a crash does not affect the other trials), aggregates the per-seed npz and produces:
  - success rate + 95 % Wilson interval (Table 3.1 criterion: lower bound > 0.90, N >= 20)
  - CoM RMSE / lateral drift / step durations (mean +/- std over the trials)
  - landing metrics (peak |w_foot|, peak GRF, make/break) — Ch5/Ch6 baseline
  - aggregated ctrl loop mean / p99
Outputs in OUTDIR: s2_seed<k>.npz + s2_seed<k>.log (stdout), s2_batch_summary.npz,
s2_batch_report.md, s2_batch_rows.csv.

Usage (Anaconda terminal, from pal_talos/):
    python s2_batch.py                      # 20 seeds (0..19), frozen S2 config
    python s2_batch.py --n 30 --start 0
    python s2_batch.py --outdir s2_batch_soft -- --softland   # A/B variant: args after --
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np

FROZEN = ["--steps", "70", "--offlat", "0.08", "--dsovl", "0.12", "--tmin", "0.24"]
SCRIPT = "talos_dcm_walk_timing.py"
TIMEOUT = 900          # s per trial (wide margin; a run is ~30-60 s)
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
    npz = os.path.join(outdir, "s2_seed%02d.npz" % seed)
    log = os.path.join(outdir, "s2_seed%02d.log" % seed)
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
    ap.add_argument("--n", type=int, default=20, help="number of trials (Table 3.1: N >= 20)")
    ap.add_argument("--start", type=int, default=0, help="first seed")
    ap.add_argument("--outdir", type=str, default="s2_batch")
    ap.add_argument("--thresh", type=float, default=0.90,
                    help="criterion: Wilson lower bound > thresh (Table 3.1)")
    ap.add_argument("extra", nargs=argparse.REMAINDER,
                    help="args passed to the walking script after -- (e.g.: -- --softland)")
    a = ap.parse_args()
    extra = [e for e in a.extra if e != "--"]
    # cmd.exe guard: comments '# ...' appended to the command are NOT
    # ignored by cmd -> they ended up in extra and crashed the child
    # script (batch 2026-07-08: artificial 0/20, trials of 0-1 s)
    for i, e in enumerate(extra):
        if e.startswith("#"):
            extra = extra[:i]; break
    junk = [e for i, e in enumerate(extra)
            if not e.startswith("-") and (i == 0 or not extra[i - 1].startswith("-"))]
    if junk:
        raise SystemExit("[ABORT] suspicious extra arguments (not options): %s" % junk)
    os.makedirs(a.outdir, exist_ok=True)
    seeds = list(range(a.start, a.start + a.n))
    print("S2 BATCH  N=%d seeds=%d..%d  frozen config %s  extra=%s  -> %s/"
          % (a.n, seeds[0], seeds[-1], " ".join(FROZEN), extra or "-", a.outdir))

    rows = []
    for s in seeds:
        npz_path, log_path, status, wall = run_seed(s, a.outdir, extra)
        row = dict(seed=s, status=status, wall_s=wall, success=False, dist=np.nan,
                   fell_at=np.nan, rmse_mm=np.nan, tstep_mean=np.nan,
                   wob_pic=np.nan, grf_pic=np.nan, mkbrk=np.nan,
                   ctrl_mean=np.nan, ctrl_p99=np.nan,
                   e_mech=np.nan, e_sq=np.nan, cot=np.nan)
        if status == "ok" and os.path.exists(npz_path):
            try:
                d = np.load(npz_path)
                ce, cm = d["com_err"], d["ctrl_ms"]
                row.update(success=bool(d["success"]), dist=float(d["dist"]),
                           fell_at=float(d["fell_at"]),
                           rmse_mm=float(np.sqrt(np.nanmean(ce ** 2)) * 1e3),
                           tstep_mean=float(d["t_steps"].mean()) if d["t_steps"].size else np.nan,
                           ctrl_mean=float(cm.mean()), ctrl_p99=float(np.percentile(cm, 99)))
                _em, _dd = float(d["e_mech"]), float(d["dist"])
                _ms = float(d["mass"])
                row.update(e_mech=_em, e_sq=float(d["e_sq"]),
                           cot=_em / (_ms * 9.81 * _dd) if _dd > 0.1 else np.nan)
                if d["land_wobble"].size:
                    row.update(wob_pic=float(d["land_wobble"].max()),
                               grf_pic=float(d["land_grf"].max()),
                               mkbrk=float(d["land_mkbrk"].mean()))
            except Exception as e:
                row["status"] = "npz_error:%s" % e
        elif status == "ok":                     # process exited without producing an npz = CRASH
            row["status"] = "crash"
        rows.append(row)
        print("  seed %2d : %-7s %s dist=%5.2f m  rmse=%5.1f mm  (%.0f s)"
              % (s, row["status"], "SUCCESS" if row["success"] else "fail   ",
                 row["dist"], row["rmse_mm"], wall))
        if row["status"] == "crash":
            try:
                tail = [l.rstrip() for l in open(log_path, encoding="utf-8", errors="replace")][-3:]
            except OSError:
                tail = ["(log unreadable)"]
            for l in tail:
                print("      | %s" % l)
            if s == seeds[0]:                    # 1st trial already crashing -> pointless to continue
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
    lines.append("# S2 batch — %d essais (seeds %d..%d)%s" %
                 (n, seeds[0], seeds[-1], "  extra: " + " ".join(extra) if extra else ""))
    lines.append("")
    lines.append("| Metrique | Valeur |")
    lines.append("|---|---|")
    lines.append("| Success rate | **%d/%d = %.1f %%** |" % (k, n, 100 * k / n))
    lines.append("| Wilson 95 %% CI | [%.3f, %.3f] |" % (lo, hi))
    lines.append("| Critere Table 3.1 (borne basse > %.2f) | **%s** |" % (a.thresh, verdict))
    for key, label, unit, sel in (
            ("dist", "Distance", "m", ok), ("rmse_mm", "CoM RMSE", "mm", ok),
            ("tstep_mean", "Duree de pas moyenne", "s", ok),
            ("wob_pic", "Pic |w_pied| au poser", "rad/s", rows),
            ("grf_pic", "Pic GRF au poser", "N", rows),
            ("mkbrk", "Make/break moyen au poser", "-", rows),
            ("ctrl_mean", "Ctrl loop mean", "ms", rows),
            ("ctrl_p99", "Ctrl loop p99", "ms", rows)):
        m, sd = mstd(key, sel)
        lines.append("| %s (%s, %s) | %.2f +/- %.2f |"
                     % (label, "succes seuls" if sel is ok else "tous", unit, m, sd))
    fails = [r for r in rows if not r["success"]]
    if fails:
        lines.append("")
        lines.append("Echecs : " + ", ".join(
            "seed %d (%s, t=%.1f s)" % (r["seed"], r["status"], r["fell_at"]) for r in fails))
    report = "\n".join(lines)
    print("\n" + report)

    with open(os.path.join(a.outdir, "s2_batch_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    with open(os.path.join(a.outdir, "s2_batch_rows.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    np.savez(os.path.join(a.outdir, "s2_batch_summary.npz"),
             seeds=np.array(seeds), success=np.array([r["success"] for r in rows]),
             k=k, n=n, wilson_low=lo, wilson_high=hi, thresh=a.thresh,
             verdict=verdict, extra=" ".join(extra))
    print("\n[ok] %s/s2_batch_report.md  +  s2_batch_rows.csv  +  s2_batch_summary.npz" % a.outdir)


if __name__ == "__main__":
    main()
