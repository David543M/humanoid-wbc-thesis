"""
S5 — Balayage magnitude x direction + bras controle (spec : s5_design.md §6).

Pattern of s2_batch.py: one subprocess per trial (isolation), Wilson aggregation
PER CELL (mag, dir), mag=0 control arm (same seeds, same virtual window),
per-direction I50 interpolation (impulse at 50 % recovery) = the calibration
deliverable that closes the Ch3 §3.6 flag.

Per-cell statistic: recovery rate = recovered / eligible (trials
debout a t_push). Les essais non eligibles (chute pre-push, heritage S2
~30 %) are counted separately - they belong to S2, already characterised.

Usage (terminal Anaconda, depuis pal_talos/) :
    python s5_batch.py                          # 4 dirs x {0,50..250} N x 10 seeds
    python s5_batch.py --n 20 --mags 100 150    # densification
    python s5_batch.py --dirs +y -y             # lateral seulement
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np

# dashless aliases (argparse swallows bare "-y"/"-x" - see talos_s5_perturb)
DIR_ALIAS = {"px": "+x", "mx": "-x", "py": "+y", "my": "-y",
             "x": "+x", "y": "+y", "x+": "+x", "y+": "+y",
             "x-": "-x", "y-": "-y"}
DIR_OK = ("+x", "-x", "+y", "-y")


def norm_dir(s):
    s = DIR_ALIAS.get(s.strip().lower(), s.strip().lower())
    if s not in DIR_OK:
        raise SystemExit("[ABORT] direction invalide : %r (attendu +x/-x/+y/-y "
                         "ou alias px/mx/py/my)" % s)
    return s


SCRIPT = "talos_s5_perturb.py"
TIMEOUT = 900
Z = 1.959963984540054


def wilson(k, n, z=Z):
    if n == 0:
        return 0.0, 1.0
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def run_trial(seed, mag, dr, outdir, extra):
    tag = "s5_m%03d_%s_seed%02d" % (int(mag), dr.replace("+", "p").replace("-", "m"), seed)
    npz = os.path.join(outdir, tag + ".npz")
    log = os.path.join(outdir, tag + ".log")
    # forme --pushdir=... : argparse avale un "-y" passe en argument separe
    cmd = [sys.executable, SCRIPT, "--seed", str(seed), "--pushmag", str(mag),
           "--pushdir=%s" % dr, "--save", npz] + extra
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
    ap.add_argument("--n", type=int, default=10, help="seeds per cell (>=10; 20 near I50)")
    ap.add_argument("--start", type=int, default=0)
    ap.add_argument("--mags", type=float, nargs="+", default=[50, 100, 150, 200, 250])
    ap.add_argument("--dirs", type=str, nargs="+", default=["+x", "-x", "+y", "-y"])
    ap.add_argument("--no-control", dest="control", action="store_false",
                    help="without the mag=0 control arm (NOT RECOMMENDED, see s5_design §2)")
    ap.add_argument("--outdir", type=str, default="s5_batch")
    ap.add_argument("extra", nargs=argparse.REMAINDER,
                    help="args passes au script apres -- (ex : -- --trec 5.0)")
    a = ap.parse_args()
    a.dirs = [norm_dir(d_) for d_ in a.dirs]
    extra = [e for e in a.extra if e != "--"]
    for i, e in enumerate(extra):            # garde-fou cmd.exe (cf. s2_batch 2026-07-08)
        if e.startswith("#"):
            extra = extra[:i]; break
    junk = [e for i, e in enumerate(extra)
            if not e.startswith("-") and (i == 0 or not extra[i - 1].startswith("-"))]
    if junk:
        raise SystemExit("[ABORT] arguments extra suspects : %s" % junk)
    os.makedirs(a.outdir, exist_ok=True)
    seeds = list(range(a.start, a.start + a.n))
    # the control arm is a single cell (mag 0) shared by every direction
    cells = ([(0.0, a.dirs[0])] if a.control else []) + \
            [(m_, d_) for d_ in a.dirs for m_ in a.mags]
    print("S5 BATCH  %d cellules x %d seeds (%d essais)  dirs=%s mags=%s  -> %s/"
          % (len(cells), a.n, len(cells) * a.n, a.dirs, a.mags, a.outdir))

    rows = []
    for (mag, dr) in cells:
        for s in seeds:
            npz_path, log_path, status, wall = run_trial(s, mag, dr, a.outdir, extra)
            row = dict(mag=mag, dir=dr, seed=s, status=status, wall_s=wall,
                       applied=False, eligible=False, recovered=False,
                       xi_max_post=np.nan, t_reband=np.nan, n_recov_post=np.nan,
                       dist=np.nan, fell_at=np.nan, success=False)
            if status == "ok" and os.path.exists(npz_path):
                try:
                    v = np.load(npz_path, allow_pickle=True)
                    row.update(applied=bool(v["push_applied"]),
                               eligible=bool(v["eligible"]),
                               recovered=bool(v["recovered"]),
                               xi_max_post=float(v["xi_max_post"]),
                               t_reband=float(v["t_reband"]),
                               n_recov_post=float(v["n_recov_post"]),
                               dist=float(v["dist"]), fell_at=float(v["fell_at"]),
                               success=bool(v["success"]))
                except Exception as e:
                    row["status"] = "npz_error:%s" % e
            elif status == "ok":
                row["status"] = "crash"
            rows.append(row)
            print("  m=%3.0f %s seed %2d : %-7s elig=%d rec=%d xi_max=%s (%.0f s)"
                  % (mag, dr, s, row["status"], row["eligible"], row["recovered"],
                     "%4.0fmm" % (row["xi_max_post"] * 1e3)
                     if np.isfinite(row["xi_max_post"]) else "  n/a", wall))
            if row["status"] == "crash" and (mag, dr) == cells[0] and s == seeds[0]:
                raise SystemExit("[ABORT] 1er essai en crash — corriger avant de "
                                 "relancer (log : %s)" % log_path)

    # ---------- per-cell aggregation ----------
    lines = ["# S5 batch — %d seeds/cellule (seeds %d..%d)%s" %
             (a.n, seeds[0], seeds[-1], "  extra: " + " ".join(extra) if extra else ""),
             "",
             "| Mag (N) | Dir | Eligibles | Recovered | Rate | Wilson 95% |",
             "|---|---|---|---|---|---|"]
    cell_stats = {}
    for (mag, dr) in cells:
        sel = [r for r in rows if r["mag"] == mag and r["dir"] == dr]
        el = [r for r in sel if r["eligible"]]
        k = sum(r["recovered"] for r in el)
        n = len(el)
        lo, hi = wilson(k, n)
        cell_stats[(mag, dr)] = (k, n, lo, hi)
        lines.append("| %.0f | %s | %d/%d | %d | %s | [%.3f, %.3f] |"
                     % (mag, dr if mag > 0 else "ctrl", n, len(sel), k,
                        "%.2f" % (k / n) if n else "n/a", lo, hi))

    # ---------- I50 per direction (linear interpolation) ----------
    lines += ["", "## Marge I50 (impulsion a 50 % de recuperation, N.s)", ""]
    dur = 0.10  # defaut --pushdur ; adapter si surcharge via extra
    for dr in a.dirs:
        pts = sorted((m_, cell_stats[(m_, dr)][0] / cell_stats[(m_, dr)][1])
                     for m_ in a.mags if cell_stats.get((m_, dr), (0, 0))[1] > 0)
        i50 = None
        for (m1, r1), (m2, r2) in zip(pts, pts[1:]):
            if (r1 - 0.5) * (r2 - 0.5) <= 0 and r1 != r2:
                i50 = (m1 + (0.5 - r1) * (m2 - m1) / (r2 - r1)) * dur
                break
        if i50 is not None:
            lines.append("- **%s : I50 ~ %.1f N.s** (F50 ~ %.0f N)" % (dr, i50, i50 / dur))
        elif pts and all(r > 0.5 for _, r in pts):
            lines.append("- %s: I50 > %.1f N.s (all cells > 50 %% - extend the sweep)"
                         % (dr, max(m_ for m_, _ in pts) * dur))
        elif pts:
            lines.append("- %s: I50 < %.1f N.s (all cells < 50 %% - reduce the sweep)"
                         % (dr, min(m_ for m_, _ in pts) * dur))
        else:
            lines.append("- %s : aucune cellule eligible" % dr)

    report = "\n".join(lines)
    print("\n" + report)
    with open(os.path.join(a.outdir, "s5_batch_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    with open(os.path.join(a.outdir, "s5_batch_rows.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    np.savez(os.path.join(a.outdir, "s5_batch_summary.npz"),
             mags=np.array(a.mags), dirs=np.array(a.dirs), n=a.n,
             cells=np.array([(m_, d_) for (m_, d_) in cells], dtype=object),
             stats=np.array([cell_stats[c] for c in cells]))
    print("\n[ok] %s/s5_batch_report.md  +  s5_batch_rows.csv  +  s5_batch_summary.npz" % a.outdir)


if __name__ == "__main__":
    main()
