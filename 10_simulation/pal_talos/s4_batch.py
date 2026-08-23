"""
S4 — Batch N seeds sur la config gelee (QS loco-manipulation, port bi-manuel).

Lance talos_s4_qs_carry.py en SOUS-PROCESSUS par seed (isolation : une chute ou un
crash n'affecte pas les autres essais), agrege les npz par seed et produit :
  - success rate FONCTIONNEL (debout + 3 m + DONE + boite portee) + Wilson 95 % CI
    -> c'est le verdict H1 (loco-manipulation quasi-statique reussie)
  - success rate STRICT (idem + EE pic < 5 cm) pour reference
  - EE error RMSE / pic en regime marche (metrique de manipulation, the thesis)
  - CoM tracking RMSE (metrique de locomotion, comparable a S2)
  - taux boite-portee, GRF pic, ctrl loop mean / p99 agreges
Sorties dans OUTDIR : s4_seed<k>.npz + s4_seed<k>.log, s4_batch_summary.npz,
s4_batch_report.md, s4_batch_rows.csv.

Note non-determinisme (cf S2/S3) : l'unite reproductible est le TAUX AGREGE, pas le
label par seed. Un meme code peut finir 3.05 m sur un run et tomber a 2.2 m sur un
autre (bifurcation d'ensemble actif du QP tranchee par l'entropie process).

Usage (terminal Anaconda, depuis pal_talos/) :
    python s4_batch.py                         # 20 seeds (0..19), payload 2 kg
    python s4_batch.py --n 30 --start 0
    python s4_batch.py --outdir s4_batch_p0 -- --payload 0    # A/B a vide : args apres --
    python s4_batch.py --outdir s4_batch_wee100 -- --wee 100  # ablation arbitrage
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np

FROZEN = ["--payload", "2.0"]       # config de campagne (dist 3.0 = defaut du script)
SCRIPT = "talos_s4_qs_carry.py"
TIMEOUT = 1200         # s par essai (QS ~65 s sim ; marge large pour le wall-clock QP)
Z = 1.959963984540054  # 95 %


def wilson(k, n, z=Z):
    """Intervalle de Wilson 95 % pour k succes sur n essais -> (low, high)."""
    if n == 0:
        return 0.0, 1.0
    p = k / n
    denom = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / denom
    half = z * np.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / denom
    return max(0.0, centre - half), min(1.0, centre + half)


def _rmse(a):
    a = np.asarray(a, float); a = a[~np.isnan(a)]
    return float(np.sqrt(np.mean(a ** 2))) if a.size else float("nan")


def run_seed(seed, outdir, extra):
    npz = os.path.join(outdir, "s4_seed%02d.npz" % seed)
    log = os.path.join(outdir, "s4_seed%02d.log" % seed)
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
    ap.add_argument("--n", type=int, default=20, help="nombre d'essais (Table 3.1 : N >= 20)")
    ap.add_argument("--start", type=int, default=0, help="premier seed")
    ap.add_argument("--outdir", type=str, default="s4_batch")
    ap.add_argument("--thresh", type=float, default=0.90,
                    help="critere : borne basse Wilson > thresh (Table 3.1)")
    ap.add_argument("extra", nargs=argparse.REMAINDER,
                    help="args passes au script S4 apres -- (ex : -- --payload 0)")
    a = ap.parse_args()
    extra = [e for e in a.extra if e != "--"]
    # garde-fou cmd.exe : commentaires '# ...' colles a la commande -> retires
    for i, e in enumerate(extra):
        if e.startswith("#"):
            extra = extra[:i]; break
    junk = [e for i, e in enumerate(extra)
            if not e.startswith("-") and (i == 0 or not extra[i - 1].startswith("-"))]
    if junk:
        raise SystemExit("[ABORT] arguments extra suspects (pas des options) : %s" % junk)
    os.makedirs(a.outdir, exist_ok=True)
    seeds = list(range(a.start, a.start + a.n))
    print("S4 BATCH  N=%d seeds=%d..%d  config %s  extra=%s  -> %s/"
          % (a.n, seeds[0], seeds[-1], " ".join(FROZEN), extra or "-", a.outdir))

    rows = []
    for s in seeds:
        npz_path, log_path, status, wall = run_seed(s, a.outdir, extra)
        row = dict(seed=s, status=status, wall_s=wall, success_func=False, success_strict=False,
                   done=False, upright=False, box_held=False, dist=np.nan,
                   com_rmse_mm=np.nan, ee_rmse_mm=np.nan, ee_pic_mm=np.nan,
                   grf_pic=np.nan, ctrl_mean=np.nan, ctrl_p99=np.nan)
        if status == "ok" and os.path.exists(npz_path):
            try:
                d = np.load(npz_path)
                cm = d["ctrl_ms"]; cerr = d["com_err"]
                eeL, eeR, reg = d["ee_L"], d["ee_R"], d["ee_regime"]
                walk = reg == 1
                ee_walk = np.concatenate([eeL[walk], eeR[walk]]) if walk.any() else np.concatenate([eeL, eeR])
                row.update(success_func=bool(d["success_func"]), success_strict=bool(d["success"]),
                           done=bool(d["done"]), upright=bool(d["upright"]),
                           box_held=bool(d["box_held"]), dist=float(d["dist"]),
                           com_rmse_mm=_rmse(cerr) * 1e3, ee_rmse_mm=_rmse(ee_walk) * 1e3,
                           ee_pic_mm=float(np.nanmax(ee_walk)) * 1e3 if ee_walk.size else np.nan,
                           grf_pic=float(np.nanmax(d["grf"])),
                           ctrl_mean=float(cm.mean()), ctrl_p99=float(np.percentile(cm, 99)))
            except Exception as e:
                row["status"] = "npz_error:%s" % e
        elif status == "ok":
            row["status"] = "crash"
        rows.append(row)
        print("  seed %2d : %-7s func=%s strict=%s dist=%5.2f m  CoM=%5.1f  EE=%5.1f mm (%.0f s)"
              % (s, row["status"], "OK " if row["success_func"] else "no ",
                 "OK " if row["success_strict"] else "no ",
                 row["dist"], row["com_rmse_mm"], row["ee_rmse_mm"], wall))
        if row["status"] == "crash":
            try:
                tail = [l.rstrip() for l in open(log_path, encoding="utf-8", errors="replace")][-3:]
            except OSError:
                tail = ["(log illisible)"]
            for l in tail:
                print("      | %s" % l)
            if s == seeds[0]:
                raise SystemExit("[ABORT] le 1er essai a crashe — corriger avant de relancer (log : %s)" % log_path)

    # ---------- agregation ----------
    n = len(rows)
    kf = sum(r["success_func"] for r in rows); ks = sum(r["success_strict"] for r in rows)
    lof, hif = wilson(kf, n); los, his = wilson(ks, n)
    okf = [r for r in rows if r["success_func"]]

    def mstd(key, sel):
        v = np.array([r[key] for r in sel], dtype=float); v = v[~np.isnan(v)]
        return (v.mean(), v.std()) if v.size else (np.nan, np.nan)

    verdict = "PASS" if lof > a.thresh else "FAIL"
    lines = []
    lines.append("# S4 batch — %d essais (seeds %d..%d)%s" %
                 (n, seeds[0], seeds[-1], "  extra: " + " ".join(extra) if extra else ""))
    lines.append("")
    lines.append("| Metrique | Valeur |")
    lines.append("|---|---|")
    lines.append("| Success FONCTIONNEL (debout+3m+DONE+boite) | **%d/%d = %.1f %%** |" % (kf, n, 100 * kf / n))
    lines.append("| Wilson 95 %% CI (fonctionnel) | [%.3f, %.3f] |" % (lof, hif))
    lines.append("| Critere Table 3.1 (borne basse > %.2f) | **%s** |" % (a.thresh, verdict))
    lines.append("| Success STRICT (+ EE pic < 5 cm) | %d/%d = %.1f %% | Wilson [%.3f, %.3f] |"
                 % (ks, n, 100 * ks / n, los, his))
    lines.append("| Taux boite portee | %d/%d |" % (sum(r["box_held"] for r in rows), n))
    for key, label, unit, sel in (
            ("dist", "Distance", "m", okf),
            ("ee_rmse_mm", "EE error RMSE (marche)", "mm", okf),
            ("ee_pic_mm", "EE error pic (marche)", "mm", okf),
            ("com_rmse_mm", "CoM tracking RMSE", "mm", okf),
            ("grf_pic", "GRF pic", "N", rows),
            ("ctrl_mean", "Ctrl loop mean", "ms", rows),
            ("ctrl_p99", "Ctrl loop p99", "ms", rows)):
        m, sd = mstd(key, sel)
        lines.append("| %s (%s, %s) | %.2f +/- %.2f |"
                     % (label, "succes fonc." if sel is okf else "tous", unit, m, sd))
    fails = [r for r in rows if not r["success_func"]]
    if fails:
        lines.append("")
        lines.append("Echecs fonctionnels : " + ", ".join(
            "seed %d (%s, dist=%.2f)" % (r["seed"], r["status"], r["dist"]) for r in fails))
    report = "\n".join(lines)
    print("\n" + report)

    with open(os.path.join(a.outdir, "s4_batch_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    with open(os.path.join(a.outdir, "s4_batch_rows.csv"), "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader(); w.writerows(rows)
    np.savez(os.path.join(a.outdir, "s4_batch_summary.npz"),
             seeds=np.array(seeds),
             success_func=np.array([r["success_func"] for r in rows]),
             success_strict=np.array([r["success_strict"] for r in rows]),
             kf=kf, ks=ks, n=n, wilson_low=lof, wilson_high=hif,
             thresh=a.thresh, verdict=verdict, extra=" ".join(extra))
    print("\n[ok] %s/s4_batch_report.md  +  s4_batch_rows.csv  +  s4_batch_summary.npz" % a.outdir)


if __name__ == "__main__":
    main()
