"""
S3 — Batch N seeds sur le sequenceur quasi-statique (config gelee QS #5c + polish 6a).

MOTIVATION (2026-08-06, avant viva) : Ch5 par.5.6 rapporte la montee 3/3 a partir d'UN
SEUL run. La contribution declaree de la these (PQ4) etant precisement le protocole
batch + intervalle de Wilson, un scenario rapporte sur un run unique est la seule
incoherence interne exploitable du manuscrit. Ce batch la ferme — ou la transforme en
limite chiffree, ce qui est defendable de la meme facon.

Lance talos_s3_stairs_qs.py en SOUS-PROCESSUS par seed. L'isolation par processus n'est
PAS un detail de confort : les scripts mutent des globals partages (talos_wbc.MODEL,
gains) et tout enchainement dans un meme interpreteur est invalide (finding transverse
2026-07-30, contamination d'etat inter-scenarios). Meme patron que s2_batch.py /
s4_batch.py, qui ne sont donc pas affectes.

VERDICT PRE-ENREGISTRE (identique au critere du script, aucune latitude apres coup) :
    success = UPRIGHT  ET  3/3 marches atteintes  ET  etat final DONE
Le bruit initial est le meme que S2 (qvel sigma=0.01 via default_rng(seed)), donc le
taux agrege est directement comparable aux 70 % de S2.

Sorties dans OUTDIR : s3_seed<k>.npz + s3_seed<k>.log, s3_batch_summary.npz,
s3_batch_report.md, s3_batch_rows.csv.

Usage (terminal Anaconda, depuis pal_talos/) :
    python s3_batch.py                       # 20 seeds (0..19), config gelee S3-QS
    python s3_batch.py --n 30 --start 0      # confirmatoire (N >= 35 pour un verdict PASS)
    python s3_batch.py --outdir s3_batch_h15 -- --hriser 0.15   # variante : args apres --
"""
import argparse, csv, os, subprocess, sys, time
import numpy as np

# Config gelee = les defauts du succes 3/3 (QS #5c + w_foot_stance=1400), passes
# EXPLICITEMENT pour que le report soit auto-documente et rejouable sans lire le script.
FROZEN = ["--rate", "0.18", "--tswing", "0.65", "--ttrmax", "3.0",
          "--postol", "0.035", "--clear", "0.14", "--hriser", "0.10"]
SCRIPT = "talos_s3_stairs_qs.py"
TIMEOUT = 1800         # s par essai (la montee QS est lente : ~25 s de sim, marge large)
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
    ap.add_argument("--n", type=int, default=20, help="nombre d'essais (Table 3.1 : N >= 20 pilote)")
    ap.add_argument("--start", type=int, default=0, help="premier seed")
    ap.add_argument("--outdir", type=str, default="s3_batch")
    ap.add_argument("--thresh", type=float, default=0.90,
                    help="critere : borne basse Wilson > thresh (Table 3.1)")
    ap.add_argument("extra", nargs=argparse.REMAINDER,
                    help="args passes au sequenceur apres -- (ex : -- --hriser 0.15)")
    a = ap.parse_args()
    extra = [e for e in a.extra if e != "--"]
    # garde-fou cmd.exe (repris de s2_batch.py) : un commentaire '# ...' colle a la
    # commande n'est PAS ignore par cmd -> il atterrit dans extra et fait crasher le
    # script enfant (batch 2026-07-08 : 0/20 artificiel, essais de 0-1 s).
    for i, e in enumerate(extra):
        if e.startswith("#"):
            extra = extra[:i]; break
    junk = [e for i, e in enumerate(extra)
            if not e.startswith("-") and (i == 0 or not extra[i - 1].startswith("-"))]
    if junk:
        raise SystemExit("[ABORT] arguments extra suspects (pas des options) : %s" % junk)
    os.makedirs(a.outdir, exist_ok=True)
    seeds = list(range(a.start, a.start + a.n))
    print("S3 BATCH  N=%d seeds=%d..%d  config gelee %s  extra=%s  -> %s/"
          % (a.n, seeds[0], seeds[-1], " ".join(FROZEN), extra or "-", a.outdir))
    print("  verdict pre-enregistre : UPRIGHT et 3/3 marches et etat final DONE")
    # Regle Ch5 par.5.2 : le verdict est la BORNE BASSE de Wilson, pas le taux ponctuel.
    # Consequence arithmetique : a N=20, meme 20/20 ne donne que LB=0.839 -> un batch
    # pilote peut ECHOUER mais jamais REUSSIR. N=35 est le premier N ou 35/35 franchit
    # 0.90 (LB=0.901). Lancer N<35 en esperant un PASS est une perte de temps.
    if a.n < 35 and a.thresh >= 0.90:
        print("  [!] N=%d : un verdict PASS est ARITHMETIQUEMENT IMPOSSIBLE (35/35 -> LB=0.901"
              " est le premier cas favorable). Ce batch est un PILOTE : il peut falsifier,"
              " pas valider. Utiliser --n 35 (ou plus) pour un batch confirmatoire." % a.n)

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
                # purge des artefacts de timer (cf. methodo 2026-07-15 : des valeurs
                # aberrantes jusqu'a 1e7 ms polluent les stats si on ne filtre pas)
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
        elif status == "ok":                     # process sorti sans npz = CRASH
            row["status"] = "crash"
        rows.append(row)
        print("  seed %2d : %-7s %s  %d/3 marches  mi=%2d/%2d  dist=%5.2f m  (%.0f s)"
              % (s, row["status"], "SUCCESS" if row["success"] else "fail   ",
                 max(row["n_climbed"], 0), row["mi"], row["n_moves"], row["dist"], wall))
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
    # distribution du nombre de marches : distingue "echoue tot" de "echoue au sommet",
    # qui n'ont pas la meme lecture (le mode d'echec du run 3/3 etait le settle final)
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
    lines.append("> ATTENTION metrique clearance : le logger `_clear_tick` du sequenceur")
    lines.append("> remonte 0 franchissement sur un run complet (item P4 du backlog polish,")
    lines.append("> non corrige — toucher le swing avait regresse la montee 3 fois). La garde")
    lines.append("> de swing n'est donc PAS chiffrable depuis ce batch ; ne pas la rapporter.")
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
