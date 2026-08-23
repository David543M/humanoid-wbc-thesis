"""
S2 — Test de determinisme (P1d, caveat reproductibilite Ch5 §5.5 / Ch6).

Question : le non-determinisme par seed observe entre les batchs N=20 et N=50
(meme config gelee, meme seed -> issues differentes ; ex. seed 3 SUCCESS 3.59 m
puis FELL 1.34 m) vient-il de la non-associativite flottante du BLAS multi-thread,
ou est-il intrinseque (chaos de marge mince, autre source d'entropie) ?

Protocole : pour chaque seed, relancer la marche R fois (defaut 2) sous DEUX
regimes de threading, en SOUS-PROCESSUS (l'env doit etre pose AVANT l'import
numpy/mujoco de l'enfant) :
  - MULTI  : threading par defaut (ce qu'utilisaient les batchs)
  - SINGLE : BLAS mono-thread force (OMP/MKL/OPENBLAS/NUMEXPR/VECLIB = 1,
             PYTHONHASHSEED=0)
Puis comparer les trajectoires com_err (et xi_err) tick-a-tick entre repetitions
d'un meme (seed, regime).

Lecture des resultats :
  - MULTI  identiques  ET SINGLE identiques -> deja deterministe (surprise ;
                                               le non-det venait d'ailleurs)
  - MULTI  divergent   ET SINGLE identiques -> HYPOTHESE CONFIRMEE : le BLAS
                                               multi-thread est la cause ;
                                               mono-thread restaure le determinisme
  - MULTI  divergent   ET SINGLE divergent  -> non-determinisme INTRINSEQUE
                                               (chaos marge mince / autre entropie)
                                               -> argument Ch6 encore plus fort

La divergence est caracterisee par : identiques bit-a-bit ? premier tick de
divergence, ecart max, et issue macro (dist / fell_at / success). Un premier
tick tres precoce ~ epsilon machine qui croit = amplification chaotique ; une
divergence des le tick 0 = entropie d'initialisation.

Usage (terminal Anaconda, depuis pal_talos/) :
    python s2_determinism.py                       # seeds 0 2 3, R=2, ~10 min
    python s2_determinism.py --seeds 0 1 2 3 --repeats 3
Sorties : determinism_out/  (npz par run + determinism_report.md)
"""
import argparse, os, subprocess, sys, time
import numpy as np

FROZEN = ["--steps", "70", "--offlat", "0.08", "--dsovl", "0.12", "--tmin", "0.24"]
SCRIPT = "talos_dcm_walk_timing.py"
TIMEOUT = 900

# Variables lues a l'import par numpy / MKL / OpenBLAS / OpenMP : posees dans
# l'env de l'ENFANT (les poser dans ce process-ci serait trop tard).
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
    """Compare deux trajectoires com_err : (identique?, 1er tick divergent, ecart max)."""
    if a is None or b is None:
        return dict(identical=False, first=-1, maxabs=float("nan"), note="run manquant")
    na, nb = len(a["com"]), len(b["com"])
    m = min(na, nb)
    ca, cb = a["com"][:m], b["com"][:m]
    if na == nb and np.array_equal(a["com"], b["com"]):
        return dict(identical=True, first=-1, maxabs=0.0, note="bit-identique")
    diff = np.abs(ca - cb)
    nz = np.nonzero(diff > 0)[0]
    first = int(nz[0]) if nz.size else -1
    note = "longueurs differentes (%d vs %d)" % (na, nb) if na != nb else "valeurs differentes"
    return dict(identical=False, first=first, maxabs=float(diff.max()), note=note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 2, 3])
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--outdir", type=str, default="determinism_out")
    ap.add_argument("--regimes", type=str, nargs="+", default=["multi", "single"])
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    print("DETERMINISME  seeds=%s  repeats=%d  regimes=%s  config=%s"
          % (a.seeds, a.repeats, a.regimes, " ".join(FROZEN)))

    data = {}       # (seed, regime, rep) -> outcome
    for regime in a.regimes:
        for seed in a.seeds:
            for rep in range(a.repeats):
                npz, wall = run(seed, regime, rep, a.outdir)
                o = outcome(npz)
                data[(seed, regime, rep)] = o
                tag = ("dist=%.3f fell=%.1f %s" % (o["dist"], o["fell"],
                       "SUCCESS" if o["succ"] else "fail")) if o else "CRASH (pas de npz)"
                print("  seed %2d %-6s r%d : %-32s (%.0f s)" % (seed, regime, rep, tag, wall))

    # ---------- comparaison rep0 vs rep1.. ----------
    lines = ["# S2 — Test de determinisme", "",
             "Config gelee : `%s`" % " ".join(FROZEN),
             "Repetitions comparees a rep0. `identique` = com_err bit-a-bit egal.", ""]
    lines += ["| seed | regime | rep | identique | 1er tick div. | ecart max (m) | issue | note |",
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
                    seed, regime, rep, "OUI" if c["identical"] else "**NON**",
                    c["first"] if c["first"] >= 0 else "—", c["maxabs"], iss, c["note"]))

    # ---------- verdict ----------
    lines += ["", "## Verdict"]
    multi_det = regime_identical.get("multi", None)
    single_det = regime_identical.get("single", None)
    if multi_det and single_det:
        v = "Les deux regimes sont deterministes -> le non-determinisme observe entre batchs venait d'AILLEURS (config differente entre runs ? a investiguer)."
    elif (multi_det is False) and single_det:
        v = "**HYPOTHESE CONFIRMEE** : multi-thread divergent, mono-thread identique -> le BLAS multi-thread (non-associativite flottante) est la cause. Le determinisme par seed est recuperable en forcant OMP/MKL=1. -> Recommander l'execution mono-thread pour toute campagne reportee (Ch5) + documenter (Ch6)."
    elif (multi_det is False) and (single_det is False):
        v = "**NON-DETERMINISME INTRINSEQUE** : divergence persistante meme en mono-thread -> l'entropie n'est pas (seulement) le BLAS. Cause probable : sensibilite chaotique du cycle limite a marge mince. -> La quantite reproductible reste le TAUX AGREGE ; argument Ch6 renforce (la config opere au bord de la stabilite)."
    else:
        v = "Resultat mixte / incomplet — voir le tableau."
    lines += [v, "",
              "Rappel : issue macro divergente deja observee entre N=20 et N=50 "
              "(seed 3 SUCCESS 3.59 m -> FELL 1.34 m ; seed 2 3.72 -> 3.16 m ; seed 0 3.37 -> 3.42 m)."]

    report = "\n".join(lines)
    print("\n" + report)
    with open(os.path.join(a.outdir, "determinism_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    print("\n[ok] %s/determinism_report.md" % a.outdir)


if __name__ == "__main__":
    main()
