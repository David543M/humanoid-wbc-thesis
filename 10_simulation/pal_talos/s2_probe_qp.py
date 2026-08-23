"""
S2 — Probe root-cause du non-determinisme par seed (P1e).

Chaine d'indices (de s2_determinism.py) :
  - condition initiale reproductible (`--seed` -> default_rng -> bruit qvel, tirage unique)
    => ticks 0-47 bit-identiques entre runs du meme seed ;
  - divergence injectee ~tick 50, i.e. a l'ENGAGEMENT du QP-WBC ;
  - le mono-thread (OMP/MKL=1) ne corrige PAS -> pas (que) le BLAS multi-thread.

SUSPECT PRINCIPAL : un buffer `np.empty` NON-INITIALISE dans l'assemblage du QP
(H/g/A pre-alloues une fois, dimension effective variable avec le nombre de
contacts 4-8 -> les entrees inutilisees contiennent du garbage tas qui varie
d'un process a l'autre et nourrit ProxQP -> solutions differentes).

TEST (decisif) — 2 modes, meme seed, en SOUS-PROCESSUS :
  1. PLAIN     : walker tel quel, 2 runs        -> attendu : DIVERGENT (reproduit le bug)
  2. ZEROEMPTY : walker avec np.empty/np.empty_like forces a zero-fill (niveau
                 Python), 2 runs                -> si IDENTIQUE : cause = memoire
                 non-initialisee cote Python ; FIX = zero-init des buffers dans
                 talos_wbc.py (remplacer np.empty par np.zeros aux pre-allocations).

Signature de comparaison = trajectoire com_err (bit-a-bit).

Lecture du verdict :
  - PLAIN divergent + ZEROEMPTY identique -> ROOT CAUSE CONFIRMEE (np.empty).
  - PLAIN divergent + ZEROEMPTY divergent -> l'entropie est cote C-extension
    (proxsuite / mujoco / pinocchio), pas un np.empty Python -> etape suivante :
    capturer les matrices reelles du QP au tick ~50 et tester ProxQP dessus,
    ou activer les flags de determinisme du solveur.
  - PLAIN identique -> pas reproduit ce coup-ci (rejouer / autre seed / +reps).

Usage (terminal Anaconda, depuis pal_talos/) :
    python s2_probe_qp.py                 # seed 3, ~6-8 min (4 runs)
    python s2_probe_qp.py --seed 2
Sorties : probe_qp_out/  (npz par run + probe_qp_report.md)
"""
import argparse, os, subprocess, sys, time
import numpy as np

FROZEN = ["--steps", "70", "--offlat", "0.08", "--dsovl", "0.12", "--tmin", "0.24"]
SCRIPT = "talos_dcm_walk_timing.py"
TIMEOUT = 900

# Bootstrap injecte dans l'enfant : patche np.empty/np.empty_like AVANT que le
# walker n'alloue quoi que ce soit, puis execute le script comme __main__.
BOOT = (
    "import numpy as _np\n"
    "_np.empty = (lambda *a, **k: _np.zeros(*a, **k))\n"
    "_np.empty_like = (lambda *a, **k: _np.zeros_like(*a, **k))\n"
    "import sys, runpy\n"
    "sys.argv = {argv!r}\n"
    "runpy.run_path({script!r}, run_name='__main__')\n"
)


def run(seed, mode, rep, outdir):
    npz = os.path.join(outdir, "probe_%s_s%02d_r%d.npz" % (mode, seed, rep))
    log = os.path.join(outdir, "probe_%s_s%02d_r%d.log" % (mode, seed, rep))
    argv = [SCRIPT] + FROZEN + ["--seed", str(seed), "--save", npz]
    if mode == "plain":
        cmd = [sys.executable, SCRIPT] + FROZEN + ["--seed", str(seed), "--save", npz]
    else:  # zeroempty
        cmd = [sys.executable, "-c", BOOT.format(argv=argv, script=SCRIPT)]
    t0 = time.time()
    with open(log, "w", encoding="utf-8") as f:
        subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT, timeout=TIMEOUT, check=False)
    return npz, time.time() - t0


def load(npz):
    if not os.path.exists(npz):
        return None
    d = np.load(npz, allow_pickle=True)
    return dict(com=d["com_err"], dist=float(d["dist"]), fell=float(d["fell_at"]),
                succ=bool(d["success"]))


def compare(a, b):
    if a is None or b is None:
        return dict(identical=False, first=-1, maxabs=float("nan"), note="run manquant")
    na, nb = len(a["com"]), len(b["com"])
    if na == nb and np.array_equal(a["com"], b["com"]):
        return dict(identical=True, first=-1, maxabs=0.0, note="bit-identique")
    m = min(na, nb)
    diff = np.abs(a["com"][:m] - b["com"][:m])
    nz = np.nonzero(diff > 0)[0]
    first = int(nz[0]) if nz.size else -1
    note = "longueurs %d vs %d" % (na, nb) if na != nb else "valeurs differentes"
    return dict(identical=False, first=first, maxabs=float(diff.max()), note=note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3, help="seed divergent connu (3, 2 ou 0)")
    ap.add_argument("--reps", type=int, default=6,
                    help="repetitions/mode ; la divergence est RARE -> >=6 pour trancher")
    ap.add_argument("--outdir", type=str, default="probe_qp_out")
    ap.add_argument("--modes", nargs="+", default=["plain", "zeroempty"])
    a = ap.parse_args()
    os.makedirs(a.outdir, exist_ok=True)
    print("PROBE QP  seed=%d  reps=%d  modes=%s  config=%s"
          % (a.seed, a.reps, a.modes, " ".join(FROZEN)))

    data = {}
    for mode in a.modes:
        for rep in range(a.reps):
            npz, wall = run(a.seed, mode, rep, a.outdir)
            o = load(npz)
            data[(mode, rep)] = o
            tag = ("dist=%.3f fell=%.1f %s" % (o["dist"], o["fell"],
                   "SUCCESS" if o["succ"] else "fail")) if o else "CRASH (pas de npz)"
            print("  %-9s r%d : %-30s (%.0f s)" % (mode, rep, tag, wall))

    # La divergence est RARE -> on compte les trajectoires UNIQUES par mode
    # (empreinte bit-a-bit de com_err). Discriminateur statistique de l'hypothese
    # np.empty : si zeroempty s'effondre a 1 trajectoire alors que plain en montre
    # plusieurs, la memoire non-init est la cause. Si les deux montrent de la
    # dispersion, l'entropie est ailleurs (C-extension / niveau process).
    import hashlib

    def sig(o):
        return hashlib.md5(o["com"].tobytes()).hexdigest()[:8] if o else "MISSING"

    lines = ["# S2 — Probe root-cause QP (P1e)", "",
             "Seed %d, config `%s`. %d repetitions/mode. La divergence etant RARE, "
             "on compte les trajectoires com_err UNIQUES." % (a.seed, " ".join(FROZEN), a.reps), "",
             "| mode | #runs | #traj. uniques | distances distinctes | #chutes |",
             "|---|---|---|---|---|"]
    uniq = {}
    for mode in a.modes:
        obs = [data.get((mode, r)) for r in range(a.reps)]
        u = len({sig(o) for o in obs})
        uniq[mode] = u
        dists = sorted({round(o["dist"], 3) for o in obs if o})
        nfall = sum(1 for o in obs if o and not o["succ"])
        lines.append("| %s | %d | **%d** | %s | %d |" % (
            mode, a.reps, u, ", ".join("%.3f" % x for x in dists) or "—", nfall))

    lines += ["", "## Verdict"]
    p, z = uniq.get("plain"), uniq.get("zeroempty")
    if p is None or z is None:
        v = "Resultat incomplet - voir le tableau."
    elif p > 1 and z == 1:
        v = ("ROOT CAUSE = memoire non-initialisee (np.empty). plain diverge, zeroempty "
             "s'effondre a 1 trajectoire. FIX : remplacer np.empty par np.zeros aux "
             "pre-allocations H/g/A dans talos_wbc.py -> determinisme par seed recuperable "
             "(protocole vraiment reproductible, coeur PQ4). Re-lancer s2_determinism.py.")
    elif p > 1 and z > 1:
        v = ("np.empty EXCLU : les deux modes dispersent (plain %d, zeroempty %d traj. uniques). "
             "L'entropie est cote C-extension / niveau process (proxsuite / mujoco / ASLR), "
             "pas un buffer Python. Evenement RARE (bascule d'ensemble actif du QP ou detection "
             "de contact marginale). Options : (i) dumper les matrices du QP au 1er solve + "
             "probe ProxQP isole ; (ii) l'accepter comme limite documentee, unite reproductible "
             "= taux agrege." % (p, z))
    elif p == 1 and z == 1:
        v = ("Aucune divergence sur %d reps/mode : pour ce seed la bascule n'a pas fire "
             "(evenement rare). Rejouer, --reps 12, ou --seed 2/0." % a.reps)
    else:
        v = ("plain stable %d, zeroempty disperse %d : le patch a DEPLACE le declencheur "
             "(adresses differentes) sans le creer -> entropie niveau process, pas un garbage "
             "lu a chaque tick. Traiter comme limite documentee." % (p, z))
    lines.append(v)
    lines.append("")
    lines.append("Rappel : quel que soit le mode, l'issue macro d'un seed peut basculer entre "
                 "invocations (seed 3 : 3.587 m SUCCESS vs 1.316 m chute) -> l'unite reproductible "
                 "reste le TAUX AGREGE (Ch5 §5.5).")

    report = "\n".join(lines)
    print("\n" + report)
    with open(os.path.join(a.outdir, "probe_qp_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    print("\n[ok] %s/probe_qp_report.md" % a.outdir)


if __name__ == "__main__":
    main()
