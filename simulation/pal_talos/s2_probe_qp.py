"""
S2 — Root-cause probe for the per-seed non-determinism (P1e).

Chain of evidence (from s2_determinism.py):
  - reproducible initial condition (`--seed` -> default_rng -> qvel noise, single draw)
    => ticks 0-47 bit-identical between runs of the same seed;
  - divergence injected ~tick 50, i.e. at the ENGAGEMENT of the QP-WBC;
  - single-threading (OMP/MKL=1) does NOT fix it -> not (only) the multi-threaded BLAS.

PRIME SUSPECT: an UNINITIALISED `np.empty` buffer in the QP assembly
(H/g/A pre-allocated once, effective dimension varying with the number of
contacts 4-8 -> the unused entries hold heap garbage that varies
from one process to the next and feeds ProxQP -> different solutions).

TEST (decisive) — 2 modes, same seed, in SUBPROCESSES:
  1. PLAIN     : walker as-is, 2 runs          -> expected: DIVERGENT (reproduces the bug)
  2. ZEROEMPTY : walker with np.empty/np.empty_like forced to zero-fill (Python
                 level), 2 runs                -> if IDENTICAL: cause = uninitialised
                 memory on the Python side; FIX = zero-init the buffers in
                 talos_wbc.py (replace np.empty with np.zeros at the pre-allocations).

Comparison signature = com_err trajectory (bit-for-bit).

Reading the verdict:
  - PLAIN divergent + ZEROEMPTY identical -> ROOT CAUSE CONFIRMED (np.empty).
  - PLAIN divergent + ZEROEMPTY divergent -> the entropy is on the C-extension side
    (proxsuite / mujoco / pinocchio), not a Python np.empty -> next step:
    capture the actual QP matrices at tick ~50 and test ProxQP on them,
    or enable the solver's determinism flags.
  - PLAIN identical -> not reproduced this time (replay / another seed / +reps).

Usage (Anaconda terminal, from pal_talos/):
    python s2_probe_qp.py                 # seed 3, ~6-8 min (4 runs)
    python s2_probe_qp.py --seed 2
Outputs: probe_qp_out/  (npz per run + probe_qp_report.md)
"""
import argparse, os, subprocess, sys, time
import numpy as np

FROZEN = ["--steps", "70", "--offlat", "0.08", "--dsovl", "0.12", "--tmin", "0.24"]
SCRIPT = "talos_dcm_walk_timing.py"
TIMEOUT = 900

# Bootstrap injected into the child: patches np.empty/np.empty_like BEFORE the
# walker allocates anything, then runs the script as __main__.
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
        return dict(identical=False, first=-1, maxabs=float("nan"), note="missing run")
    na, nb = len(a["com"]), len(b["com"])
    if na == nb and np.array_equal(a["com"], b["com"]):
        return dict(identical=True, first=-1, maxabs=0.0, note="bit-identical")
    m = min(na, nb)
    diff = np.abs(a["com"][:m] - b["com"][:m])
    nz = np.nonzero(diff > 0)[0]
    first = int(nz[0]) if nz.size else -1
    note = "lengths %d vs %d" % (na, nb) if na != nb else "different values"
    return dict(identical=False, first=first, maxabs=float(diff.max()), note=note)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=3, help="known divergent seed (3, 2 or 0)")
    ap.add_argument("--reps", type=int, default=6,
                    help="repetitions/mode; the divergence is RARE -> >=6 to decide")
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
                   "SUCCESS" if o["succ"] else "fail")) if o else "CRASH (no npz)"
            print("  %-9s r%d : %-30s (%.0f s)" % (mode, rep, tag, wall))

    # The divergence is RARE -> count the UNIQUE trajectories per mode
    # (bit-for-bit fingerprint of com_err). Statistical discriminator for the
    # np.empty hypothesis: if zeroempty collapses to 1 trajectory while plain shows
    # several, uninitialised memory is the cause. If both show
    # spread, the entropy is elsewhere (C-extension / process level).
    import hashlib

    def sig(o):
        return hashlib.md5(o["com"].tobytes()).hexdigest()[:8] if o else "MISSING"

    lines = ["# S2 — QP root-cause probe (P1e)", "",
             "Seed %d, config `%s`. %d repetitions/mode. Since the divergence is RARE, "
             "we count the UNIQUE com_err trajectories." % (a.seed, " ".join(FROZEN), a.reps), "",
             "| mode | #runs | #unique traj. | distinct distances | #falls |",
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
        v = "Incomplete result - see the table."
    elif p > 1 and z == 1:
        v = ("ROOT CAUSE = uninitialised memory (np.empty). plain diverges, zeroempty "
             "collapses to 1 trajectory. FIX: replace np.empty with np.zeros at the "
             "H/g/A pre-allocations in talos_wbc.py -> per-seed determinism recoverable "
             "(truly reproducible protocol). Re-run s2_determinism.py.")
    elif p > 1 and z > 1:
        v = ("np.empty RULED OUT: both modes spread (plain %d, zeroempty %d unique traj.). "
             "The entropy is on the C-extension / process side (proxsuite / mujoco / ASLR), "
             "not a Python buffer. RARE event (QP active-set switch or marginal "
             "contact detection). Options: (i) dump the QP matrices at the first solve + "
             "isolated ProxQP probe; (ii) accept it as a documented limitation, reproducible unit "
             "= aggregate rate." % (p, z))
    elif p == 1 and z == 1:
        v = ("No divergence over %d reps/mode: for this seed the switch did not fire "
             "(rare event). Replay, --reps 12, or --seed 2/0." % a.reps)
    else:
        v = ("plain stable %d, zeroempty spread %d: the patch MOVED the trigger "
             "(different addresses) without creating it -> process-level entropy, not garbage "
             "read at every tick. Treat as a documented limitation." % (p, z))
    lines.append(v)
    lines.append("")
    lines.append("Reminder: whatever the mode, the macro outcome of a seed can flip between "
                 "invocations (seed 3: 3.587 m SUCCESS vs 1.316 m fall) -> the reproducible unit "
                 "remains the AGGREGATE RATE (Ch5 §5.5).")

    report = "\n".join(lines)
    print("\n" + report)
    with open(os.path.join(a.outdir, "probe_qp_report.md"), "w", encoding="utf-8") as f:
        f.write(report + "\n")
    print("\n[ok] %s/probe_qp_report.md" % a.outdir)


if __name__ == "__main__":
    main()
