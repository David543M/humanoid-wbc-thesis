"""
validate_s1_batch.py — STATISTICAL validation of scenario S1 (static balancing).

Protocol (methodology chapter): stay standing under external pushes; success > 90 % over
N >= 20 trials, 95 % Wilson confidence interval. Each trial applies a
domain randomisation (mass, friction, initial state) + a random push
(magnitude, direction). Success = stays standing, recovers, 0 QP fallback.

RESUMABLE: each trial is persisted to results.csv (tolerant to timeouts).
  python validate_s1_batch.py --n 25 --budget 38 --out runs/s1     # re-invoke until [ALL DONE]
  python validate_s1_batch.py --report --out runs/s1               # force the report

Additions 2026-07-16 (backup: validate_s1_batch_BACKUP_20260716.py):
  - per-trial recovery time t_rec (S1 metric grounded in the methodology, never quantified):
    t_rec = time between the END of the impulse and the last instant at which the planar
    CoM deviation leaves the band REC_BAND = 20 mm (i.e. re-enters AND STAYS in the band
    until the end of the trial). 20 mm = ~1/3 of the measured lateral half-sole
    (60 mm), >> pre-push standing regulation (~1 mm), << success criterion (80 mm).
    NaN on a fall or if the band is not re-acquired by the end of the trial.
  - per-trial peak_dev (max post-push CoM excursion).
  - --exec {qp,ns,pd}: QP-WBC executor (default) or H3 baselines (s1_baselines.py:
    ns = Sentis-Khatib null-space task-priority; pd = gravity-compensated PD).
    SAME protocol/seeds -> comparison table Ch5 SS5.4.
  ⚠ extended CSV columns: use a NEW --out (e.g. runs/s1_corners_v3,
    runs/s1_ns, runs/s1_pd); runs/s1_corners_v2 stays the intact 50/50 record.
  - --viewer --seed N: replays trial N in the MuJoCo viewer (SAME draws as
    the batch: mass/friction/noise/push), verdict printed at t=DUR.
      python validate_s1_batch.py --viewer --seed 1000 --exec pd
"""
import argparse, os, csv, time, math, numpy as np, mujoco
import talos_wbc as W
import s1_baselines as BL

# push distribution (Ch3 flag: not grounded, chosen) + domain randomisation
F_MIN, F_MAX = 100.0, 300.0     # push magnitude (N), drawn U(F_MIN, F_MAX)
T_PUSH, IMPULSE = 2.0, 0.1      # impulse onset + duration (s)
DUR = 4.5                       # trial duration (s)
MASS_PCT, FRIC_PCT, INIT_NOISE = 0.10, 0.20, 0.01
Z_FALL = 0.8                    # standing if z > 0.8
COM_REC = 0.08                  # recovered if final CoM deviation < 8 cm
REC_BAND = 0.02                 # recovery band (m) for t_rec (see docstring)


def _setup(seed, execkind="qp", pdscale=1.0, hip=False):
    """Initial state + executor + push for one trial — SHARED batch/viewer.
    The rng draw order is FROZEN (batch and viewer see the same trial)."""
    rng = np.random.default_rng(seed)
    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    base_mass = m.body_mass.copy(); base_fric = m.geom_friction.copy()
    # --- domain randomisation ---
    m.body_mass[:] = base_mass * (1.0 + rng.uniform(-MASS_PCT, MASS_PCT, base_mass.shape))
    f = base_fric.copy(); f[:, 0] *= (1.0 + rng.uniform(-FRIC_PCT, FRIC_PCT))
    m.geom_friction[:] = np.clip(f, 1e-3, None)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.qpos[7:] += rng.uniform(-INIT_NOISE, INIT_NOISE, d.qpos[7:].shape)
    d.qvel[:] += rng.uniform(-INIT_NOISE, INIT_NOISE, d.qvel.shape)
    mujoco.mj_forward(m, d)
    if execkind == "qp":
        wbc = W.WBC(m, d, corners=W.CORNERS_MEASURED, hip=hip)   # measured corners (S1)
    elif execkind == "pd":
        wbc = BL.PDGrav(m, d, scale=pdscale)            # H3 baseline (pd, gains x scale)
    else:
        wbc = BL.EXECUTORS[execkind](m, d)              # H3 baseline (ns)
    # --- random push (magnitude + horizontal direction) ---
    F = rng.uniform(F_MIN, F_MAX); th = rng.uniform(0, 2*np.pi)
    return m, d, wbc, F, th


def trial(seed, execkind="qp", pdscale=1.0, hip=False):
    m, d, wbc, F, th = _setup(seed, execkind, pdscale, hip)
    dt = m.opt.timestep
    fx, fy = F*np.cos(th), F*np.sin(th)
    fell = False
    t_end_push = T_PUSH + IMPULSE
    peak_dev = 0.0; last_out = None; dev = 0.0
    for i in range(int(DUR/dt)):
        wbc.control()
        d.xfrc_applied[wbc.base, :] = 0.0
        if T_PUSH <= d.time < t_end_push:
            d.xfrc_applied[wbc.base, 0] = fx; d.xfrc_applied[wbc.base, 1] = fy
        mujoco.mj_step(m, d)
        if d.qpos[2] < 0.6: fell = True; break
        if d.time >= T_PUSH:                            # post-push tracking (t_rec, peak)
            dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
            peak_dev = max(peak_dev, dev)
            if d.time >= t_end_push and dev >= REC_BAND:
                last_out = d.time
    com_dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
    # t_rec: re-enters AND stays in REC_BAND until the end; NaN on fall/not re-acquired
    if fell or com_dev >= REC_BAND:
        t_rec = float("nan")
    elif last_out is None:
        t_rec = 0.0
    else:
        t_rec = last_out - t_end_push
    fails = getattr(wbc, "_qp_fail", 0)
    success = (not fell) and d.qpos[2] > Z_FALL and com_dev < COM_REC and fails == 0
    return dict(seed=seed, F=F, theta=th, fell=fell, z=float(d.qpos[2]),
                com_dev=com_dev, peak_dev=peak_dev, t_rec=t_rec,
                qp_fails=fails, success=int(success))


def view_trial(seed, execkind="qp", pdscale=1.0, hip=False):
    """Replays trial `seed` in the MuJoCo viewer (same draws as the batch).
    Verdict printed at t=DUR; the sim then keeps running until the window is closed."""
    import time as _time
    from mujoco import viewer as mjv
    m, d, wbc, F, th = _setup(seed, execkind, pdscale, hip)
    dt = m.opt.timestep
    fx, fy = F*np.cos(th), F*np.sin(th)
    t_end_push = T_PUSH + IMPULSE
    ARROW_SHOW, ARROW_SCALE = 0.6, 0.0015
    print(f"[viewer] seed {seed}  exec={execkind}  F={F:.0f} N  dir={math.degrees(th):.0f} deg  "
          f"push at t={T_PUSH:.1f}s for {IMPULSE:.1f}s")
    m.vis.map.force = 0.002; m.vis.scale.forcewidth = 0.04
    peak_dev = 0.0; last_out = None; dev = 0.0; fell = False; done = False
    with mjv.launch_passive(m, d) as viewer:
        viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
        start = _time.time()
        while viewer.is_running():
            wbc.control()
            d.xfrc_applied[wbc.base, :] = 0.0
            if T_PUSH <= d.time < t_end_push:
                d.xfrc_applied[wbc.base, 0] = fx; d.xfrc_applied[wbc.base, 1] = fy
            mujoco.mj_step(m, d)
            if d.qpos[2] < 0.6: fell = True
            if T_PUSH <= d.time <= DUR and not fell:
                dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
                peak_dev = max(peak_dev, dev)
                if d.time >= t_end_push and dev >= REC_BAND:
                    last_out = d.time
            # --- verdict at the same horizon as the batch ---
            if not done and (d.time >= DUR or fell):
                done = True
                if fell or dev >= REC_BAND: t_rec = float("nan")
                elif last_out is None:      t_rec = 0.0
                else:                       t_rec = last_out - t_end_push
                ok = (not fell) and d.qpos[2] > Z_FALL and dev < COM_REC \
                     and getattr(wbc, "_qp_fail", 0) == 0
                print(f"[verdict t={DUR:.1f}s] {'SUCCESS' if ok else 'FAILURE'}  "
                      f"z={d.qpos[2]:.2f}  dev={dev:.3f}  peak={peak_dev:.3f}  t_rec={t_rec:.2f}s  "
                      f"qp_fails={getattr(wbc, '_qp_fail', 0)}")
            # --- orange push arrow (extended display) ---
            sc = viewer.user_scn; sc.ngeom = 0
            if T_PUSH <= d.time < T_PUSH + ARROW_SHOW:
                fvec = np.array([fx, fy, 0.0]); nfv = np.linalg.norm(fvec)
                dirv = fvec / nfv; L = nfv * ARROW_SCALE
                p_head = d.xpos[wbc.base].copy(); p_tail = p_head - dirv * (L + 0.25)
                g = sc.geoms[sc.ngeom]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                    np.zeros(3), np.eye(3).ravel(),
                                    np.array([1.0, 0.45, 0.0, 1.0], dtype=np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.05, p_tail, p_head)
                sc.ngeom += 1
            viewer.cam.lookat[0] = float(d.xpos[wbc.base][0])
            viewer.cam.lookat[1] = float(d.xpos[wbc.base][1])
            viewer.sync()
            slp = (start + d.time) - _time.time()
            if slp > 0: _time.sleep(slp)


def wilson(k, n, z=1.96):
    if n == 0: return (0.0, 0.0, 0.0)
    p = k/n; den = 1 + z*z/n
    centre = (p + z*z/(2*n))/den
    half = (z*math.sqrt(p*(1-p)/n + z*z/(4*n*n)))/den
    return p, max(0, centre-half), min(1, centre+half)


def make_report(out, n_target, execkind="qp"):
    rows = list(csv.DictReader(open(os.path.join(out, "results.csv"))))
    k = sum(int(r["success"]) for r in rows); n = len(rows)
    p, lo, hi = wilson(k, n)
    L = [f"# Validation statistique S1 — static balancing (executeur : {execkind})\n",
         f"- essais : **{n}** (cible >= {n_target})",
         f"- succes : **{k}/{n}**",
         f"- taux de succes : **{100*p:.1f}%**",
         f"- IC Wilson 95% : **[{100*lo:.1f}%, {100*hi:.1f}%]**",
         f"- seuil these : > 90% — **{'PASS' if lo > 0.90 else ('borderline' if p>0.90 else 'ECHEC')}** "
         f"(critere strict : borne basse IC > 90%)\n",
         f"Randomisation : masse +-{int(100*MASS_PCT)}%, friction +-{int(100*FRIC_PCT)}%, "
         f"bruit etat {INIT_NOISE}, poussee U({F_MIN:.0f},{F_MAX:.0f}) N direction aleatoire, "
         f"impulsion {IMPULSE}s.\n"]
    # --- recovery time + excursion (columns present since 2026-07-16) ---
    if rows and "t_rec" in rows[0]:
        tr = np.array([float(r["t_rec"]) for r in rows])
        pk = np.array([float(r["peak_dev"]) for r in rows if r.get("peak_dev") not in (None, "")])
        settled = tr[~np.isnan(tr)]
        L += ["## Recovery time (bande %.0f mm, depuis fin d'impulsion)\n" % (1000 * REC_BAND),
              f"- essais re-stabilises dans la bande : **{settled.size}/{n}**"]
        if settled.size:
            L += [f"- t_rec : mean **{settled.mean():.2f} s** ± {settled.std():.2f} "
                  f"| median {np.median(settled):.2f} s | max {settled.max():.2f} s"]
        if pk.size:
            L += [f"- excursion CoM post-poussee : mean **{1000*pk.mean():.0f} mm** "
                  f"± {1000*pk.std():.0f} | max {1000*pk.max():.0f} mm"]
        L += [""]
    L += ["## Echecs\n"]
    fails = [r for r in rows if not int(r["success"])]
    if fails:
        L.append("| seed | F (N) | dir (deg) | tombe | z | CoM dev (m) |")
        L.append("|---|---|---|---|---|---|")
        for r in fails:
            L.append(f"| {r['seed']} | {float(r['F']):.0f} | {math.degrees(float(r['theta'])):.0f} "
                     f"| {r['fell']} | {float(r['z']):.2f} | {float(r['com_dev']):.3f} |")
    else:
        L.append("Aucun echec.")
    open(os.path.join(out, "s1_report.md"), "w", encoding="utf-8").write("\n".join(L))
    print(f"\nS1 : {k}/{n} success = {100*p:.1f}%  Wilson CI95 [{100*lo:.1f}, {100*hi:.1f}]  "
          f"-> {'PASS (>90%)' if lo>0.90 else ('p>90% but wide CI' if p>0.90 else 'FAILURE')}")
    print(f"[ok] {out}/s1_report.md")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=25)
    ap.add_argument("--fmin", type=float, default=None)
    ap.add_argument("--fmax", type=float, default=None)
    ap.add_argument("--budget", type=float, default=38.0)
    ap.add_argument("--seed0", type=int, default=1000)
    ap.add_argument("--report", action="store_true")
    ap.add_argument("--out", default="runs/s1")
    ap.add_argument("--exec", dest="execkind", choices=["qp", "ns", "pd"], default="qp",
                    help="executor: qp = QP-WBC measured corners (default); "
                         "ns = null-space task-priority; pd = gravity-compensated PD (H3)")
    ap.add_argument("--viewer", action="store_true",
                    help="replay ONE trial in the MuJoCo viewer (with --seed)")
    ap.add_argument("--seed", type=int, default=None,
                    help="seed of the trial to replay with --viewer (e.g. 1000-1049)")
    ap.add_argument("--pdscale", type=float, default=1.0,
                    help="PD gain stiffening (exec pd): KP*s, KD*sqrt(s); "
                         "1.0 = QP-matched gains (under-tuned); 10 = defensible")
    ap.add_argument("--hip", action="store_true",
                    help="hip strategy (exec qp): lean-into-push, extends the sagittal envelope")
    a = ap.parse_args()
    global F_MIN, F_MAX
    if a.fmin is not None: F_MIN = a.fmin
    if a.fmax is not None: F_MAX = a.fmax
    if a.viewer:
        view_trial(a.seed if a.seed is not None else a.seed0, a.execkind, a.pdscale, a.hip); return
    os.makedirs(a.out, exist_ok=True)
    csv_path = os.path.join(a.out, "results.csv")
    done = set()
    if os.path.exists(csv_path):
        rd = csv.DictReader(open(csv_path))
        if rd.fieldnames and "t_rec" not in rd.fieldnames and not a.report:
            raise SystemExit(f"[!] {csv_path} = old format (no t_rec). "
                             "Use a NEW --out (v2 stays the intact record).")
        for r in rd: done.add(int(r["seed"]))
    else:
        with open(csv_path, "w", newline="") as f:
            csv.writer(f).writerow(["seed", "F", "theta", "fell", "z", "com_dev",
                                    "peak_dev", "t_rec", "qp_fails", "success"])
    todo = [a.seed0 + i for i in range(a.n) if (a.seed0 + i) not in done]
    if a.report or not todo:
        make_report(a.out, a.n, a.execkind); return
    t0 = time.time()
    for sd in todo:
        if time.time() - t0 > a.budget: break
        r = trial(sd, a.execkind, a.pdscale, a.hip)
        with open(csv_path, "a", newline="") as f:
            csv.writer(f).writerow([r["seed"], f"{r['F']:.1f}", f"{r['theta']:.4f}", int(r["fell"]),
                                    f"{r['z']:.3f}", f"{r['com_dev']:.4f}", f"{r['peak_dev']:.4f}",
                                    f"{r['t_rec']:.3f}", r["qp_fails"], r["success"]])
        print(f"  seed {sd}: F={r['F']:.0f}N {'SUCCESS' if r['success'] else 'FAILURE'} "
              f"(z={r['z']:.2f} dev={r['com_dev']:.3f} peak={r['peak_dev']:.3f} t_rec={r['t_rec']:.2f}s)", flush=True)
    nd = sum(1 for _ in open(csv_path)) - 1
    print(f"[progress] {nd}/{a.n} trials" + ("  -> re-invoke to continue" if nd < a.n else "  [ALL DONE]"), flush=True)
    if nd >= a.n: make_report(a.out, a.n, a.execkind)


if __name__ == "__main__":
    main()
