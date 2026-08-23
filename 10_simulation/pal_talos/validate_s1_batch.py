"""
validate_s1_batch.py — validation STATISTIQUE du scenario S1 (static balancing).

Protocole (the thesis methodology chapter) : tenir debout sous poussees externes ; succes > 90 % sur
N >= 20 essais, intervalle de confiance de Wilson 95 %. Chaque essai applique une
randomisation de domaine (masse, friction, etat initial) + une poussee aleatoire
(amplitude, direction). Succes = reste debout, recupere, 0 repli QP.

RESUMABLE : chaque essai est persiste dans results.csv (tolerant aux timeouts).
  python validate_s1_batch.py --n 25 --budget 38 --out runs/s1     # rappeler jusqu'a [ALL DONE]
  python validate_s1_batch.py --report --out runs/s1               # forcer le rapport

Ajouts 2026-07-16 (backup : validate_s1_batch_BACKUP_20260716.py) :
  - recovery time t_rec par essai (metrique S1 ancree the thesis methodology chapter, jamais chiffree) :
    t_rec = duree entre la FIN de l'impulsion et le dernier instant ou la deviation
    CoM plane sort de la bande REC_BAND = 20 mm (i.e. rentre ET RESTE dans la bande
    jusqu'a la fin de l'essai). 20 mm = ~1/3 de la demi-semelle laterale mesuree
    (60 mm), >> regulation debout pre-poussee (~1 mm), << critere de succes (80 mm).
    NaN si chute ou si la bande n'est pas re-acquise a la fin de l'essai.
  - peak_dev par essai (excursion CoM max post-poussee).
  - --exec {qp,ns,pd} : executeur QP-WBC (defaut) ou baselines H3 (s1_baselines.py :
    ns = task-priority null-space Sentis-Khatib ; pd = PD gravite-compense).
    MEME protocole/seeds -> table comparative Ch5 SS5.4.
  ⚠ colonnes CSV etendues : utiliser un NOUVEAU --out (ex. runs/s1_corners_v3,
    runs/s1_ns, runs/s1_pd) ; runs/s1_corners_v2 reste le record 50/50 intact.
  - --viewer --seed N : rejoue l'essai N dans le viewer MuJoCo (MEMES tirages que
    le batch : masse/friction/bruit/poussee), verdict imprime a t=DUR.
      python validate_s1_batch.py --viewer --seed 1000 --exec pd
"""
import argparse, os, csv, time, math, numpy as np, mujoco
import talos_wbc as W
import s1_baselines as BL

# distribution de poussee (flag Ch3 : non ancree, choisie) + randomisation de domaine
F_MIN, F_MAX = 100.0, 300.0     # amplitude de poussee (N), tiree U(F_MIN, F_MAX)
T_PUSH, IMPULSE = 2.0, 0.1      # instant + duree de l'impulsion (s)
DUR = 4.5                       # duree de l'essai (s)
MASS_PCT, FRIC_PCT, INIT_NOISE = 0.10, 0.20, 0.01
Z_FALL = 0.8                    # debout si z > 0.8
COM_REC = 0.08                  # recupere si deviation CoM finale < 8 cm
REC_BAND = 0.02                 # bande de recuperation (m) pour t_rec (voir docstring)


def _setup(seed, execkind="qp", pdscale=1.0, hip=False):
    """Etat initial + executeur + poussee d'un essai — PARTAGE batch/viewer.
    L'ordre des tirages rng est FIGE (batch et viewer voient le meme essai)."""
    rng = np.random.default_rng(seed)
    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    base_mass = m.body_mass.copy(); base_fric = m.geom_friction.copy()
    # --- randomisation de domaine ---
    m.body_mass[:] = base_mass * (1.0 + rng.uniform(-MASS_PCT, MASS_PCT, base_mass.shape))
    f = base_fric.copy(); f[:, 0] *= (1.0 + rng.uniform(-FRIC_PCT, FRIC_PCT))
    m.geom_friction[:] = np.clip(f, 1e-3, None)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    d.qpos[7:] += rng.uniform(-INIT_NOISE, INIT_NOISE, d.qpos[7:].shape)
    d.qvel[:] += rng.uniform(-INIT_NOISE, INIT_NOISE, d.qvel.shape)
    mujoco.mj_forward(m, d)
    if execkind == "qp":
        wbc = W.WBC(m, d, corners=W.CORNERS_MEASURED, hip=hip)   # coins mesures (S1)
    elif execkind == "pd":
        wbc = BL.PDGrav(m, d, scale=pdscale)            # baseline H3 (pd, gains x scale)
    else:
        wbc = BL.EXECUTORS[execkind](m, d)              # baseline H3 (ns)
    # --- poussee aleatoire (amplitude + direction horizontale) ---
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
        if d.time >= T_PUSH:                            # suivi post-poussee (t_rec, peak)
            dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
            peak_dev = max(peak_dev, dev)
            if d.time >= t_end_push and dev >= REC_BAND:
                last_out = d.time
    com_dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
    # t_rec : rentre ET reste dans REC_BAND jusqu'a la fin ; NaN si chute/non re-acquis
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
    """Rejoue l'essai `seed` dans le viewer MuJoCo (memes tirages que le batch).
    Verdict imprime a t=DUR ; la sim continue ensuite jusqu'a fermeture."""
    import time as _time
    from mujoco import viewer as mjv
    m, d, wbc, F, th = _setup(seed, execkind, pdscale, hip)
    dt = m.opt.timestep
    fx, fy = F*np.cos(th), F*np.sin(th)
    t_end_push = T_PUSH + IMPULSE
    ARROW_SHOW, ARROW_SCALE = 0.6, 0.0015
    print(f"[viewer] seed {seed}  exec={execkind}  F={F:.0f} N  dir={math.degrees(th):.0f} deg  "
          f"poussee a t={T_PUSH:.1f}s pendant {IMPULSE:.1f}s")
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
            # --- verdict au meme horizon que le batch ---
            if not done and (d.time >= DUR or fell):
                done = True
                if fell or dev >= REC_BAND: t_rec = float("nan")
                elif last_out is None:      t_rec = 0.0
                else:                       t_rec = last_out - t_end_push
                ok = (not fell) and d.qpos[2] > Z_FALL and dev < COM_REC \
                     and getattr(wbc, "_qp_fail", 0) == 0
                print(f"[verdict t={DUR:.1f}s] {'SUCCES' if ok else 'ECHEC'}  "
                      f"z={d.qpos[2]:.2f}  dev={dev:.3f}  peak={peak_dev:.3f}  t_rec={t_rec:.2f}s  "
                      f"qp_fails={getattr(wbc, '_qp_fail', 0)}")
            # --- fleche orange de poussee (affichage prolonge) ---
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
    # --- recovery time + excursion (colonnes presentes depuis 2026-07-16) ---
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
    print(f"\nS1 : {k}/{n} succes = {100*p:.1f}%  IC95 Wilson [{100*lo:.1f}, {100*hi:.1f}]  "
          f"-> {'PASS (>90%)' if lo>0.90 else ('p>90% mais IC large' if p>0.90 else 'ECHEC')}")
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
                    help="executeur : qp = QP-WBC coins mesures (defaut) ; "
                         "ns = task-priority null-space ; pd = PD gravite-compense (H3)")
    ap.add_argument("--viewer", action="store_true",
                    help="rejoue UN essai dans le viewer MuJoCo (avec --seed)")
    ap.add_argument("--seed", type=int, default=None,
                    help="seed de l'essai a rejouer avec --viewer (ex. 1000-1049)")
    ap.add_argument("--pdscale", type=float, default=1.0,
                    help="raidissement gains PD (exec pd) : KP*s, KD*sqrt(s) ; "
                         "1.0 = gains apparies QP (sous-regle) ; 10 = defendable")
    ap.add_argument("--hip", action="store_true",
                    help="levier hanche (exec qp) : lean-into-push, etend l'enveloppe sagittale")
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
            raise SystemExit(f"[!] {csv_path} = ancien format (sans t_rec). "
                             "Utiliser un NOUVEAU --out (v2 reste le record intact).")
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
        print(f"  seed {sd}: F={r['F']:.0f}N {'SUCCES' if r['success'] else 'ECHEC'} "
              f"(z={r['z']:.2f} dev={r['com_dev']:.3f} peak={r['peak_dev']:.3f} t_rec={r['t_rec']:.2f}s)", flush=True)
    nd = sum(1 for _ in open(csv_path)) - 1
    print(f"[progress] {nd}/{a.n} essais" + ("  -> rappelle pour continuer" if nd < a.n else "  [ALL DONE]"), flush=True)
    if nd >= a.n: make_report(a.out, a.n, a.execkind)


if __name__ == "__main__":
    main()
