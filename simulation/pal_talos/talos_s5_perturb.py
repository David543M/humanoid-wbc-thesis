"""
TALOS — S5 : robustesse aux perturbations (impulsions pendant la marche S2).

Spec : s5_design.md (2026-07-10). Marcheur = DCMWalkT (S2, config gelee
--steps 70 --offlat 0.08 --dsovl 0.12 --tmin 0.24 = DEFAUTS ici), fichiers
S2 INCHANGES. La perturbation est injectee dans la BOUCLE DE SIMULATION
(d.xfrc_applied sur base_link), pas dans le controleur : le WBC traverse la
poussee sans information, comme un robot reel.

Protocole par essai :
  - la poussee part quand le pas k atteint --pushstep (regime etabli), a une
    phase aleatoire du pas tiree de default_rng(seed + 1000) — decouplee du
    bruit initial qvel (seed) pour que le bras controle (mag 0) partage
    exactement l'etat initial ;
  - force constante F pendant --pushdur (impulsion I = F*dur), directions
    monde {+x,-x,+y,-y} ;
  - ELIGIBLE = debout et en marche a t_push ; RECOVERED = debout (base_z>0.8)
    a t_push + --trec, parmi les eligibles. Verdict binaire UNIQUE ;
    grandeurs mecanisme (excursion DCM max, temps de retour en bande, pas
    [recov] post-push) RAPPORTEES mais hors critere.

⚠ Dette assumee (cf. s5_design §7) : le bloc de setup gait (globals B/W)
est copie de talos_dcm_walk_timing.main() — a re-synchroniser si la config
gelee S2 evolue.

Usage (terminal Anaconda, depuis pal_talos/) :
    python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --save s5_run.npz
    python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --viewer
    python talos_s5_perturb.py --pushmag 100 --pushdir +y --viewer --manual
                                          # poussee sur ENTREE/ESPACE (exploration)
    python talos_s5_perturb.py --seed 0 --pushmag 0                  # bras controle
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B
import talos_dcm_walk_timing as T

DIRS = {"+x": np.array([1.0, 0.0, 0.0]), "-x": np.array([-1.0, 0.0, 0.0]),
        "+y": np.array([0.0, 1.0, 0.0]), "-y": np.array([0.0, -1.0, 0.0])}
# alias SANS tiret initial : argparse avale "-y" comme une option (bug sonde
# 2026-07-10 : « --pushdir: expected one argument ») -> px/mx/py/my
DIR_ALIAS = {"px": "+x", "mx": "-x", "py": "+y", "my": "-y",
             "x": "+x", "y": "+y", "x+": "+x", "y+": "+y",
             "x-": "-x", "y-": "-y"}


def norm_dir(s):
    s = DIR_ALIAS.get(s.strip().lower(), s.strip().lower())
    if s not in DIRS:
        raise SystemExit("[ABORT] --pushdir invalide : %r (attendu +x/-x/+y/-y "
                         "ou alias px/mx/py/my)" % s)
    return s
BAND_FLOOR = 0.050   # bande de retour DCM : max(2 x mediane pre-push, 50 mm)
BAND_HOLD  = 0.50    # duree de maintien en bande pour valider le retour (s)


def main():
    ap = argparse.ArgumentParser()
    # ---- config gelee S2 par DEFAUT (s2_batch.FROZEN) ----
    ap.add_argument("--steps", type=int, default=70)
    ap.add_argument("--steplen", type=float, default=0.08)
    ap.add_argument("--tstep", type=float, default=0.5)
    ap.add_argument("--tmin", type=float, default=0.24)
    ap.add_argument("--tmax", type=float, default=None)
    ap.add_argument("--dsovl", type=float, default=0.12)
    ap.add_argument("--offlat", type=float, default=0.08)
    ap.add_argument("--minsep", type=float, default=0.14)
    ap.add_argument("--kdcm", type=float, default=1.0)
    ap.add_argument("--kfoot", type=float, default=1.0)
    ap.add_argument("--offmax", type=float, default=0.20)
    ap.add_argument("--dist", type=float, default=3.0)
    ap.add_argument("--zcdrop", type=float, default=0.04)
    ap.add_argument("--wfoot", type=float, default=6000.0)
    ap.add_argument("--wfootst", type=float, default=500.0)
    # --- LEVIER L1 (sagittal) : mêmes flags/semantique que talos_dcm_walk_timing.py.
    # OFF par defaut -> la campagne S5 de reference (230 essais) reste bit-identique.
    ap.add_argument("--sagfb", action="store_true",
                    help="L1 : ajustement sagittal du lieu de pas couple a la duree adaptative Tk")
    ap.add_argument("--sagsub", action="store_true",
                    help="L1 en substitution : coupe le terme sagittal du footfb (foot_lat_only)")
    ap.add_argument("--sagdev", type=float, default=0.06,
                    help="L1 : ecart sagittal max autorise vs plan (m)")
    ap.add_argument("--bsagk", type=float, default=1.0,
                    help="L1 : calibration de b_sag sur le gait reel (0.74 = mesure 2026-07-20)")
    # ---- perturbation S5 ----
    ap.add_argument("--pushmag", type=float, default=100.0,
                    help="force de poussee (N) ; 0 = bras controle (aucune force)")
    ap.add_argument("--pushdir", type=str, default="+y",
                    help="direction monde : +x/-x/+y/-y ; sous cmd.exe utiliser "
                         "les alias px/mx/py/my OU la forme --pushdir=-y "
                         "(argparse avale '-y' nu)")
    ap.add_argument("--pushdur", type=float, default=0.10, help="duree de la poussee (s)")
    ap.add_argument("--pushstep", type=int, default=12,
                    help="pas k de declenchement (regime etabli)")
    ap.add_argument("--pushphase", type=float, default=None,
                    help="phase du pas [0,1) ; defaut = tirage rng(seed+1000)")
    ap.add_argument("--trec", type=float, default=5.0,
                    help="fenetre de verdict : debout a t_push + trec (s)")
    ap.add_argument("--manual", action="store_true",
                    help="viewer uniquement : la poussee part quand on appuie sur "
                         "ENTREE ou ESPACE (re-declenchable) au lieu du protocole "
                         "k/phase — exploration qualitative, PAS pour la campagne")
    # ---- divers ----
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()
    a.pushdir = norm_dir(a.pushdir)
    if a.tmax is None:
        a.tmax = 1.6 * a.tstep

    # ---- setup gait S2 (copie assumee de talos_dcm_walk_timing.main) ----
    B.N_STEPS = a.steps; B.STEP_LEN = a.steplen; B.T_STEP = a.tstep; B.DS_OVL = a.dsovl
    B.STEP_H = 0.04
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0

    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv)
        mujoco.mj_forward(m, d)
    x0 = float(d.subtree_com[m.body("base_link").id][0])

    # phase de poussee : rng DEDIE (seed+1000) -> le bras controle (mag 0)
    # partage le meme etat initial que les cellules poussees
    phase = a.pushphase
    if phase is None:
        phase = float(np.random.default_rng((a.seed or 0) + 1000).uniform(0.0, 1.0))
    u = DIRS[a.pushdir]

    T.apply_squat(m, d, a.zcdrop)
    c = T.DCMWalkT(m, d, t_nom=a.tstep, t_min=a.tmin, t_max=a.tmax)
    c.use_timing = True
    c.use_dcm_fb = True; c.k_dcm = a.kdcm
    c.use_foot_fb = True; c.k_foot = a.kfoot
    c.OFF_MAX = np.array([a.offmax, a.offmax])
    c.use_cp_swing = True
    c.use_foot_ori = True
    c.w_foot_swing = a.wfoot
    c.w_foot_stance = a.wfootst
    c.off_lat = a.offlat
    c.min_sep = a.minsep
    c.sag_fb = a.sagfb                    # LEVIER L1 (defaut False = campagne S5 de reference)
    c.sag_dev = a.sagdev
    c.b_sag *= a.bsagk
    if a.sagsub: c.foot_lat_only = True
    if a.sagfb:
        print("[L1] sagfb=True  sub=%s  b_sag=%.4f m (k=%.2f)  dev=%.3f m"
              % (a.sagsub, c.b_sag, a.bsagk, a.sagdev))
    c.debug = a.debug
    base = c.base
    dt = m.opt.timestep

    # ---- etat de la poussee + instrumentation ----
    # armed=True MEME a mag 0 : le bras controle declenche la meme fenetre
    # virtuelle (t_push, T_REC) -> eligible/recovered definis identiquement,
    # la comparaison par cellule est litteralement a fenetre egale
    st = dict(armed=True, t_push=None, i_push=None, i_end=None,
              eligible=False, pushing=False, recov_pre=0, recov_post=0,
              done_recover=None, fire=False)

    def _fire_push():
        first = st["t_push"] is None
        st["t_push"] = c.t
        if first:                    # metriques/verdict = 1ere poussee seulement
            st["i_push"] = len(c.log["com_err"])
            st["k_push"] = c.k
            st["eligible"] = d.qpos[2] > 0.8
            st["recov_pre"] = _recov_count()
        d.xfrc_applied[base, :3] = a.pushmag * u
        st["pushing"] = True
        print("[push] t=%.2f s  k=%d  F=%.0f N %s  dur=%.2f s  (I=%.1f N.s)"
              "  eligible=%s%s"
              % (c.t, c.k, a.pushmag, a.pushdir, a.pushdur,
                 a.pushmag * a.pushdur, st["eligible"],
                 "  [MANUEL]" if a.manual else "  phase=%.2f" % phase))

    def push_tick():
        """Gestion de la fenetre de force ; retourne True pendant la poussee."""
        if st["pushing"]:
            if c.t >= st["t_push"] + a.pushdur:
                d.xfrc_applied[base, :3] = 0.0
                st["pushing"] = False
                st["i_end"] = len(c.log["com_err"])
        elif a.manual:
            # declenchement A LA DEMANDE (touche ENTREE/ESPACE, re-declenchable)
            if st["fire"]:
                st["fire"] = False
                _fire_push()
        elif st["t_push"] is None:
            # protocole : pas k atteint + phase du pas courant depassee
            if (st["armed"] and c.t0 is not None and c.ended is None
                    and c.k >= a.pushstep and (c.t - c.t0) >= phase * c.Tk):
                _fire_push()
        return st["pushing"]

    def _recov_count():
        return int(getattr(c, "_n_recov", 0))

    # compter les pas [recov] sans toucher la classe : wrapper sur update
    _upd = c.update
    def upd(dt_):
        k_before = c.k
        _upd(dt_)
        if c.k != k_before and getattr(c, "_recover", False):
            c._n_recov = _recov_count() + 1
    c.update = upd

    # ---- boucle ----
    if a.manual and not a.viewer:
        raise SystemExit("[ABORT] --manual necessite --viewer (declenchement clavier) ; "
                         "en headless le declenchement est le protocole k/phase")
    if a.viewer:
        from mujoco import viewer as mjv
        ARROW_SHOW = 0.6      # duree d'AFFICHAGE de la fleche (s) — decouplee des 0.1 s
                              # de physique (patron S1, talos_wbc.py --push)
        ARROW_SCALE = 0.0015  # longueur de fleche par Newton
        KEY_ENTER, KEY_KPENTER, KEY_SPACE = 257, 335, 32   # keycodes GLFW

        def key_cb(keycode):
            # NB : le viewer passif ne transmet PAS les modificateurs (Ctrl…) ;
            # le declenchement est donc sur ENTREE ou ESPACE simple
            if a.manual and keycode in (KEY_ENTER, KEY_KPENTER, KEY_SPACE):
                st["fire"] = True

        if a.manual:
            print("[manual] poussee A LA DEMANDE : appuyer sur ENTREE ou ESPACE "
                  "dans la fenetre du viewer (re-declenchable ; F=%.0f N %s)"
                  % (a.pushmag, a.pushdir))
        with mjv.launch_passive(m, d, key_callback=key_cb) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True   # fleches GRF aux pieds
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True
            start = time.time()
            while viewer.is_running():
                c.update(dt); c.control(); push_tick(); mujoco.mj_step(m, d)
                # --- fleche orange de la poussee (queue hors du torse, pointe sur le bassin) ---
                sc = viewer.user_scn; sc.ngeom = 0
                if (st["t_push"] is not None and a.pushmag > 0.0
                        and st["t_push"] <= c.t < st["t_push"] + ARROW_SHOW):
                    L = a.pushmag * ARROW_SCALE
                    p_head = d.xpos[base].copy()
                    p_tail = p_head - u * (L + 0.25)
                    g = sc.geoms[sc.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                        np.zeros(3), np.eye(3).ravel(),
                                        np.array([1.0, 0.45, 0.0, 1.0], dtype=np.float32))
                    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.05, p_tail, p_head)
                    sc.ngeom += 1
                viewer.cam.lookat[0] = float(d.xpos[base][0])
                viewer.cam.lookat[1] = float(d.xpos[base][1])
                viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0:
                    time.sleep(slp)
                if d.qpos[2] < 0.5:
                    print("[viewer] chute a t=%.2f s (pas k=%d)" % (c.t, c.k))
                    break
        return

    t_cap = 6.0 + a.steps * a.tmax + T.T_END
    fell_at = None
    while c.t < t_cap:
        c.update(dt); c.control(); push_tick(); mujoco.mj_step(m, d)
        if d.qpos[2] < 0.6:
            fell_at = c.t; break
        if c.ended is not None and (c.t - c.ended) > T.T_END:
            break
        # verdict fige des que la fenetre T_REC est ecoulee (l'essai continue)
        if (st["t_push"] is not None and st["done_recover"] is None
                and c.t >= st["t_push"] + a.pushdur + a.trec):
            st["done_recover"] = bool(d.qpos[2] > 0.8)
    if st["pushing"]:                                  # chute pendant la poussee
        d.xfrc_applied[base, :3] = 0.0
    if st["t_push"] is not None and st["done_recover"] is None:
        # chute (ou fin d'essai) avant la fin de la fenetre
        st["done_recover"] = bool(fell_at is None and d.qpos[2] > 0.8)

    # ---- analyse post-push (mecanisme, hors verdict) ----
    ce = np.array(c.log["com_err"]) if c.log["com_err"] else np.array([np.nan])
    xe = np.array(c.log["xi_err"]) if c.log["xi_err"] else np.array([np.nan])
    cm = np.array(c.log["ctrl_ms"]); ts = np.array(c.log["t_steps"])
    xi_max_post = np.nan; t_reband = np.nan; band = np.nan
    if st["i_push"] is not None and st["i_push"] < len(xe):
        # fenetre mecanisme = [t_push, t_push + dur + T_REC] (spec s5_design §5),
        # PAS jusqu'a la fin du run : les excursions [recov] tardives du regime
        # de base (bring-up #1 : xi 247->? mm venait de k=52) ne comptent pas
        i_end_win = st["i_push"] + int(round((a.pushdur + a.trec) / dt))
        pre = xe[:st["i_push"]]; post = xe[st["i_push"]:i_end_win]
        band = max(2.0 * float(np.nanmedian(pre)) if pre.size else BAND_FLOOR, BAND_FLOOR)
        xi_max_post = float(np.nanmax(post)) if post.size else np.nan
        hold = max(1, int(BAND_HOLD / dt))
        ok = post < band
        for i in range(len(ok) - hold + 1):
            if ok[i:i + hold].all():
                t_reband = i * dt
                break
    n_recov_post = _recov_count() - st["recov_pre"] if st["t_push"] is not None else 0

    # ---- bilan ----
    com = d.subtree_com[base]
    dist = float(com[0] - x0)
    upright = d.qpos[2] > 0.8 and fell_at is None
    success = upright and dist >= a.dist and c.ended is not None
    nq = int(getattr(c, "_qp_fail", 0)); ntick = max(len(cm), 1)
    print("=" * 66)
    print("S5 PERTURB  F=%.0f N %s x %.2f s (I=%.1f N.s)  pushstep=%d phase=%.2f"
          "  trec=%.1f  seed=%s"
          % (a.pushmag, a.pushdir, a.pushdur, a.pushmag * a.pushdur,
             a.pushstep, phase, a.trec, a.seed))
    if st["t_push"] is None:
        print("  poussee      : NON APPLIQUEE (chute/fin avant k=%d)" % a.pushstep)
    else:
        print("  poussee      : t=%.2f s (k=%d)  eligible=%s" %
              (st["t_push"], st.get("k_push", -1), st["eligible"]))
        print("  RECOVERED    : %s  (debout a t_push+%.1f s)" %
              (st["done_recover"], a.trec))
        print("  mecanisme    : xi_max_post=%.0f mm | retour bande(%.0f mm)=%s"
              " | pas [recov] post=%d"
              % (xi_max_post * 1e3, band * 1e3,
                 "%.2f s" % t_reband if np.isfinite(t_reband) else "jamais",
                 n_recov_post))
    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, pas k=%d)" % (fell_at, c.k)))
    print("  distance     : %.2f m (cible %.1f) -> %s" %
          (dist, a.dist, "SUCCESS" if success else "FAIL"))
    print("  CoM err (xy) : RMSE %.1f mm | max %.1f mm" %
          (np.sqrt(np.nanmean(ce ** 2)) * 1e3, np.nanmax(ce) * 1e3))
    if len(cm):
        print("  ctrl loop    : mean %.2f ms | p99 %.2f ms | QP fails %d/%d (%.1f%% feas)" %
              (cm.mean(), np.percentile(cm, 99), nq, ntick, 100.0 * (1 - nq / ntick)))
    print("=" * 66)

    if a.save:
        np.savez(a.save, com_err=ce, xi_err=xe, ctrl_ms=cm, t_steps=ts,
                 dist=dist, success=success, fell_at=fell_at or -1.0,
                 seed=a.seed if a.seed is not None else -1,
                 push_mag=a.pushmag, push_dir=a.pushdir, push_dur=a.pushdur,
                 push_step=a.pushstep, push_phase=phase, trec=a.trec,
                 sagfb=a.sagfb, sagsub=a.sagsub, sagdev=a.sagdev, bsagk=a.bsagk,
                 push_applied=st["t_push"] is not None,
                 t_push=st["t_push"] or -1.0,
                 i_push=st["i_push"] if st["i_push"] is not None else -1,
                 eligible=st["eligible"],
                 recovered=st["done_recover"] if st["done_recover"] is not None else False,
                 xi_max_post=xi_max_post, t_reband=t_reband, band=band,
                 n_recov_post=n_recov_post)
        print("[ok] %s" % a.save)


if __name__ == "__main__":
    main()
