"""
TALOS — S2 : marche 3 m avec TIMING DE PAS ADAPTATIF (event-based touchdown).

Complement direct de WBC_planning_diagnosis.md §4 : le closed-loop --walk-cl
(feedback DCM + capture-point + cpswing) double la survie mais chute vers le
6e pas car la decision reste A HORAIRE FIXE. Ce fichier rend le cycle de
marche EVENEMENTIEL (Khadiv et al., step timing adaptation, T-RO 2020) :

  E1. DEMARRAGE : la marche ne commence pas a une date fixe mais quand le
      DCM mesure atteint la condition d'entree du cycle limite lateral
      (d_lat <= ly - xi_s) pendant le transfert de poids.
  E2. TOUCHDOWN : pendant l'appui simple le DCM lateral diverge en
      d(tau) = d0*e^{omega*tau} ; le poser est déclenché quand d atteint
      la valeur de commutation d* = ly + xi_s :
          T_touchdown = tau + ln(d*/d)/omega   (ne peut que RACCOURCIR le pas)
  E3. GARDE-PIED : le switch de contact n'a lieu que si le pied de
      balancement est effectivement pres du sol (< 1.5 cm) — jamais de
      no-slip dur sur un pied en l'air.

Le QP-WBC (talos_wbc.py, valide S1) et talos_dcm_walk.py restent INCHANGES —
tout est surcharge dans une sous-classe. Les lois de juin (--feedback,
--footfb, --cpswing, --footori) sont actives par defaut, debrayables --no-*.

A placer dans pal_talos/ :
    python talos_dcm_walk_timing.py                 # S2 : 40 pas x 0.08 = 3.2 m
    python talos_dcm_walk_timing.py --steps 10      # debug court
    python talos_dcm_walk_timing.py --no-timing     # ablation (= walk-cl-like)
    python talos_dcm_walk_timing.py --viewer        # viewer MuJoCo temps reel
    python talos_dcm_walk_timing.py --seed 3 --save s2.npz
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B

T_END     = 2.5    # equilibre final apres le dernier pas (s)
XFER_RATE = 0.10   # vitesse de transfert du poids (m/s) en phase initiale
XFER_TMIN = 0.40   # duree minimale du transfert (s)
XFER_TMAX = 4.0    # timeout du transfert (s) — demarre quand meme + warning
FOOT_TOL  = 0.010  # tolerance hauteur pied pour autoriser le touchdown (m)
Z_LAND    = 0.60   # vitesse de descente de la REFERENCE en mode landing (m/s)
Z_DESC    = 0.25   # vitesse de descente PHYSIQUE observee du pied (m/s) — pour l'anticipation
MIN_SEP   = 0.11   # separation laterale min entre centres des pieds (m) — anti-croisement
MAX_SEP   = 0.42   # separation laterale max (m) — anti-grand-ecart (large : autorite de recuperation)
SAG_MAX   = 0.13   # limite de capture sagittale : DCM avant - appui (m) -> poser immediat
# --- soft landing (--softland) : anti-vacillement de cheville au poser ---
DEBOUNCE_N = 2     # ticks de contact consecutifs requis avant bascule d'appui (4 ms @ 500 Hz)
ZLAND_K    = 15.0  # ease-in : v_descente ref = K*h (1/s) — decelere SEULEMENT sous ZLAND_MIN/K
                   # (~1.2 cm) : retard de toucher ~+45 ms vs baseline. v2-v3 (K=8, decel des 2.2 cm
                   # + lag PD du pied ~2 cm) : toucher ~0.15 s en retard -> overshoot DCM (FELL)
ZLAND_MIN  = 0.18  # vitesse plancher d'impact (m/s) — v1=0.10 REJETE (toucher bien trop tardif)
ZDESC_SOFT = 0.18  # vitesse physique de descente observee avec ease-in (pour l'anticipation t_lat)
LAND_WIN   = 0.10  # fenetre des metriques de poser (s) — loggees AVEC ou SANS --softland (A/B)


def apply_squat(m, d, drop):
    """Flechit les genoux dans la CONFIGURATION initiale, sans passer par le
    WBC : hanche -a, genou +2a, cheville -a (pieds a plat), base abaissee
    pour garder les semelles au sol. Le controleur construit ensuite sa
    posture home, zc et omega sur cette pose flechie — sortie de la
    singularite jambe tendue des le premier tick."""
    if drop <= 1e-6:
        return
    L_LEG = 0.76                        # cuisse+tibia TALOS (approx.; base corrigee numeriquement)
    alpha = float(np.arccos(np.clip(1.0 - drop / L_LEG, 0.2, 1.0)))
    feet = [m.body("leg_left_6_link").id, m.body("leg_right_6_link").id]
    z0 = min(float(d.xpos[b][2]) for b in feet)
    for side in ("left", "right"):
        for num, dq in (("3", -alpha), ("4", 2.0 * alpha), ("5", -alpha)):
            j = m.joint("leg_%s_%s_joint" % (side, num))
            d.qpos[m.jnt_qposadr[j.id]] += dq
    mujoco.mj_forward(m, d)
    z1 = min(float(d.xpos[b][2]) for b in feet)
    d.qpos[2] -= (z1 - z0)              # ramener les semelles au sol
    mujoco.mj_forward(m, d)
    print("[squat] genou=%.1f deg  base_z=%.3f m  CoM_z=%.3f m"
          % (np.degrees(2 * alpha), d.qpos[2],
             float(d.subtree_com[m.body("base_link").id][2])))


class DCMWalkT(B.DCMWalk):
    """Machine a pas evenementielle + timing adaptatif au-dessus de DCMWalk."""

    def __init__(self, m, d, t_nom, t_min, t_max, verbose=True):
        super().__init__(m, d)
        # NB : si apply_squat() a ete appele avant, zc et omega (mesures par
        # la classe de base sur la pose flechie) sont deja coherents.
        self.T_nom, self.T_MIN, self.T_MAX = t_nom, t_min, t_max
        self.xi_s = self.ly * np.tanh(self.omega * t_nom / 2.0)
        self.use_timing = True
        # --- LEVIER L1 (--sagfb) : ajustement sagittal du LIEU de pas, couple a
        # la duree adaptative Tk. OFF PAR DEFAUT -> S2 (config gelee, batch N=50)
        # et S5 (230 essais) restent BIT-IDENTIQUES.
        # Motivation : audit_timing_vs_reactive_planners.md -> la loi de juin a
        # bien la loi de duree (Khadiv2020) mais AUCUN ajustement sagittal du
        # lieu ni couplage lieu<->duree ; S5 mesure I50 sagittal 20-30 N.s vs
        # lateral 45-50+ = signature d'un canal sagittal sans mecanisme actif.
        # b_sag = offset DCM sagittal du point fixe du cycle nominal :
        #   b * e^{omega*T} = STEP_LEN + b   =>   b = L / (e^{omega*T} - 1)
        # En regime nominal u_x = xi_pred_x - b_sag == plan : le levier est
        # NO-OP tant que le DCM sagittal ne devie pas (faible risque de regression).
        self.sag_fb = False
        self.sag_dev = 0.06            # ecart max autorise vs plan (m)
        self.b_sag = float(B.STEP_LEN / (np.exp(self.omega * t_nom) - 1.0))
        self.verbose = verbose
        self.k = 0; self._last_k = -1
        self.t0 = None                 # debut du pas courant (None = transfert)
        self.Tk = t_nom                # duree (adaptative) du pas courant
        self.swing_pos = None
        self.ended = None              # instant d'entree en phase END
        # cible finale : dernier appui + pied de fermeture cote a cote
        self.y0 = 0.5 * (float(d.xpos[self.left][1]) + float(d.xpos[self.right][1]))
        last = self.zmp[-1]
        close_y = self.ly if self.support[-1] == self.right else -self.ly
        self.closing = np.array([last[0], close_y])
        self.end_target = 0.5 * (last + self.closing)
        self.log = dict(com_err=[], xi_err=[], ctrl_ms=[], t_steps=[],
                        land_wobble=[], land_grf=[], land_mkbrk=[])
        # mecanismes soft-landing cote timing (opt-in ; sl_imp/sl_ramp sont dans la base)
        self.sl_ease = False           # descente deceleree avant impact
        self.sl_debounce = False       # contact confirme sur DEBOUNCE_N ticks avant bascule
        # etat des metriques de poser + debounce contact
        self._cnt_touch = 0
        self._lw_foot = None; self._lw_t0 = 0.0
        self._lw_w = 0.0; self._lw_grf = 0.0; self._lw_mb = 0; self._lw_prev = True

    def _foot_contact(self, fb):
        """Contact du corps fb avec le SOL (geoms du worldbody, body 0) — reste
        robuste au pied incline (pointe qui touche, cheville haute).
        v4 2026-07-08 : avant, N'IMPORTE QUEL contact du corps comptait (pied-pied,
        pied-jambe) -> poses fantomes a dz=+0.03..0.046 observees run70h."""
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                return True
        return False

    def _grf_z(self, fb):
        """Somme des forces normales de contact pied-SOL (N) sur le corps fb."""
        d = self.d; tot = 0.0; f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                mujoco.mj_contactForce(self.m, d, i, f6)
                tot += abs(float(f6[0]))            # composante normale (repere contact)
        return tot

    def _land_metrics_tick(self):
        """Fenetre LAND_WIN apres chaque poser : pic de vitesse angulaire du pied
        (vacillement), pic de GRF (impact), transitions make/break (rebond).
        Loggee avec ET sans --softland -> comparaison A/B directe pour Ch5/Ch6."""
        if self._lw_foot is None:
            return
        if self.t - self._lw_t0 <= LAND_WIN:
            v6 = np.zeros(6)
            mujoco.mj_objectVelocity(self.m, self.d, mujoco.mjtObj.mjOBJ_BODY,
                                     self._lw_foot, v6, 0)
            self._lw_w = max(self._lw_w, float(np.linalg.norm(v6[:3])))
            self._lw_grf = max(self._lw_grf, self._grf_z(self._lw_foot))
            inc = self._foot_contact(self._lw_foot)
            if inc != self._lw_prev:
                self._lw_mb += 1; self._lw_prev = inc
        else:
            self._flush_land_metrics()

    def _flush_land_metrics(self):
        if self._lw_foot is None:
            return
        self.log["land_wobble"].append(self._lw_w)
        self.log["land_grf"].append(self._lw_grf)
        self.log["land_mkbrk"].append(self._lw_mb)
        self._lw_foot = None

    def _dlat(self, xi_y, k):
        """Distance laterale DCM->appui k, positive vers l'interieur du robot.
        Le cote de l'appui vient de support[k] (G/D), PAS du signe de sa
        position monde — qui s'inverse des que le robot derive lateralement."""
        p_y = self.zmp[k][1]
        zeta = 1.0 if self.support[k] == self.left else -1.0
        return (xi_y - p_y) * (-zeta)

    # ---------- machine a pas ----------
    def update(self, dt):
        self.t += dt
        com, com_v, xi_meas = self._measured_dcm()
        self._land_metrics_tick()                    # metriques de poser (toutes phases)

        # ---- E1 : transfert de poids evenementiel (remplace SETTLE fixe) ----
        if self.t0 is None:
            p0 = self.zmp[0]
            dvec = p0 - self.com_ref_xy; dn = float(np.linalg.norm(dvec))
            if dn > 1e-9:
                self.com_ref_xy = self.com_ref_xy + dvec / dn * min(XFER_RATE * dt, dn)
            self.phase = "DS"; self.stance = self.support[0]
            self.swing = self.left if self.stance == self.right else self.right
            self.xi_ref = self.com_ref_xy.copy()
            d0 = self._dlat(xi_meas[1], 0)
            entry = self.ly - self.xi_s
            if (self.t > XFER_TMIN and d0 <= entry * 1.05) or self.t > XFER_TMAX:
                self.t0 = self.t
                if self.verbose:
                    tag = "TIMEOUT — depart force" if self.t > XFER_TMAX else "condition d'entree atteinte"
                    print("[gait] depart t=%.2f s (%s : dlat=%.3f, entree=%.3f)"
                          % (self.t, tag, d0, entry))
            return

        tau = self.t - self.t0
        k = self.k

        # ---- E2/E3 : evenement de poser = CONTACT reel ----
        # Des que le pied touche pendant la fenetre d'atterrissage, on bascule
        # IMMEDIATEMENT le support : sinon le pied au sol reste hors du set de
        # contact et se fait trainer par les taches de swing encore actives.
        if self.ended is None:
            foot_z = float(self.d.xpos[self.swing][2])
            raw = (foot_z <= self.fz + FOOT_TOL) or self._foot_contact(self.swing)
            # debounce (--softland) : un micro-rebond ne doit pas faire osciller
            # le set de contact DS/SS entre deux ticks
            self._cnt_touch = self._cnt_touch + 1 if raw else 0
            touched = self._cnt_touch >= (DEBOUNCE_N if self.sl_debounce else 1)
            due = (tau >= self.Tk) and (touched or tau >= self.T_MAX)
            early = bool(getattr(self, "_landing", False)) and touched
            if due or early:
                if self.swing_pos is not None:
                    self.swing_from[self.swing] = self.swing_pos.copy()
                self.log["t_steps"].append(min(tau, self.T_MAX))
                # depart de la rampe de contact (--softland) + fenetre de metriques
                self.land_t[self.swing] = self.t
                self._flush_land_metrics()           # fenetre precedente non close (pas < 100 ms)
                self._lw_foot = self.swing; self._lw_t0 = self.t
                self._lw_w = 0.0; self._lw_grf = 0.0; self._lw_mb = 0; self._lw_prev = True
                self._cnt_touch = 0
                if self.verbose:
                    dl = self._dlat(xi_meas[1], k)
                    print("[step %2d] pose %s  T=%.2fs  in=%.3f out=%.3f (d*=%.3f)  pied_dz=%+.3f"
                          % (k, "G" if self.swing == self.left else "D", tau,
                             getattr(self, "d_in", -1.0), dl,
                             self.ly + self.xi_s, foot_z - self.fz))
                self._landing = False
                self._force_land = False
                if k + 1 < self.N:
                    # aligner TOUT LE PLAN RESTANT sur la realite : l'appui
                    # suivant devient le pied reellement pose, et les pas
                    # futurs (+ la geometrie nominale du plan) sont translates
                    # d'autant — plus aucune cible dans un repere fantome
                    p_real = self.d.xpos[self.swing]
                    sx = float(p_real[0]) - self.zmp[k + 1][0]
                    self.zmp[k + 1][0] = float(p_real[0])
                    self.zmp[k + 1][1] = float(p_real[1])
                    # pas futurs : x translate sur la realite, mais le y se
                    # RE-CENTRE vers la ligne de depart (<= 2 cm/pas) -> marche
                    # en ligne droite malgre les derives des recuperations
                    side1 = self.ly if self.support[k + 1] == self.left else -self.ly
                    ym = float(p_real[1]) - side1
                    for j in range(k + 2, self.N):
                        self.zmp[j][0] += sx
                        ym += float(np.clip(self.y0 - ym, -0.02, 0.02))
                        sj = self.ly if self.support[j] == self.left else -self.ly
                        self.zmp[j][1] = ym + sj
                    self.closing[0] += sx
                    self.closing[1] = ym + (self.ly if self.support[-1] == self.right else -self.ly)
                    self.end_target = 0.5 * (self.zmp[-1] + self.closing)
                    self._recompute_dcm()
                    self.k = k = k + 1
                    self.t0 = self.t; tau = 0.0
                    self.Tk = self.T_nom
                else:
                    self.ended = self.t
                    # cible finale = milieu des pieds REELS (pas du plan)
                    pl = self.d.xpos[self.left][:2]; pr = self.d.xpos[self.right][:2]
                    self.end_target = 0.5 * (np.asarray(pl) + np.asarray(pr))

        # ---- phase finale : equilibre statique entre les deux pieds ----
        if self.ended is not None:
            self.phase = "DS"; self.stance = self.support[-1]
            self.swing = self.left if self.stance == self.right else self.right
            self.com_ref_xy = self.com_ref_xy + \
                (self.end_target - self.com_ref_xy) * min(dt / 0.3, 1.0)
            self.xi_ref = self.end_target.copy()
            self.log["xi_err"].append(float(np.linalg.norm(xi_meas - self.xi_ref)))
            self.log["com_err"].append(float(np.linalg.norm(com - self.com_ref_xy)))
            return

        # ---- debut de pas : decision de placement (capture point, base) ----
        if k != self._last_k:
            if self.use_foot_fb:
                delta = self.k_foot * (xi_meas - self.xi_ini[k])
                if self.foot_lat_only: delta[0] = 0.0
                self.set_footstep_offset(delta)
            self._apply_offset(k)                        # + _recompute_dcm()
            self._last_k = k
            # re-planification par pas : la reference laterale repart de
            # l'entree DCM MESUREE (coherence regle de timing <-> reference)
            d0 = self._dlat(xi_meas[1], k)
            self.d_in = float(np.clip(d0, 0.010, 0.9 * (self.ly + self.xi_s)))
            self._force_land = False
            # mode RECUPERATION : entree degeneree (DCM du mauvais cote ou
            # saturee) -> le pas suivant est purement LATERAL, on arrete
            # d'avancer tant que le cycle n'est pas rattrape
            self._recover = (d0 <= 0.005) or (d0 >= 0.85 * (self.ly + self.xi_s))
            if self._recover and self.verbose:
                print("[recov] k=%d entree degeneree (d0=%+.3f) -> pas lateral pur" % (k, d0))

        self.stance = self.support[k]
        self.swing = self.left if self.stance == self.right else self.right
        s = min(max(tau / self.Tk, 0.0), 1.0)

        # ---- E2 : timing adaptatif (canal lateral, en appui simple) ----
        # Suivre l'estimation COURANTE du temps de commutation : un DCM qui
        # fuit avance le poser ; un DCM sage peut allonger jusqu'a T_MAX.
        if self.use_timing and s >= B.DS_OVL:
            dlat = self._dlat(xi_meas[1], k)
            dstar = self.ly + self.xi_s
            dsag = xi_meas[0] - self.zmp[k][0]
            # les DEUX canaux sont predictifs : on pose quand le premier des
            # deux DCM (lateral ou sagittal) atteindra sa valeur de commutation
            if dlat > 1e-4:
                t_lat_rem = np.log(max(dstar / dlat, 1e-9)) / self.omega
            else:
                # DCM du mauvais cote de l'appui -> poser IMMEDIATEMENT
                # (et descendre le pied tout de suite, sans repasser par la cloche)
                t_lat_rem = 0.0
                self._force_land = True
            if dsag > 1e-4:
                t_sag_rem = np.log(max(SAG_MAX / dsag, 1e-9)) / self.omega
            else:
                t_sag_rem = 1e9
            t_rem = min(t_lat_rem, max(t_sag_rem, 0.0))
            self.Tk = float(np.clip(tau + max(t_rem, 0.0), self.T_MIN, self.T_MAX))

        # ---- reference DCM : x = recursion (temps virtuel), y = cycle limite ----
        tv = s * self.T_nom                              # temps virtuel sagittal
        xi_x = self.zmp[k][0] + (self.xi_ini[k][0] - self.zmp[k][0]) * \
            np.exp(self.omega * tv)
        p_y = self.zmp[k][1]
        zeta = 1.0 if self.support[k] == self.left else -1.0
        d_in = getattr(self, "d_in", self.ly - self.xi_s)
        xi_y = p_y + (-zeta) * d_in * np.exp(self.omega * min(tau, self.Tk))
        xi = np.array([xi_x, xi_y])
        self.xi_ref = xi.copy()

        # ---- feedback DCM -> reference CoM STABLE (forme validee en juin) ----
        if self.use_dcm_fb:
            e = np.clip(xi - xi_meas, -self.fb_clip, self.fb_clip)
            xi_cmd = xi + self.k_dcm * e
        else:
            xi_cmd = xi
        self.com_ref_xy = self.com_ref_xy + dt * self.omega * (xi_cmd - self.com_ref_xy)

        # ---- pied de balancement (duree variable geree par s = tau/Tk) ----
        if k + 1 < self.N:
            goal = np.array([self.zmp[k+1][0], self.zmp[k+1][1], self.fz])
        else:
            goal = np.array([self.closing[0], self.closing[1], self.fz])
        if getattr(self, "_recover", False):
            goal[0] = self.zmp[k][0]          # recuperation : pas lateral pur,
                                              # aucune progression sagittale
        # phase : fin de pas en DS SEULEMENT si le pied est reellement pose —
        # jamais de contrainte no-slip sur un pied en l'air
        foot_z = float(self.d.xpos[self.swing][2])
        landed = (foot_z <= self.fz + FOOT_TOL) or self._foot_contact(self.swing)
        if s < B.DS_OVL:
            self.phase = "DS"
        elif landed and s > 1 - B.DS_OVL:
            self.phase = "DS"
        else:
            self.phase = "SS"
        if self.use_cp_swing and self.phase == "SS":
            # poser le pied AU-DELA du DCM mesure, decale de off_lat :
            # reproduit l'entree du cycle au pas suivant (Khadiv). off_lat >
            # (ly - xi_s) nominal pour compenser la poussee du double appui
            # (ZMP entre les pieds -> le DCM derive vers le nouvel appui).
            sgn = 1.0 if self.swing == self.left else -1.0
            # viser le DCM PREDIT au moment du poser : pendant la descente du
            # pied (~0.15 s) le DCM continue de diverger (~x1.6) — viser le
            # DCM courant fait atterrir le pied "en retard", offset mange
            t_go = min(max(self.Tk - tau, 0.0), 0.30)
            p_yst = self.zmp[k][1]
            xi_pred = p_yst + (xi_meas[1] - p_yst) * np.exp(self.omega * t_go)
            off = getattr(self, "off_lat", self.ly - self.xi_s)
            if getattr(self, "_recover", False):
                off = max(off, 0.11)          # recuperation : poser plus loin au-dela du DCM
            gy = xi_pred + sgn * off
            # bornes RELATIVES AU PIED D'APPUI reel : jamais moins de min_sep
            # entre les pieds (anti-croisement), jamais plus de MAX_SEP
            p_st = float(self.d.xpos[self.stance][1])
            msep = getattr(self, "min_sep", MIN_SEP)
            if self.swing == self.left:
                gy = min(max(gy, p_st + msep), p_st + MAX_SEP)
            else:
                gy = max(min(gy, p_st - msep), p_st - MAX_SEP)
            # ---- L1 : ajustement sagittal du lieu de pas (capture point) ----
            # u_x = xi_pred_x(t_go) - b_sag, avec le MEME t_go que le canal
            # lateral -> le LIEU est couple a la DUREE adaptative Tk (c'est le
            # couplage lieu<->duree absent de la loi de juin).
            # Garde-fous : (1) inactif en mode recuperation (le pas lateral pur
            # reste prioritaire) ; (2) borne a +-sag_dev autour du plan = une
            # CORRECTION, pas une replanification ; (3) jamais en arriere de
            # l'appui courant -> progression sagittale monotone.
            gx = goal[0]
            if self.sag_fb and not getattr(self, "_recover", False):
                p_xst = self.zmp[k][0]
                xi_pred_x = p_xst + (xi_meas[0] - p_xst) * np.exp(self.omega * t_go)
                u_x = xi_pred_x - self.b_sag
                gx = goal[0] + float(np.clip(u_x - goal[0], -self.sag_dev, self.sag_dev))
                gx = max(gx, p_xst)
            goal = np.array([gx, gy, self.fz])
        zdesc = ZDESC_SOFT if self.sl_ease else Z_DESC
        t_lat = min(max((foot_z - self.fz) / zdesc + 0.06, 0.0), 0.30)
        if (tau >= self.Tk - t_lat or getattr(self, "_force_land", False)) and not landed:
            # LANDING demande : descente active, cible FIGEE (plus de poursuite
            # du DCM en fin de vol), swing task actif jusqu'au contact
            if not getattr(self, "_landing", False):
                self._landing = True
                self._land_goal = goal.copy()
            g = self._land_goal
            z_prev = self.swing_pos[2] if self.swing_pos is not None else self.fz + B.STEP_H
            vz = Z_LAND
            if self.sl_ease:
                # ease-in : la reference decelere pres du sol -> impact ~ZLAND_MIN
                # au lieu de ~Z_DESC (l'energie d'impact chute d'un facteur ~(0.25/0.10)^2)
                vz = float(np.clip(ZLAND_K * (foot_z - self.fz), ZLAND_MIN, Z_LAND))
            self.swing_pos = np.array([g[0], g[1], max(self.fz, z_prev - vz * dt)])
        else:
            frm = self.swing_from[self.swing]
            self.swing_pos = (1 - s) * frm + s * goal
            self.swing_pos[2] = self.fz + B.STEP_H * np.sin(np.pi * s)

        # ---- debug : etat du timing toutes les ~0.1 s ----
        if getattr(self, "debug", False):
            self._dbg = getattr(self, "_dbg", 0) + 1
            if self._dbg % 50 == 0:
                print("[dbg] k=%d tau=%.2f Tk=%.2f s=%.2f dlat=%+.3f dsag=%+.3f dz=%+.3f %s%s"
                      % (k, tau, self.Tk, s, self._dlat(xi_meas[1], k),
                         xi_meas[0] - self.zmp[k][0],
                         float(self.d.xpos[self.swing][2]) - self.fz, self.phase,
                         " LAND" if getattr(self, "_landing", False) else ""))

        # ---- metriques ----
        self.log["xi_err"].append(float(np.linalg.norm(xi_meas - xi)))
        self.log["com_err"].append(float(np.linalg.norm(com - self.com_ref_xy)))

    def control(self):
        t0 = time.perf_counter()
        super().control()
        self.log["ctrl_ms"].append((time.perf_counter() - t0) * 1e3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--steps", type=int, default=40)
    ap.add_argument("--steplen", type=float, default=0.08)
    ap.add_argument("--tstep", type=float, default=0.5, help="duree nominale d'un pas (s)")
    ap.add_argument("--tmin", type=float, default=0.30)
    ap.add_argument("--tmax", type=float, default=None, help="defaut: 1.6 x tstep")
    ap.add_argument("--dsovl", type=float, default=0.25)
    ap.add_argument("--offlat", type=float, default=0.05,
                    help="offset lateral du poser au-dela du DCM predit (m) ; nominal ly-xi_s~0.028, majore pour compenser le double appui")
    ap.add_argument("--minsep", type=float, default=0.14,
                    help="separation laterale minimale entre les centres des deux pieds (m)")
    ap.add_argument("--sagfb", action="store_true",
                    help="LEVIER L1 : ajustement sagittal du lieu de pas (u_x = xi_pred_x - b_sag) "
                         "couple a la duree adaptative Tk. OFF par defaut -> config S2 gelee et "
                         "campagne S5 bit-identiques. A/B : --sagfb vs sans, memes seeds.")
    ap.add_argument("--sagdev", type=float, default=0.06,
                    help="L1 : ecart sagittal max autorise vs plan (m) — borne la correction")
    ap.add_argument("--sagsub", action="store_true",
                    help="L1 en SUBSTITUTION : desactive le terme sagittal du footfb "
                         "(foot_lat_only=True) pour que L1 REMPLACE la boucle sagittale existante "
                         "au lieu de s'y superposer. Sans ce flag les deux lois se combattent : "
                         "footfb pousse la cible en avant (1x/pas), L1 la ramene au capture point "
                         "(continu) -> freinage systematique, batch N=50 du 2026-07-20 = 0/50.")
    ap.add_argument("--bsagk", type=float, default=1.0,
                    help="L1 : facteur de CALIBRATION de b_sag. b_sag theorique (LIPM ideal) "
                         "SURESTIME l'offset DCM sagittal du gait reel de ~26 %% (mesure : offset "
                         "reel 0.0415 m vs point fixe LIPM 0.0563 m) -> le levier freine a chaque "
                         "pas (correction mediane -17 mm sur un pas de 80 mm). 0.74 = calibre.")
    ap.add_argument("--kdcm", type=float, default=1.0)
    ap.add_argument("--kfoot", type=float, default=1.0)
    ap.add_argument("--offmax", type=float, default=0.20)
    ap.add_argument("--dist", type=float, default=3.0, help="distance cible S2 (m)")
    ap.add_argument("--zcdrop", type=float, default=0.04,
                    help="flexion initiale des genoux via abaissement (m) de la CONFIGURATION de depart (0 = jambes tendues)")
    ap.add_argument("--wfoot", type=float, default=6000.0,
                    help="poids de la tache semelle-a-plat du pied de balancement (base: 1500)")
    ap.add_argument("--wfootst", type=float, default=500.0,
                    help="poids semelle-a-plat du pied d'APPUI (complement de la contrainte dure)")
    ap.add_argument("--softland", action="store_true",
                    help="poser doux COTE IMPACT (anti-vacillement cheville) : descente deceleree, "
                         "debounce contact, impedance d'orientation programmee — le transfert de "
                         "charge (contraintes dures, FZ_MIN) reste INSTANTANE comme la baseline")
    ap.add_argument("--sl-ramp", dest="slramp", action="store_true",
                    help="EXPERIMENTAL : rampe de transfert de charge (rotation dure differee + "
                         "FZ_MIN rampe). Desynchronise le transfert vs plan ZMP a switch instantane "
                         "-> oscillation periode-2 (FELL k=7 v1, k=11 v2, 2026-07-08). Ablation Ch6.")
    ap.add_argument("--tramp", type=float, default=0.06,
                    help="duree de la rampe / fenetre d'impedance (s)")
    ap.add_argument("--zlk", type=float, default=15.0,
                    help="ease-in : gain de descente v=K*h (1/s) ; deceleration sous zlmin/K metres")
    ap.add_argument("--zlmin", type=float, default=0.18,
                    help="ease-in : vitesse plancher d'impact (m/s)")
    ap.add_argument("--floortc", type=float, default=0.0,
                    help="adoucit le contact du SOL : solref timeconst (s), ex. 0.02 ; 0 = modele "
                         "par defaut. Test de sensibilite au modele de contact (Ch3 par.3.5) — "
                         "independant de --softland")
    ap.add_argument("--no-timing", dest="timing", action="store_false")
    ap.add_argument("--no-feedback", dest="feedback", action="store_false")
    ap.add_argument("--no-footfb", dest="footfb", action="store_false")
    ap.add_argument("--no-cpswing", dest="cpswing", action="store_false")
    ap.add_argument("--no-footori", dest="footori", action="store_false")
    ap.add_argument("--limit-cycle", dest="limit_cycle", action="store_true",
                    help="injection qvel initiale (optionnel — le demarrage evenementiel suffit normalement)")
    ap.add_argument("--debug", action="store_true", help="trace interne du timing toutes les ~0.1 s")
    ap.add_argument("--seed", type=int, default=None, help="bruit initial qvel (essais batch)")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--viewer", action="store_true", help="viewer MuJoCo temps reel (relance auto si chute)")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()
    if a.tmax is None: a.tmax = 1.6 * a.tstep
    global ZLAND_K, ZLAND_MIN
    ZLAND_K, ZLAND_MIN = a.zlk, a.zlmin
    # parametres de gait via les globals du module de base (convention etablie)
    B.N_STEPS = a.steps; B.STEP_LEN = a.steplen; B.T_STEP = a.tstep; B.DS_OVL = a.dsovl
    B.STEP_H = 0.04     # cloche plus basse : moins de distance a descendre au poser
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0   # chevilles fermes (900/90 + wfootst 1500
                                          # teste 2026-07-06 : FELL — vole les couples
                                          # de cheville necessaires au ZMP)
    B.KP_SW, B.KD_SW = 420.0, 42.0       # swing plus rapide : cibles laterales
                                          # larges atteignables en < 0.3 s

    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    if a.floortc > 0:                     # sol plus mou : timeconst du solref des geoms du worldbody
        nfl = 0
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:
                m.geom_solref[g][0] = a.floortc; nfl += 1
        print("[floor] solref timeconst=%.3f s applique a %d geom(s) du sol" % (a.floortc, nfl))
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv)
        mujoco.mj_forward(m, d)
    x0 = float(d.subtree_com[m.body("base_link").id][0])
    y0m = float(d.subtree_com[m.body("base_link").id][1])

    def build(d_):
        apply_squat(m, d_, a.zcdrop)     # flexion AVANT construction du WBC
        c_ = DCMWalkT(m, d_, t_nom=a.tstep, t_min=a.tmin, t_max=a.tmax)
        c_.use_timing = a.timing
        c_.use_dcm_fb = a.feedback; c_.k_dcm = a.kdcm
        c_.use_foot_fb = a.footfb; c_.k_foot = a.kfoot
        if a.footfb: c_.OFF_MAX = np.array([a.offmax, a.offmax])
        c_.use_cp_swing = a.cpswing
        c_.use_foot_ori = a.footori
        c_.w_foot_swing = a.wfoot
        c_.w_foot_stance = a.wfootst
        c_.off_lat = a.offlat
        c_.min_sep = a.minsep
        c_.sag_fb = a.sagfb                  # LEVIER L1 (defaut False = config gelee)
        c_.sag_dev = a.sagdev
        c_.b_sag *= a.bsagk                  # calibration du point fixe sur le gait REEL
        if a.sagsub: c_.foot_lat_only = True # L1 remplace (au lieu de doubler) le footfb sagittal
        c_.sl_ease = c_.sl_debounce = c_.sl_imp = a.softland
        c_.sl_ramp = a.slramp
        c_.T_RAMP = a.tramp
        c_.debug = a.debug
        if a.limit_cycle: B.init_limit_cycle(c_, d_)
        return c_

    c = build(d)
    dt = m.opt.timestep

    # ---------- viewer temps reel (meme pattern que view_talos_dcm.py) ----------
    if a.viewer:
        from mujoco import viewer as mjv
        with mjv.launch_passive(m, d) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            start = time.time()
            while viewer.is_running():
                c.update(dt); c.control(); mujoco.mj_step(m, d)
                viewer.cam.lookat[0] = float(d.xpos[c.base][0])
                viewer.cam.lookat[1] = float(d.xpos[c.base][1])
                viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0: time.sleep(slp)
                if d.qpos[2] < 0.5:                      # chute -> relance auto
                    print("[viewer] chute a t=%.2f s (pas k=%d) — relance" % (c.t, c.k))
                    time.sleep(0.6)
                    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
                    c = build(d); start = time.time()
        return

    t_cap = 6.0 + a.steps * a.tmax + T_END
    render = None
    if a.video:
        import talos_sim; render = talos_sim.render
    fe = max(1, int(1 / (30 * dt))); frames = []
    fell_at = None
    while c.t < t_cap:
        c.update(dt); c.control(); mujoco.mj_step(m, d)
        if d.qpos[2] < 0.6:
            fell_at = c.t; break
        if c.ended is not None and (c.t - c.ended) > T_END:
            break
        if a.video and int(c.t / dt) % fe == 0:
            frames.append(render(m, d))

    # ---------- bilan S2 ----------
    com = d.subtree_com[c.base]
    dist = float(com[0] - x0)
    upright = d.qpos[2] > 0.8 and fell_at is None
    success = upright and dist >= a.dist and c.ended is not None
    ce = np.array(c.log["com_err"]) if c.log["com_err"] else np.array([np.nan])
    xe = np.array(c.log["xi_err"]) if c.log["xi_err"] else np.array([np.nan])
    cm = np.array(c.log["ctrl_ms"]); ts = np.array(c.log["t_steps"])
    nq = int(getattr(c, "_qp_fail", 0)); ntick = max(len(cm), 1)
    print("=" * 66)
    sl = []
    if a.softland: sl.append("ease+deb+imp")
    if a.slramp:   sl.append("ramp")
    sl_desc = ("+".join(sl) + "(T=%.2fs)" % a.tramp) if sl else "False"
    if a.floortc > 0: sl_desc += " floortc=%.3f" % a.floortc
    print("S2 TIMING  steps=%d len=%.2f Tnom=%.2f [%.2f,%.2f] zc=%.3f w=%.2f seed=%s timing=%s softland=%s" %
          (a.steps, a.steplen, a.tstep, a.tmin, a.tmax, c.zc, c.omega, a.seed, a.timing, sl_desc))
    print("  levier L1    : sagfb=%s%s" %
          (a.sagfb, ("  (b_sag=%.4f m [k=%.2f], dev max %.3f m)" % (c.b_sag, a.bsagk, a.sagdev)) if a.sagfb else ""))
    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, pas k=%d)" % (fell_at, c.k)))
    print("  distance     : %.2f m (cible %.1f) -> %s | derive laterale %.2f m" %
          (dist, a.dist, "SUCCESS" if success else "FAIL", float(com[1] - y0m)))
    print("  pas atteint  : k=%d / %d | pas poses=%d" % (c.k, a.steps - 1, len(ts)))
    if len(ts):
        print("  duree de pas : mean %.2f s | min %.2f | max %.2f (adaptatif)" %
              (ts.mean(), ts.min(), ts.max()))
    print("  CoM err (xy) : RMSE %.1f mm | max %.1f mm" %
          (np.sqrt(np.nanmean(ce**2)) * 1e3, np.nanmax(ce) * 1e3))
    print("  DCM err      : mean %.1f mm | max %.1f mm" %
          (np.nanmean(xe) * 1e3, np.nanmax(xe) * 1e3))
    lw = np.array(c.log["land_wobble"]); lg = np.array(c.log["land_grf"])
    lmb = np.array(c.log["land_mkbrk"], dtype=float)
    if len(lw):
        print("  poser 100 ms : |w_pied| pic %.2f rad/s (mean %.2f) | GRF pic %.0f N (mean %.0f)"
              " | make/break mean %.1f" %
              (lw.max(), lw.mean(), lg.max(), lg.mean(), lmb.mean()))
    if len(cm):
        print("  ctrl loop    : mean %.2f ms | p99 %.2f ms | QP fails %d/%d (%.1f%% feas)" %
              (cm.mean(), np.percentile(cm, 99), nq, ntick, 100.0 * (1 - nq / ntick)))
    print("=" * 66)
    if a.save:
        np.savez(a.save, com_err=ce, xi_err=xe, ctrl_ms=cm, t_steps=ts,
                 e_mech=c.E_mech, e_sq=c.E_sq, mass=c.mass,
                 land_wobble=lw, land_grf=lg, land_mkbrk=lmb,
                 softland=a.softland, slramp=a.slramp, tramp=a.tramp,
                 floortc=a.floortc, zlk=a.zlk, zlmin=a.zlmin,
                 sagfb=a.sagfb, sagdev=a.sagdev, b_sag=c.b_sag,
                 dist=dist, success=success, fell_at=fell_at or -1.0, seed=a.seed or -1)
        print("[ok] %s" % a.save)
    if a.video and frames:
        import imageio.v2 as imageio
        imageio.mimsave("talos_dcm_walk_timing.mp4", frames, fps=30)
        print("[ok] talos_dcm_walk_timing.mp4")


if __name__ == "__main__":
    main()
