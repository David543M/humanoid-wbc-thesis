"""
TALOS — S3 : MONTEE D'ESCALIER par SEQUENCEUR QUASI-STATIQUE.

Motivation (voir Ch5 §5.6) : le marcheur plat
DCM (talos_dcm_walk_timing) repose sur un TIMING EVENEMENTIEL (touchdown
adaptatif, capture-point, recovery) qui s'est revele FONDAMENTALEMENT
INCOMPATIBLE avec les gros transferts de poids deliberes d'un escalier :
le timing raccourcit le pas a T_MIN des que le DCM derive, terminant le
mount avant que le corps ait bascule sur la marche.

Ce fichier ABANDONNE ce timing. Il garde UNIQUEMENT l'executeur QP-WBC
valide (DCMWalk.control() : dynamique corps-rigide dure, cone de friction,
limites de couple, taches CoM / orientation / posture / pied ; PROXQP) et
pilote une MACHINE A ETATS EXPLICITE quasi-statique :

    SETTLE                     : les 2 pieds au sol, CoM centre, posture etablie
    pour chaque pas de plan :
      TRANSFER (double appui)  : les 2 pieds PORTENT (contact dur dans le QP),
                                 le CoM bascule (x,y) sur le pied d'APPUI +
                                 la hauteur de CoM monte a la marche d'appui,
                                 jusqu'a convergence (pas de course contre une
                                 horloge : on attend le transfert).
      SWING   (appui simple)   : SEUL le pied d'appui porte ; le pied libre
                                 suit un arc UP-OVER-DOWN vers le centre du
                                 giron fige ; le CoM reste sur l'appui.
      -> pose (contact) -> pas suivant.
    DONE                       : equilibre final sur le palier.

Le point cle : en DS, control() met les DEUX pieds dans l'ensemble de
contact (contrainte no-slip dure sur chacun) -> le transfert de poids
lateral+vertical se fait a deux pieds, ce qui manquait au marcheur plat.
Aucune horloge de pas ne peut interrompre le transfert : la transition
DS->SS n'a lieu QUE lorsque le CoM a effectivement bascule.

Usage (depuis pal_talos/, conda base) :
    python talos_s3_stairs_qs.py                     # bring-up
    python talos_s3_stairs_qs.py --viewer
    python talos_s3_stairs_qs.py --trace --save s3qs.npz
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B
import talos_dcm_walk_timing as T          # apply_squat
import talos_s3_stairs as S3               # make_risers, top_x_of, build_stairs_xml


class StairQS(B.DCMWalk):
    """Sequenceur quasi-statique de montee d'escalier au-dessus de l'executeur QP."""

    def __init__(self, m, d, risers, tread, max_mount=0.28,
                 rate=0.5, zrate=0.4, clear=0.14, tswing=0.8,
                 t_settle=0.8, t_tr_min=0.4, t_tr_max=1.6, postol=0.05):
        B.N_STEPS = 2; B.STEP_LEN = 0.1        # plan plat de la base : ignore
        super().__init__(m, d)                 # W.WBC + control() + feet/home/zc/omega
        self.dt = float(m.opt.timestep)
        self._risers = risers; self._tread = tread; self._max_mount = max_mount
        self.fz0 = float(self.fz)              # hauteur cheville pied au sol (~0.104)
        self.zc0 = float(self.zc)
        # --- drapeaux executeur (control() les lit) ---
        self.use_hard_contact = True; self.use_foot_ori = True
        # w_foot_stance RELEVE 500->1400 (POLISH #6a) : le pied d'APPUI doit rester
        # TOTALEMENT plat/pose pendant le swing. A 500 (vs swing 2500) la reaction du
        # pied qui se leve fait BASCULER la semelle d'appui sur une arete (rocking, le
        # pied "quitte le sol un peu"). On raidit la tache pied-porteur (position+ori du
        # sole a plat) sans toucher a la trajectoire de swing ni au CoM (2500). Bonus :
        # sur le palier haut, un pied d'appui plus ferme donne plus d'appui a la base
        # contre le tangage de la jonction finale (aide la recuperation au sommet).
        self.w_foot_swing = 2500.0; self.w_foot_stance = 1400.0  # swing < CoM (equilibre prioritaire)
        self.sl_imp = False; self.sl_ramp = False; self.T_RAMP = 0.06; self.land_t = {}
        self.use_dcm_fb = False
        # --- reglages sequenceur ---
        self.RATE = rate; self.ZRATE = zrate; self.CROSS_CLEAR = clear
        self.T_SWING = tswing; self.POS_TOL = postol
        self.T_SETTLE = t_settle; self.T_TR_MIN = t_tr_min; self.T_TR_MAX = t_tr_max
        self.VTOL = 0.03                       # vitesse CoM max pour finir le transfert (m/s)
        # SEEK d'atterrissage : si l'arc finit sans contact, on presse le pied vers le
        # giron (jusqu'a SEEK_MAX sous la cible) au lieu de basculer en DONE pied en l'air.
        self.SEEK_RATE = 0.10; self.SEEK_MAX = 0.06; self.T_SEEK_MAX = 1.6
        # biais AVANT de la cible CoM : la cheville n'est pas au centre de la semelle
        # (talon −0.125 / pointe +0.075) et la montee tend a laisser le CoM EN RETRAIT
        # (bascule arriere en haut). On vise legerement vers la pointe.
        self.COM_FWD = 0.05
        self._com_prev = self.d.subtree_com[self.base][:2].copy()
        self._com_speed = 0.0
        # --- etat physique des pieds (positions plantees courantes) ---
        self.foot_pos = {fb: self.d.xpos[fb].copy() for fb in self.feet}
        self._build_moves()
        # --- etat machine ---
        self.state = "SETTLE"; self.t = 0.0; self.t_state = 0.0; self.mi = -1
        self.phase = "DS"; self.stance = self.right; self.swing = self.left
        self.swing_pos = self.d.xpos[self.left].copy()
        self.com_ref_xy = self.d.subtree_com[self.base][:2].copy()
        self.swing_from = self.foot_pos[self.left].copy()
        self.target = self.foot_pos[self.left].copy()
        self._maxclimb = 0.0
        self.log = dict(clear=[], grf=[], ctrl_ms=[])
        self._swmin = None; self._swact = False

    # ---------- geometrie escalier ----------
    def _sh(self, x):
        h = 0.0
        for xe, z in self._risers:
            if x >= xe - 1e-6:
                h = z
        return h

    def _build_moves(self):
        """Sequence de deplacements de pieds : approche a petits pas (pied bride
        avant la 1ere contremarche) puis montee step-together (centre du giron)."""
        ly = self.ly; tread = self._tread; risers = self._risers
        yof = lambda side: ly if side == self.left else -ly
        rx = float(self.d.xpos[self.right][0])
        foots = []                              # (side, x) ; index 0 = appui initial (pas un move)
        side = self.right; foots.append((side, rx)); x = rx
        ctr0 = risers[0][0] + 0.5 * tread
        TOE, MARG = 0.075, 0.03
        x_app_max = risers[0][0] - TOE - MARG
        g = 0
        while (ctr0 - x) > self._max_mount and g < 80:
            side = self.left if side == self.right else self.right
            x = min(x + 0.08, x_app_max); foots.append((side, x)); g += 1
            if x >= x_app_max - 1e-9:
                break
        self._n_approach = len(foots)           # nb de footholds d'approche (incl. initial)
        for t in range(len(risers)):
            ctr = risers[t][0] + 0.5 * tread
            side = self.left if side == self.right else self.right; foots.append((side, ctr))
            side = self.left if side == self.right else self.right; foots.append((side, ctr))
        self.moves = []
        for (side, x) in foots[1:]:
            self.moves.append((side, np.array([x, yof(side), self.fz0 + self._sh(x)])))
        print("[plan-QS] %d moves (appui initial D@x=%.3f) :" % (len(self.moves), rx))
        for i, (s, t3) in enumerate(self.moves):
            tag = "app" if i < self._n_approach - 1 else "CLIMB"
            print("   m%02d swing %s -> x=%.3f y=%+.3f z=%.3f  [%s]"
                  % (i, "G" if s == self.left else "D", t3[0], t3[1], t3[2], tag))

    # ---------- helpers pilotage ----------
    def _approach(self, xy, zc):
        r = self.RATE * self.dt
        self.com_ref_xy = self.com_ref_xy + np.clip(np.asarray(xy, float) - self.com_ref_xy, -r, r)
        self.zc += float(np.clip(zc - self.zc, -self.ZRATE * self.dt, self.ZRATE * self.dt))

    def _arc(self, frm, tgt, s):
        """Trajectoire cheville UP-OVER-DOWN : monte vertical au-dessus du depart,
        avance a hauteur de pic (semelle au-dessus du nez), descend sur le giron."""
        peak = max(float(frm[2]), float(tgt[2])) + self.CROSS_CLEAR
        LIFT, FWD = 0.30, 0.72
        if s < LIFT:
            xf = 0.0
            z = float(frm[2]) + (peak - float(frm[2])) * np.sin(np.pi / 2 * (s / LIFT))
        elif s < FWD:
            u = (s - LIFT) / (FWD - LIFT); xf = u * u * (3.0 - 2.0 * u); z = peak
        else:
            u = (s - FWD) / (1.0 - FWD); xf = 1.0
            z = float(tgt[2]) + (peak - float(tgt[2])) * np.cos(np.pi / 2 * u)
        x = float(frm[0]) + (float(tgt[0]) - float(frm[0])) * xf
        y = float(frm[1]) + (float(tgt[1]) - float(frm[1])) * xf
        return np.array([x, y, z])

    def _contact(self, fb):
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                return True
        return False

    def _grf_z(self, fb):
        d = self.d; tot = 0.0; f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                mujoco.mj_contactForce(self.m, d, i, f6); tot += abs(float(f6[0]))
        return tot

    def _clear_tick(self):
        if self.phase == "SS":
            sw = self.swing
            sole = float(self.d.xpos[sw][2]) - self.fz0
            gap = sole - self._sh(float(self.d.xpos[sw][0]))
            self._swmin = gap if self._swmin is None else min(self._swmin, gap)
            self._swact = True
        elif self._swact and self._swmin is not None:
            self.log["clear"].append(self._swmin); self._swmin = None; self._swact = False

    def _begin_move(self, i):
        self.mi = i
        if i >= len(self.moves):
            self.state = "DONE"; self.t_state = self.t; return
        side, tgt = self.moves[i]
        self.swing = side
        self.stance = self.left if side == self.right else self.right
        self.target = np.asarray(tgt, float)
        self.swing_from = self.foot_pos[side].copy()
        self.state = "TRANSFER"; self.t_state = self.t

    # ---------- machine a etats ----------
    def update(self, dt):
        self.t += dt
        com_now = self.d.subtree_com[self.base][:2].copy()
        self._com_speed = float(np.linalg.norm(com_now - self._com_prev)) / max(dt, 1e-6)
        self._com_prev = com_now
        st = self.state
        if st == "SETTLE":
            self.phase = "DS"
            ctr = 0.5 * (self.foot_pos[self.left][:2] + self.foot_pos[self.right][:2])
            self._approach(ctr, self.zc0)
            if self.t - self.t_state > self.T_SETTLE:
                self._begin_move(0)

        elif st == "TRANSFER":
            self.phase = "DS"
            st_live = np.asarray(self.d.xpos[self.stance][:2], float)   # pied VIVANT
            tgt = np.array([st_live[0], st_live[1]])                    # CoM sur l'appui vivant
            self._approach(tgt, self.zc0 + self._sh(float(self.d.xpos[self.stance][0])))
            self.swing_pos = self.foot_pos[self.swing].copy()
            tau = self.t - self.t_state
            com = self.d.subtree_com[self.base][:2]
            com_ok = np.linalg.norm(com - tgt) < self.POS_TOL
            settled = self._com_speed < self.VTOL      # CoM ARRETE sur l'appui (anti-coast)
            if (tau > self.T_TR_MIN and com_ok and settled) or tau > self.T_TR_MAX:
                self.state = "SWING"; self.t_state = self.t

        elif st == "SWING":
            self.phase = "SS"
            tau = self.t - self.t_state
            s = min(tau / self.T_SWING, 1.0)
            self.swing_pos = self._arc(self.swing_from, self.target, s)
            st_live = self.d.xpos[self.stance][:2]
            # APPUI SIMPLE : CoM strictement au-dessus du pied porteur (pas de biais avant).
            self._approach([float(st_live[0]), float(st_live[1])],
                           self.zc0 + self._sh(float(self.d.xpos[self.stance][0])))
            self._clear_tick()
            landed = self._contact(self.swing) and s >= 0.9
            if s >= 1.0 and (landed or tau > self.T_SWING + 0.6):
                self.foot_pos[self.swing] = self.d.xpos[self.swing].copy()
                self._maxclimb = max(self._maxclimb,
                                     self._sh(float(self.d.xpos[self.swing][0])))
                self._begin_move(self.mi + 1)

        elif st == "DONE":
            self.phase = "DS"
            # SETTLE FINAL — recuperation confinee a DONE (moves 0-7 = montee 3/3 propre,
            # intouches). Le dernier pied de swing (m08) peut ne PAS s'etre pose : sur le
            # palier haut le bassin tangue et la cible mondiale est ratee (GRF=0, pied en
            # l'air) -> l'ancienne version basculait en arriere sur UN seul pied.
            # Deux temps :
            #   (1) pied libre PAS pose -> on le PRESSE vers le giron (seek z) ET on tient
            #       le CoM ferme sur le pied D'APPUI (deja sur le palier) + biais avant :
            #       ca DE-tangue la base et fait descendre le pied jusqu'au contact ;
            #   (2) deux pieds au sol -> CoM au centre du polygone sommet.
            if not self._contact(self.swing):
                over = self.t - self.t_state
                seek = min(self.SEEK_MAX, self.SEEK_RATE * over)
                self.swing_pos = np.array([float(self.target[0]), float(self.target[1]),
                                           float(self.target[2]) - seek])
                stf = self.d.xpos[self.stance][:2]
                ctr = np.array([float(stf[0]) + self.COM_FWD, float(stf[1])])
            else:
                self.swing_pos = self.d.xpos[self.swing].copy()
                lf = self.d.xpos[self.left][:2]; rf = self.d.xpos[self.right][:2]
                ctr = np.array([0.5 * (float(lf[0]) + float(rf[0])) + self.COM_FWD,
                                0.5 * (float(lf[1]) + float(rf[1]))])
            self._approach(ctr, self.zc0 + self._sh(float(self.d.xpos[self.stance][0])))

    def control(self):
        t0 = time.perf_counter()
        super().control()
        self.log["ctrl_ms"].append((time.perf_counter() - t0) * 1e3)
        for fb in self.feet:                    # suivi GRF (pic)
            self.log["grf"].append(self._grf_z(fb))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x0", type=float, default=0.30)
    ap.add_argument("--tread", type=float, default=0.28)
    ap.add_argument("--hriser", type=float, default=0.10)
    ap.add_argument("--maxmount", type=float, default=0.28)
    ap.add_argument("--rate", type=float, default=0.18, help="vitesse ref CoM xy (m/s) — lent = CoM suit sans overshoot")
    ap.add_argument("--zrate", type=float, default=0.35, help="vitesse rampe hauteur CoM (m/s)")
    ap.add_argument("--clear", type=float, default=0.14, help="garde swing au-dessus du nez (m)")
    ap.add_argument("--tswing", type=float, default=0.65, help="duree de l'arc de swing (s) — 0.65 = montee 3/3 propre ; PLUS LONG (1.0) reintroduit la derive laterale en appui simple (regression QS #5b)")
    ap.add_argument("--tsettle", type=float, default=0.8)
    ap.add_argument("--ttrmin", type=float, default=0.5)
    ap.add_argument("--ttrmax", type=float, default=3.0, help="temps max de transfert (attend le CoM arrete)")
    ap.add_argument("--postol", type=float, default=0.035, help="tolerance CoM<->appui pour finir le transfert (m)")
    ap.add_argument("--zcdrop", type=float, default=0.04)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--trace", action="store_true")
    ap.add_argument("--traceevery", type=int, default=50)
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()

    risers = S3.make_risers(a.x0, a.tread, a.hriser)
    top_target = risers[-1][1]
    with open("scene_stairs.xml", "w") as f:
        f.write(S3.build_stairs_xml(a.x0, a.tread, a.hriser))
    print("[scene] scene_stairs.xml genere : x0=%.2f tread=%.2f h=%.2f n=%d"
          % (a.x0, a.tread, a.hriser, len(risers)))

    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0
    # CoM = PRIORITE en appui simple. Le trace #1 a montre que les taches du pied de
    # swing (W_SWING/w_foot_swing = 6000) ECRASENT la tache CoM (400) -> le CoM derive
    # hors du pied porteur (demi-semelle ±0.06 m) et le pendule inverse (omega~3.3/s)
    # diverge. On donne au CoM une autorite comparable au swing + gains raidis/amortis,
    # et on abaisse le poids du swing pour qu'il ne vole pas l'equilibre.
    B.KP_COM_W, B.KD_COM_W, B.W_COM_W = 500.0, 180.0, 2500.0
    B.W_SWING = 2500.0
    # BASSIN A PLAT : W_ORI par defaut (40) est ridicule vs CoM/swing (2500) -> le
    # bassin part en tangage pendant les grands swings de montee, ce qui PROJETTE le
    # pied de swing au-dela de sa cible (collision, cf trace stair-3). On raidit fort.
    # 600 = valeur de la montee 3/3 PROPRE (QS #5). Monter a 1100 n'a PAS aide et a
    # coincide avec la regression laterale (QS #5b) -> on garde 600. Le pitch -19 deg
    # transitoire du swing-jonction m08 est rattrape par le SEEK (le pied SE POSE) +
    # recuperation DONE a DEUX pieds, pas par un raidissement orientation global.
    W.KP_ORI, W.KD_ORI, W.W_ORI = 450.0, 80.0, 600.0
    W.MODEL = "scene_stairs.xml"
    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv); mujoco.mj_forward(m, d)
    com0 = d.subtree_com[m.body("base_link").id].copy()
    x0m, z0m = float(com0[0]), float(com0[2])

    def build(d_):
        T.apply_squat(m, d_, a.zcdrop)
        return StairQS(m, d_, risers, a.tread, max_mount=a.maxmount,
                       rate=a.rate, zrate=a.zrate, clear=a.clear, tswing=a.tswing,
                       t_settle=a.tsettle, t_tr_min=a.ttrmin, t_tr_max=a.ttrmax, postol=a.postol)

    c = build(d)
    dt = m.opt.timestep

    def trc(cc):
        com = d.subtree_com[cc.base]
        q = d.qpos[3:7]
        import math
        pitch = math.degrees(math.asin(max(-1, min(1, 2 * (q[0] * q[2] - q[3] * q[1])))))
        print("[trc] t=%.2f %-8s mi=%d %-2s | CoM x=%.3f y=%.3f z=%.3f  ref x=%.3f y=%.3f zc=%.3f | "
              "pitch=%+.1f bz=%.3f | sw %s x=%.3f z=%.3f cmd x=%.3f z=%.3f | stx=%.3f | GRF L=%.0f R=%.0f"
              % (cc.t, cc.state, cc.mi, cc.phase, com[0], com[1], com[2],
                 cc.com_ref_xy[0], cc.com_ref_xy[1], cc.zc, pitch, d.qpos[2],
                 "G" if cc.swing == cc.left else "D", d.xpos[cc.swing][0], d.xpos[cc.swing][2],
                 cc.swing_pos[0], cc.swing_pos[2], d.xpos[cc.stance][0],
                 cc._grf_z(cc.left), cc._grf_z(cc.right)))

    if a.viewer:
        from mujoco import viewer as mjv
        with mjv.launch_passive(m, d) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            start = time.time(); last = -1
            while viewer.is_running():
                c.update(dt); c.control(); mujoco.mj_step(m, d)
                if a.trace and int(round(c.t / dt)) % a.traceevery == 0:
                    trc(c)
                viewer.cam.lookat[0] = float(d.xpos[c.base][0])
                viewer.cam.lookat[1] = float(d.xpos[c.base][1])
                viewer.cam.lookat[2] = float(d.xpos[c.base][2])
                viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0: time.sleep(slp)
                if d.qpos[2] < 0.5:
                    print("[viewer] chute t=%.2f s (%s) — relance" % (c.t, c.state))
                    time.sleep(0.6); mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
                    c = build(d); start = time.time()
        return

    t_cap = c.T_SETTLE + len(c.moves) * (c.T_TR_MAX + c.T_SWING + c.T_SEEK_MAX) + 3.0
    fell_at = None; _pstate = None
    while c.t < t_cap:
        c.update(dt); c.control(); mujoco.mj_step(m, d)
        if a.trace and (c.state != _pstate or int(round(c.t / dt)) % a.traceevery == 0):
            trc(c); _pstate = c.state
        if d.qpos[2] < 0.6:
            fell_at = c.t
            if a.trace: print("[trc] ===== CHUTE ====="); trc(c)
            break
        if c.state == "DONE" and (c.t - c.t_state) > 3.0:
            break

    com = d.subtree_com[c.base]
    dist = float(com[0] - x0m); climb = float(com[2] - z0m)
    upright = d.qpos[2] > 0.8 and fell_at is None
    n_climbed = int(round(c._maxclimb / a.hriser)) if a.hriser > 0 else 0
    reached_top = c._maxclimb >= top_target - 1e-6
    success = upright and reached_top and c.state == "DONE"
    cl = np.array(c.log["clear"]) if c.log["clear"] else np.array([0.0])
    gr = np.array(c.log["grf"]) if c.log["grf"] else np.array([np.nan])
    cm = np.array(c.log["ctrl_ms"]) if c.log["ctrl_ms"] else np.array([np.nan])
    nq = int(getattr(c, "_qp_fail", 0)); ntick = max(len(cm), 1)
    print("=" * 70)
    print("S3-QS  escalier x0=%.2f tread=%.2f h=%.2f | moves=%d rate=%.2f tswing=%.2f seed=%s"
          % (a.x0, a.tread, a.hriser, len(c.moves), a.rate, a.tswing, a.seed))
    print("  outcome    : %s%s | etat final=%s"
          % ("UPRIGHT" if upright else "FELL",
             "" if fell_at is None else "  (t=%.2f s)" % fell_at, c.state))
    print("  montee     : %d/%d marches (max appui %.2f m) | gain CoM z %.3f m -> %s"
          % (n_climbed, len(risers), c._maxclimb, climb, "SUCCESS" if success else "FAIL"))
    print("  avancee x  : %.2f m | move atteint mi=%d/%d" % (dist, c.mi, len(c.moves) - 1))
    print("  clearance  : min %.3f m | mean %.3f m | franchissements=%d"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clear"])))
    print("  GRF (pic)  : %.0f N | mean %.0f N" % (np.nanmax(gr), np.nanmean(gr)))
    print("  ctrl loop  : mean %.2f ms | p99 %.2f ms | QP feas %.1f%%"
          % (np.nanmean(cm), np.nanpercentile(cm, 99), 100.0 * (1 - nq / ntick)))
    print("=" * 70)
    if a.save:
        # NB : ajout 2026-08-06 de cles de REPORTING pour s3_batch.py. Bloc post-boucle,
        # apres la fin de la simulation -> strictement inerte vis-a-vis de la dynamique
        # et du controleur (aucune ligne de control()/update() touchee). La config gelee
        # du succes 3/3 (QS #5c + w_foot_stance=1400) reste intacte.
        np.savez(a.save, clear=cl, grf=gr, ctrl_ms=cm, dist=dist, climb=climb,
                 n_climbed=n_climbed, max_climb=c._maxclimb, success=success,
                 fell_at=fell_at or -1.0, hriser=a.hriser,
                 upright=upright, reached_top=reached_top, state=c.state,
                 mi=c.mi, n_moves=len(c.moves) - 1, n_risers=len(risers),
                 qp_fail=nq, n_ticks=ntick)
        print("[ok] %s" % a.save)


if __name__ == "__main__":
    main()
