"""
TALOS — S3 : MONTEE D'ESCALIER (3 marches) par approche LIPM PAR PLATEAUX.

Voir s3_design.md. Le marcheur S2 evenementiel (talos_dcm_walk_timing.DCMWalkT)
suppose un sol plat : hauteur de pied `self.fz` scalaire et hauteur de CoM
`self.zc` figee (omega=sqrt(g/zc) calcule une seule fois). Ce fichier leve
l'hypothese plane SANS toucher au noyau (talos_wbc, talos_dcm_walk,
talos_dcm_walk_timing INCHANGES) via une sous-classe :

  1. `self.fz` devient une PROPRIETE renvoyant la hauteur de la marche CIBLE du
     pied de balancement (stair_height(x_cible)). Toute la logique de swing, de
     garde au sol et de detection de poser de DCMWalkT.update() l'utilise telle
     quelle -> cloche de swing et poser corrects sur chaque marche, sans
     reimplementer update().
  2. `self.zc` (reference verticale de CoM, lue dans DCMWalk.control() ligne 219)
     est ELEVEE en rampe vers la hauteur de la marche D'APPUI -> le CoM gagne
     reellement de l'altitude au lieu de s'accroupir. omega reste fixe (plateau
     quasi-statique, cf. s3_design.md par.3 ; omega_k = raffinement future work).
  3. Metrique NOUVELLE `foot clearance` : gap minimal semelle<->surface sous le
     pied pendant le vol (negatif = collision avec le nez de marche).

Le plan de pas DCM reste 2D (xy) : l'escalier n'ajoute que la composante z.

Profil d'escalier = doit rester synchronise avec scene_stairs.xml.

Usage (depuis pal_talos/, conda base avec mujoco) :
    python talos_s3_stairs.py                    # bring-up deterministe
    python talos_s3_stairs.py --viewer           # viewer MuJoCo (relance si chute)
    python talos_s3_stairs.py --seed 0 --save s3_run.npz
    python talos_s3_stairs.py --hriser 0.05      # balayage de conception
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B
import talos_dcm_walk_timing as T

# --- profil d'escalier : (front_x, top_z) ---
# Le profil de REFERENCE (make_stair_height) et la SCENE physique
# (build_stairs_xml) sont derives des MEMES parametres (x0, tread, h_riser, n),
# ce qui garantit leur coherence quel que soit le balayage.
EPS = 1e-6              # tolerance de bord (evite les artefacts flottants aux nez de marche)
PLATFORM = 0.40        # profondeur du palier d'arrivee (m)


def make_risers(x0=0.30, tread=0.28, h=0.10, n=3):
    return [(x0 + i * tread, (i + 1) * h) for i in range(n)]


def top_x_of(x0=0.30, tread=0.28, n=3):
    return x0 + n * tread + PLATFORM


def make_stair_height(risers, top_x):
    def stair_height(x):
        h = 0.0
        for xe, z in risers:
            if x >= xe - EPS:
                h = z
        # au-dela du palier d'arrivee, on retombe au sol (securite : le robot ne
        # doit pas depasser top_x, sinon la marche disparait sous le pied)
        return h if x <= top_x + EPS else 0.0
    return stair_height


def build_stairs_xml(x0=0.30, tread=0.28, h=0.10, n=3):
    """Genere la scene MuJoCo (n marches BOX) a partir des memes parametres que
    le profil de reference -> physique et reference toujours synchrones."""
    end_x = top_x_of(x0, tread, n)
    steps = []
    for i in range(n):
        front = x0 + i * tread
        top = (i + 1) * h
        cx = 0.5 * (front + end_x); hx = 0.5 * (end_x - front)
        cz = 0.5 * top;            hz = 0.5 * top
        steps.append('    <geom name="step%d" type="box" pos="%.4f 0 %.4f" '
                     'size="%.4f 1.0 %.4f" material="stair"/>'
                     % (i + 1, cx, cz, hx, hz))
    return """<mujoco model="talos stairs scene (S3)">
  <include file="talos_motor.xml"/>
  <statistic center="0.6 0 0.9" extent="2.2"/>
  <visual>
    <rgba haze="0.15 0.25 0.35 1"/>
    <global azimuth="160" elevation="-10"/>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3" specular="0 0 0"/>
  </visual>
  <asset>
    <texture type="skybox" builtin="gradient" rgb1="0.3 0.5 0.7" rgb2="0 0 0" width="512" height="3072"/>
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3"
      markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    <material name="stair" rgba="0.55 0.45 0.35 1" reflectance="0.05"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <light name="spotlight" mode="targetbody" target="base_link" pos="1 0 20"/>
%s
  </worldbody>
</mujoco>
""" % "\n".join(steps)


class DCMWalkS3(T.DCMWalkT):
    """Montee d'escalier par plateaux au-dessus du marcheur evenementiel S2."""
    _stairs = None          # defaut classe : desactive (self.fz == sol plat) tant
                            # que la config escalier n'est pas posee en fin d'__init__

    def __init__(self, m, d, stair_height, risers, tread, max_mount=0.32,
                 zc_rate=0.6, **kw):
        super().__init__(m, d, **kw)        # DCMWalk.__init__ fait self.fz = p0[...]
                                            #  -> passe par le setter -> _fz0
        self._stairs = None                 # plat pendant la reconstruction du plan
        self.zc0 = self.zc                  # hauteur de CoM au sol (reference)
        # offset cheville->semelle (pour la metrique de clearance)
        self._ankle_off = float(d.xpos[self.left][2]) - self._fz0
        self.ZC_RATE = zc_rate              # vitesse de rampe verticale du CoM (m/s)
        # garde au sol ADAPTATIVE : plat = valeur S2 gelee (0.04, ne PAS casser le
        # cycle limite valide) ; franchissement de contremarche = garde majoree.
        # NB : self.fz = hauteur de la marche CIBLE releve deja toute la cloche au
        # niveau superieur -> STEP_H n'a qu'a couvrir le retard PD au-dessus du nez,
        # PAS la hauteur de contremarche (erreur du 1er jet : 0.14 global -> chute plat).
        self.FLAT_CLEAR = 0.04
        self.CROSS_CLEAR = 0.10             # surchargeable via --steph
        self.COM_RATE = 0.9                 # vitesse de co-pilotage CoM en montee (m/s)
        self.T_SWING_ARC = 0.45             # duree de l'arc up-over-down en appui simple (s)
        self._climb_k = -1; self._ss_t0 = None
        self._risers = risers; self._tread = tread; self._max_mount = max_mount
        self._k_prev = -1; self._pin_k = -1
        self._fz_takeoff = self._fz0
        self.log["clearance"] = []
        self._sw_clear = None; self._sw_active = False
        self._max_climb = 0.0               # plus haute marche d'appui atteinte
        # STAIR-MODE : reconstruire le plan de pas (approche petits-pas + montee
        # step-together = 1 pied monte sur le giron, l'autre le rejoint). Les
        # appuis d'escalier sont FIGES par la geometrie (pas de replanification de
        # derive comme sur le plat) -> voir re-pin dans update().
        self._build_stair_plan()
        self._stairs = stair_height         # ACTIVE la logique escalier

    def _rebuild_xi(self):
        """Recursion arriere DCM : xi_ini a l'entree de chaque appui (identique a
        DCMWalk.__init__, recalculee sur le plan d'escalier courant)."""
        E = np.exp(self.omega * B.T_STEP)
        xi_ini = [None] * self.N
        xi_end = self.zmp[-1].copy()
        for k in range(self.N - 1, -1, -1):
            xi_ini[k] = self.zmp[k] + (xi_end - self.zmp[k]) / E
            xi_end = xi_ini[k]
        self.xi_ini = xi_ini

    def _build_stair_plan(self):
        """Plan de pas stair-mode : approche plate puis montee step-together.
        Remplace le plan uniforme 0.08 m de la classe de base."""
        d = self.d; ly = self.ly
        rx = float(d.xpos[self.right][0])
        risers = self._risers; tread = self._tread
        yof = lambda side: ly if side == self.left else -ly
        zmp = []; support = []
        side = self.right                              # appui initial = pied droit
        zmp.append(np.array([rx, yof(side)])); support.append(side)
        x_cur = rx
        ctr0 = risers[0][0] + 0.5 * tread              # centre du giron 1 (cible cheville)
        # anti-collision : la POINTE de la semelle (cheville + TOE_EXT) du dernier pied
        # d'approche ne doit PAS depasser le nez de la 1ere contremarche, sinon le pied
        # se pose dans la marche (bug « va trop loin »). On bride la cheville en consequence.
        TOE_EXT = 0.075; MARGIN = 0.03
        x_app_max = risers[0][0] - TOE_EXT - MARGIN     # cheville max d'un pied d'approche
        # APPROCHE : petits pas plats jusqu'a etre a portee de montee du giron 1
        guard = 0
        while (ctr0 - x_cur) > self._max_mount and guard < 80:
            side = self.left if side == self.right else self.right
            x_cur = min(x_cur + B.STEP_LEN, x_app_max)
            zmp.append(np.array([x_cur, yof(side)])); support.append(side)
            guard += 1
            if x_cur >= x_app_max - 1e-9:              # bord anti-collision atteint -> monter
                break
        self._n_approach = len(zmp)
        # MONTEE step-together : mount (monte sur le giron) puis join (l'autre pied rejoint)
        for t in range(len(risers)):
            ctr = risers[t][0] + 0.5 * tread
            side = self.left if side == self.right else self.right   # mount
            zmp.append(np.array([ctr, yof(side)])); support.append(side)
            side = self.left if side == self.right else self.right   # join
            zmp.append(np.array([ctr, yof(side)])); support.append(side)
        self.zmp = zmp; self.support = support; self.N = len(zmp)
        self._plan0 = [p.copy() for p in zmp]          # plan fige (re-pin)
        self._rebuild_xi()
        last = self.zmp[-1]
        close_y = ly if self.support[-1] == self.right else -ly
        self.closing = np.array([last[0], close_y])
        self.end_target = 0.5 * (last + self.closing)
        # journaliser le plan (footholds vises)
        sh = self._stairs_probe
        print("[plan] %d footholds : approche %d petits-pas + montee %d (step-together)"
              % (self.N, self._n_approach, self.N - self._n_approach))
        for i, (p, s) in enumerate(zip(self.zmp, self.support)):
            tag = "app" if i < self._n_approach else "CLIMB"
            print("   z%02d %s x=%.3f y=%+.3f z=%.2f  [%s]"
                  % (i, "G" if s == self.left else "D", p[0], p[1], sh(float(p[0])), tag))

    def _stairs_probe(self, x):
        """Hauteur d'escalier au point x (independant de l'activation self._stairs)."""
        h = 0.0
        for xe, z in self._risers:
            if x >= xe - EPS:
                h = z
        return h

    # ---- self.fz : hauteur de la marche CIBLE du pied de balancement ----
    @property
    def fz(self):
        if getattr(self, "_stairs", None) is None:
            return self._fz0                # sol plat (avant activation / fallback)
        k = self.k
        gx = self.zmp[k + 1][0] if (k + 1 < self.N) else self.closing[0]
        return self._fz0 + self._stairs(float(gx))

    @fz.setter
    def fz(self, v):
        self._fz0 = float(v)                # hauteur de reference du sol plat

    # ---- reference verticale de CoM : suit la marche d'APPUI en rampe ----
    def update(self, dt):
        # garde au sol adaptative : fixee AVANT que le gait de base ne lise B.STEP_H.
        # crossing = le pied de balancement change de niveau (fz_land > fz_takeoff).
        # NB self.swing n'existe qu'apres le 1er super().update() -> garde hasattr
        # (avant le 1er pas, pas de swing : STEP_H=0.04 par defaut, correct).
        sw = getattr(self, "swing", None)
        if getattr(self, "_stairs", None) is not None and sw is not None:
            k = self.k
            if k != self._k_prev:           # nouveau pas : memoriser la hauteur de decollage
                self._fz_takeoff = float(self.d.xpos[sw][2]) - self._ankle_off
                self._k_prev = k
            gx = self.zmp[k + 1][0] if (k + 1 < self.N) else self.closing[0]
            fz_land = self._fz0 + self._stairs(float(gx))
            crossing = (fz_land - self._fz_takeoff) > 1e-4
            B.STEP_H = self.CROSS_CLEAR if crossing else self.FLAT_CLEAR

        super().update(dt)                  # gait S2 evenementiel inchange
        if getattr(self, "_stairs", None) is None:
            return
        # RE-PIN : la base translate les appuis futurs de l'erreur de poser (derive
        # plane). En escalier les appuis sont FIGES par la geometrie -> on restaure
        # le plan fige pour les pas a venir (l'appui courant garde le poser reel).
        if self.ended is None and self.k != self._pin_k:
            for j in range(self.k + 1, self.N):
                self.zmp[j][:] = self._plan0[j]
            self._rebuild_xi()
            self._pin_k = self.k
        sx = float(self.d.xpos[self.stance][0])
        step_h = self._stairs(sx)
        self._max_climb = max(self._max_climb, step_h)
        z_tgt = self.zc0 + step_h
        self.zc += float(np.clip(z_tgt - self.zc, -self.ZC_RATE * dt, self.ZC_RATE * dt))
        # STAIR-MODE (montee) : co-piloter le CoM en QUASI-STATIQUE. Le DCM plan seul
        # laisse le CoM traîner (~270 mm) derriere les grands footholds -> la jambe ne
        # peut pas atteindre la marche -> percute. On avance donc explicitement la
        # reference CoM (xy) sur l'appui, en la menant vers le milieu appui->prochain
        # foothold avec la phase du pas. Remplace la reference DCM sur les pas de montee.
        self._climb_com_codrive(dt)
        # STAIR-MODE : surcharger la trajectoire du pied sur les pas de MONTEE
        # (remplace la cloche plate + le mode recovery lateral-pur, inadaptes).
        self._climb_swing_override()
        self._clearance_tick()

    def _climb_com_codrive(self, dt):
        """Reference CoM quasi-statique pendant la montee : suit l'appui et avance
        vers le milieu (appui, prochain foothold) selon la phase -> le corps monte
        AVEC les pieds au lieu de traîner. Ralentir (--tstep) = plus quasi-statique."""
        if self.ended is not None:
            return
        k = self.k
        # stair-mode actif des que la CIBLE du pas (zmp[k+1]) est un foothold
        # d'escalier -> k+1 >= _n_approach (sinon le 1er mount reste au gait plat
        # = overshoot + percute la marche : bug d'indice observe « va trop loin »).
        if (k + 1) < self._n_approach or self.t0 is None:
            return
        # QUASI-STATIQUE STRICT : la reference CoM SUIT LE PIED D'APPUI (x ET y).
        # Le trace #7 a montre que viser le MILIEU appui->prochain foothold poussait
        # le CoM DEVANT le seul pied au sol pendant l'appui simple (CoM 0.44 vs appui
        # 0.19) -> bascule avant (pitch +65, DCM_x->1.8, chute). En restant au-dessus
        # de l'appui, le CoM n'avance QUE lorsque ce pied (plus avance) devient l'appui
        # a l'etape suivante -> montee en « CoM suit l'appui », sans se jeter en avant.
        st = np.asarray(self.d.xpos[self.stance][:2], dtype=float)
        tgt = np.array([float(st[0]), float(st[1])])
        rate = self.COM_RATE * dt
        self.com_ref_xy = self.com_ref_xy + np.clip(tgt - self.com_ref_xy, -rate, rate)
        self.xi_ref = self.com_ref_xy.copy()

    def _climb_swing_override(self):
        """Trajectoire de swing dediee au franchissement : leve d'abord (pic a
        s~0.45, sans saut au depart), avance ensuite (retardee), descend au CENTRE
        du giron fige. Phase indexee sur T_nom (PAS sur le timing adaptatif qui
        ecourtait le pas -> pied court -> percute la contremarche). Bypasse aussi
        le mode recovery de la base (lateral-pur = fatal en escalier)."""
        if self.ended is not None:
            return
        k = self.k
        # actif des que la CIBLE (zmp[k+1]) est un foothold d'escalier ; le 1er mount
        # est le pas k=_n_approach-1 (bug d'indice corrige : « 3e pas va trop loin »).
        if (k + 1) < self._n_approach or (k + 1) >= self.N:
            return
        # NB : le GATING (maintien du pied au sol en DS) a ete RETIRE (stair-mode #9) :
        # le timing adaptatif du gait plat raccourcit le pas a T_MIN quand le DCM derive,
        # terminant le pas PENDANT le maintien -> mount jamais execute (0/3). Incompatibilite
        # architecturale documentee (Ch5 §5.6 / Ch6). Config conservee = « CoM suit l'appui ».
        if self.t0 is None or getattr(self, "phase", None) == "DS":
            return                                       # double appui : gere par la base
        frm = self.swing_from.get(self.swing) if hasattr(self.swing_from, "get") else None
        if frm is None:
            return
        land_xy = self._plan0[k + 1]
        land_z = self._fz0 + self._stairs(float(land_xy[0]))
        takeoff_z = float(frm[2])
        peak = max(takeoff_z, land_z) + self.CROSS_CLEAR
        s = min(max((self.t - self.t0) / self.T_nom, 0.0), 1.0)
        # profil UP-OVER-DOWN : monter VERTICAL au-dessus du decollage (phase 1),
        # avancer A HAUTEUR DE PIC (phase 2, la semelle est deja au-dessus du nez),
        # descendre sur le giron cible (phase 3). Le mouvement horizontal ne se fait
        # QU'A hauteur de pic -> la semelle ne frotte plus la contremarche (bug frottement).
        # NB self.swing_pos est la cheville ; la semelle est ~_ankle plus bas, la garde
        # CROSS_CLEAR (peak) couvre cet offset + le retard PD.
        LIFT, FWD = 0.30, 0.72
        if s < LIFT:                                     # 1) monter vertical
            xfrac = 0.0
            z = takeoff_z + (peak - takeoff_z) * np.sin(np.pi / 2 * (s / LIFT))
        elif s < FWD:                                    # 2) avancer a hauteur de pic
            u = (s - LIFT) / (FWD - LIFT)
            xfrac = u * u * (3.0 - 2.0 * u)              # smootherstep horizontal
            z = peak
        else:                                            # 3) descendre sur le giron
            xfrac = 1.0
            u = (s - FWD) / (1.0 - FWD)
            z = land_z + (peak - land_z) * np.cos(np.pi / 2 * u)
        x = float(frm[0]) + (float(land_xy[0]) - float(frm[0])) * xfrac
        y = float(frm[1]) + (float(land_xy[1]) - float(frm[1])) * xfrac
        self.swing_pos = np.array([x, y, z])

    def _clearance_tick(self):
        """Gap minimal semelle<->surface directement sous le pied pendant le vol."""
        d = self.d
        if getattr(self, "phase", None) == "SS":
            sw = self.swing
            sole_z = float(d.xpos[sw][2]) - self._ankle_off
            surf = self._fz0 + self._stairs(float(d.xpos[sw][0]))
            gap = sole_z - surf
            self._sw_clear = gap if self._sw_clear is None else min(self._sw_clear, gap)
            self._sw_active = True
        elif self._sw_active and self._sw_clear is not None:
            self.log["clearance"].append(self._sw_clear)
            self._sw_clear = None; self._sw_active = False


def _base_pitch_deg(d):
    """Tangage du bassin (deg) : + = penche vers l'AVANT (se jette en avant)."""
    import math
    w, x, y, z = float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6])
    return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))


def trace_row(m, d, c):
    """Instantane detaille de l'etat (1 ligne de trace)."""
    com = d.subtree_com[c.base]
    try:
        xi = c._measured_dcm()[2]
    except Exception:
        xi = (0.0, 0.0)
    sw = getattr(c, "swing", None); st = getattr(c, "stance", None)
    swp = getattr(c, "swing_pos", None)
    s = 0.0
    if getattr(c, "t0", None) is not None and getattr(c, "Tk", 0):
        s = min(max((c.t - c.t0) / c.Tk, 0.0), 1.0)
    return dict(
        t=round(c.t, 3), k=c.k, phase=getattr(c, "phase", "?"), s=round(s, 3),
        Tk=round(float(getattr(c, "Tk", 0.0)), 3),
        com_x=round(float(com[0]), 4), com_z=round(float(com[2]), 4),
        ref_x=round(float(c.com_ref_xy[0]), 4), ref_y=round(float(c.com_ref_xy[1]), 4),
        zc=round(float(c.zc), 4),
        dcm_x=round(float(xi[0]), 4), dcm_y=round(float(xi[1]), 4),
        base_vx=round(float(d.qvel[0]), 3), base_vz=round(float(d.qvel[2]), 3),
        pitch=round(_base_pitch_deg(d), 2), base_z=round(float(d.qpos[2]), 4),
        sw=("G" if sw == c.left else "D") if sw is not None else "-",
        sw_x=round(float(d.xpos[sw][0]), 4) if sw is not None else 0.0,
        sw_z=round(float(d.xpos[sw][2]), 4) if sw is not None else 0.0,
        cmd_x=round(float(swp[0]), 4) if swp is not None else 0.0,
        cmd_z=round(float(swp[2]), 4) if swp is not None else 0.0,
        st_x=round(float(d.xpos[st][0]), 4) if st is not None else 0.0,
        st_z=round(float(d.xpos[st][2]), 4) if st is not None else 0.0,
        grfL=round(c._grf_z(c.left), 0), grfR=round(c._grf_z(c.right), 0),
        rec=int(bool(getattr(c, "_recover", False))),
    )


def _trace_print(r):
    print("[trc] t=%.2f k=%d %-2s s=%.2f Tk=%.2f | CoM x=%.3f z=%.3f  ref x=%.3f zc=%.3f | "
          "DCM %.3f,%.3f | vx=%+.2f vz=%+.2f pitch=%+.1f bz=%.3f | sw %s realx=%.3f realz=%.3f "
          "cmdx=%.3f cmdz=%.3f | stx=%.3f | GRF L=%.0f R=%.0f%s"
          % (r["t"], r["k"], r["phase"], r["s"], r["Tk"], r["com_x"], r["com_z"],
             r["ref_x"], r["zc"], r["dcm_x"], r["dcm_y"], r["base_vx"], r["base_vz"],
             r["pitch"], r["base_z"], r["sw"], r["sw_x"], r["sw_z"], r["cmd_x"], r["cmd_z"],
             r["st_x"], r["grfL"], r["grfR"], "  REC" if r["rec"] else ""))


def main():
    ap = argparse.ArgumentParser()
    # --- geometrie escalier (doit rester synchro avec scene_stairs.xml) ---
    ap.add_argument("--x0", type=float, default=0.30, help="x du premier nez de marche (m)")
    ap.add_argument("--tread", type=float, default=0.28, help="giron (m)")
    ap.add_argument("--hriser", type=float, default=0.10, help="contremarche (m) — balayage {0.05,0.10,0.15}")
    ap.add_argument("--zcrate", type=float, default=0.6, help="vitesse de rampe verticale du CoM (m/s)")
    ap.add_argument("--maxmount", type=float, default=0.28,
                    help="pas de montee max autorise (m) : fin de l'approche quand le centre du giron 1 est a portee")
    ap.add_argument("--sagmax", type=float, default=0.50,
                    help="capture sagittale SAG_MAX (m) : elargie vs S2 (0.13) pour autoriser les grands pas de montee")
    ap.add_argument("--comrate", type=float, default=0.6,
                    help="vitesse de co-pilotage quasi-statique du CoM en montee (m/s) ; monter si le CoM traîne")
    # --- gait : defauts = config S2 gelee (the project log) ; tstep majore pour grands pas ---
    ap.add_argument("--steps", type=int, default=20)
    ap.add_argument("--steplen", type=float, default=0.08)
    ap.add_argument("--tstep", type=float, default=0.8)
    ap.add_argument("--tmin", type=float, default=0.24)
    ap.add_argument("--tmax", type=float, default=None)
    ap.add_argument("--dsovl", type=float, default=0.12,
                    help="fraction de double-appui (0.12 = valeur S2 plat gelee)")
    ap.add_argument("--offlat", type=float, default=0.08)
    ap.add_argument("--minsep", type=float, default=0.14)
    ap.add_argument("--kdcm", type=float, default=1.0)
    ap.add_argument("--kfoot", type=float, default=1.0)
    ap.add_argument("--offmax", type=float, default=0.20)
    ap.add_argument("--zcdrop", type=float, default=0.04)
    ap.add_argument("--wfoot", type=float, default=6000.0)
    ap.add_argument("--wfootst", type=float, default=500.0)
    ap.add_argument("--steph", type=float, default=0.14,
                    help="garde au sol du pied de balancement sur le pas de FRANCHISSEMENT (m) ; "
                         "le plat garde la valeur S2 gelee 0.04 (adaptatif par pas). "
                         "NB self.fz releve deja la cloche au niveau superieur : steph ne couvre que le retard PD au-dessus du nez")
    ap.add_argument("--no-timing", dest="timing", action="store_false")
    ap.add_argument("--no-feedback", dest="feedback", action="store_false")
    ap.add_argument("--no-footfb", dest="footfb", action="store_false")
    ap.add_argument("--no-cpswing", dest="cpswing", action="store_false")
    ap.add_argument("--no-footori", dest="footori", action="store_false")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--trace", action="store_true",
                    help="LOG DETAILLE par tick -> s3_trace.csv + trace console compacte pendant la montee")
    ap.add_argument("--tracefile", type=str, default="s3_trace.csv")
    ap.add_argument("--traceevery", type=int, default=40,
                    help="periode (ticks) d'impression console de la trace pendant la montee")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()
    if a.tmax is None: a.tmax = 1.6 * a.tstep

    risers = make_risers(a.x0, a.tread, a.hriser)
    top_x = top_x_of(a.x0, a.tread, len(risers))
    stair_height = make_stair_height(risers, top_x)
    top_target = risers[-1][1]              # hauteur du sommet (= n*h_riser)
    # generer la scene physique depuis les MEMES parametres (coherence garantie)
    with open("scene_stairs.xml", "w") as f:
        f.write(build_stairs_xml(a.x0, a.tread, a.hriser))
    print("[scene] scene_stairs.xml genere : x0=%.2f tread=%.2f h=%.2f n=%d (top_x=%.2f)"
          % (a.x0, a.tread, a.hriser, len(risers), top_x))

    # parametres de gait via les globals du module de base (convention etablie)
    B.N_STEPS = a.steps; B.STEP_LEN = a.steplen; B.T_STEP = a.tstep; B.DS_OVL = a.dsovl
    B.STEP_H = 0.04                          # garde plat = valeur S2 gelee ; adaptee par pas dans DCMWalkS3.update()
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0
    T.ZLAND_K, T.ZLAND_MIN = 15.0, 0.18
    T.SAG_MAX = a.sagmax                     # capture sagittale elargie (grands pas de montee)

    W.MODEL = "scene_stairs.xml"             # <-- escalier au lieu du sol plat
    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv); mujoco.mj_forward(m, d)
    com0 = d.subtree_com[m.body("base_link").id].copy()
    x0m, z0m = float(com0[0]), float(com0[2])

    def build(d_):
        T.apply_squat(m, d_, a.zcdrop)
        c_ = DCMWalkS3(m, d_, stair_height, risers, a.tread, max_mount=a.maxmount,
                       zc_rate=a.zcrate, t_nom=a.tstep, t_min=a.tmin, t_max=a.tmax)
        c_.use_timing = a.timing
        c_.use_dcm_fb = a.feedback; c_.k_dcm = a.kdcm
        c_.use_foot_fb = a.footfb; c_.k_foot = a.kfoot
        if a.footfb: c_.OFF_MAX = np.array([a.offmax, a.offmax])
        c_.use_cp_swing = a.cpswing
        c_.use_foot_ori = a.footori
        c_.w_foot_swing = a.wfoot; c_.w_foot_stance = a.wfootst
        c_.off_lat = a.offlat; c_.min_sep = a.minsep
        c_.FLAT_CLEAR = 0.04; c_.CROSS_CLEAR = a.steph
        c_.COM_RATE = a.comrate
        c_.debug = a.debug
        return c_

    c = build(d)
    dt = m.opt.timestep

    if a.viewer:
        from mujoco import viewer as mjv
        with mjv.launch_passive(m, d) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            start = time.time()
            while viewer.is_running():
                c.update(dt); c.control(); mujoco.mj_step(m, d)
                if a.trace and c.k >= c._n_approach - 1 and int(round(c.t / dt)) % a.traceevery == 0:
                    _trace_print(trace_row(m, d, c))
                viewer.cam.lookat[0] = float(d.xpos[c.base][0])
                viewer.cam.lookat[1] = float(d.xpos[c.base][1])
                viewer.cam.lookat[2] = float(d.xpos[c.base][2])
                viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0: time.sleep(slp)
                if d.qpos[2] < 0.5:
                    print("[viewer] chute t=%.2f s (pas k=%d) — relance" % (c.t, c.k))
                    time.sleep(0.6)
                    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
                    c = build(d); start = time.time()
        return

    t_cap = 6.0 + c.N * a.tmax + T.T_END     # duree basee sur la longueur du plan d'escalier
    render = None
    if a.video:
        import talos_sim; render = talos_sim.render
    fe = max(1, int(1 / (30 * dt))); frames = []
    fell_at = None
    trace_rows = []
    while c.t < t_cap:
        c.update(dt); c.control(); mujoco.mj_step(m, d)
        if a.trace:
            r = trace_row(m, d, c); trace_rows.append(r)
            if c.k >= c._n_approach - 1 and int(round(c.t / dt)) % a.traceevery == 0:
                _trace_print(r)
        if d.qpos[2] < 0.6:
            fell_at = c.t
            if a.trace:
                _trace_print(trace_row(m, d, c)); print("[trc] ===== CHUTE (base_z<0.6) =====")
            break
        if c.ended is not None and (c.t - c.ended) > T.T_END:
            break
        if a.video and int(c.t / dt) % fe == 0:
            frames.append(render(m, d))
    if a.trace and trace_rows:
        import csv
        with open(a.tracefile, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(trace_rows[0].keys()))
            wr.writeheader(); wr.writerows(trace_rows)
        print("[ok] %s (%d lignes)" % (a.tracefile, len(trace_rows)))

    # ---------- bilan S3 ----------
    com = d.subtree_com[c.base]
    dist = float(com[0] - x0m); climb = float(com[2] - z0m)
    upright = d.qpos[2] > 0.8 and fell_at is None
    n_climbed = int(round(c._max_climb / a.hriser)) if a.hriser > 0 else 0
    reached_top = c._max_climb >= top_target - 1e-6
    success = upright and reached_top and c.ended is not None
    ce = np.array(c.log["com_err"]) if c.log["com_err"] else np.array([np.nan])
    cm = np.array(c.log["ctrl_ms"]); ts = np.array(c.log["t_steps"])
    cl = np.array(c.log["clearance"]) if c.log["clearance"] else np.array([np.nan])
    lg = np.array(c.log["land_grf"]) if c.log["land_grf"] else np.array([np.nan])
    nq = int(getattr(c, "_qp_fail", 0)); ntick = max(len(cm), 1)
    print("=" * 68)
    print("S3 STAIRS  steps=%d len=%.2f Tnom=%.2f [%.2f,%.2f] | escalier x0=%.2f tread=%.2f h=%.2f steph=%.2f seed=%s"
          % (a.steps, a.steplen, a.tstep, a.tmin, a.tmax, a.x0, a.tread, a.hriser, a.steph, a.seed))
    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, pas k=%d)" % (fell_at, c.k)))
    print("  montee       : %d/%d marches (max appui %.2f m) | gain CoM z %.3f m -> %s"
          % (n_climbed, len(risers), c._max_climb, climb, "SUCCESS" if success else "FAIL"))
    print("  avancee x    : %.2f m | pas poses=%d / %d" % (dist, len(ts), a.steps - 1))
    print("  CLEARANCE    : min %.3f m | mean %.3f m | franchissements=%d%s"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clearance"]),
             "  ⚠ COLLISION" if (len(c.log["clearance"]) and np.nanmin(cl) < 0) else ""))
    print("  CoM err (xy) : RMSE %.1f mm | max %.1f mm"
          % (np.sqrt(np.nanmean(ce**2)) * 1e3, np.nanmax(ce) * 1e3))
    if len(c.log["land_grf"]):
        print("  poser GRF    : pic %.0f N | mean %.0f N" % (np.nanmax(lg), np.nanmean(lg)))
    if len(cm):
        print("  ctrl loop    : mean %.2f ms | p99 %.2f ms | QP feas %.1f%%"
              % (cm.mean(), np.percentile(cm, 99), 100.0 * (1 - nq / ntick)))
    print("=" * 68)
    if a.save:
        np.savez(a.save, com_err=ce, ctrl_ms=cm, t_steps=ts, clearance=cl, land_grf=lg,
                 dist=dist, climb=climb, n_climbed=n_climbed, max_climb=c._max_climb,
                 success=success, fell_at=fell_at or -1.0, hriser=a.hriser,
                 tread=a.tread, x0=a.x0, steph=a.steph, seed=a.seed if a.seed is not None else -1)
        print("[ok] %s" % a.save)
    if a.video and frames:
        import imageio.v2 as imageio
        imageio.mimsave("talos_s3_stairs.mp4", frames, fps=30)
        print("[ok] talos_s3_stairs.mp4")


if __name__ == "__main__":
    main()
