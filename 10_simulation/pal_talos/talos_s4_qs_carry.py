r"""
TALOS — S4 : LOCO-MANIPULATION quasi-statique (marche plane + port d'une VRAIE boite).

Test de l'hypothese H1 (the thesis) : coordination STABLE et SIMULTANEE
locomotion + manipulation. Voir s4_design.md.

CHOIX D'ARCHITECTURE (decision David, 2026-07-10) :
  * Locomotion = SEQUENCEUR QUASI-STATIQUE (reutilise `StairQS` de S3), PAS le
    marcheur dynamique S2 (70 %, marge mince). Motif : porter une vraie boite
    DEPLACE le CoM ; la marche QS garde le CoM dans le polygone d'appui (marge
    STATIQUE) et encaisse ce decalage, la ou le cycle limite dynamique S2
    basculerait. On execute donc un plan de footholds PLAT (au lieu d'escalier).
  * Objet = VRAIE BOITE : corps rigide (freejoint + geom + inertie) SOUDE aux
    DEUX mains par contraintes d'egalite MuJoCo (weld). Pas de prehension par
    friction des doigts (fragile). Le poids de la boite transite par les welds
    -> perturbation reelle sur les bras, rejetee par les taches posture + EE.

MISE EN PLACE DU PORT (build) :
  1. apply_squat (jambes flechies, comme S2/S3) ;
  2. IK amortie sur les 14 DOF de bras -> mains amenees a une pose de PORT
     (devant le torse, symetrique, degagee des jambes) ;
  3. boite placee au milieu des deux mains ;
  4. relpose des welds ecrite au runtime (= box^-1 * main, layout eq_data
     verifie empiriquement : [anchor(3), relpose_pos(3), relpose_quat(4),
     torquescale(1)]) -> contrainte satisfaite a residu machine ;
  5. controleur construit APRES -> sa posture `home` = la pose de port, et les
     references EE (relatives-base) sont figees sur cette pose.

DISCIPLINE : talos_wbc.py, talos_dcm_walk.py, talos_dcm_walk_timing.py,
talos_s3_stairs_qs*.py restent INCHANGES. `control()` est une COPIE FIDELE de
DCMWalk.control() + les 2 taches EE (la base n'offre pas de hook). Re-synchroniser
si le control() de base evolue.

Usage (depuis pal_talos/, conda base) :
    python talos_s4_qs_carry.py --payload 2.0 --save s4qs.npz
    python talos_s4_qs_carry.py --payload 2.0 --viewer
    python talos_s4_qs_carry.py --payload 0   --trace       # A/B sans charge
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B
import talos_dcm_walk_timing as T          # apply_squat
import talos_s3_stairs_qs as QS            # StairQS

HAND = {"L": "arm_left_7_link", "R": "arm_right_7_link"}
W_EE_DEF, KP_EE_DEF, KD_EE_DEF = 50.0, 400.0, 40.0


# ============================ scene generation ============================
def build_carry_xml(box_hx, box_hy, box_hz, box_mass):
    box_mass = max(float(box_mass), 1e-3)                 # plancher (A/B payload 0)
    inertia = max(box_mass * (box_hx**2 + box_hy**2) / 3.0, 1e-4)   # cuboid ; plancher > mjMINVAL
    return f"""<mujoco model="talos carry scene (S4)">
  <include file="talos_motor.xml"/>
  <statistic center="0 0 .9" extent="1.6"/>
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
    <material name="boxmat" rgba="0.75 0.55 0.25 1"/>
  </asset>
  <worldbody>
    <geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>
    <light name="spotlight" mode="targetbody" target="base_link" pos="1 0 20"/>
    <body name="carry_box" pos="0.35 0 0.90">
      <freejoint name="box_free"/>
      <inertial pos="0 0 0" mass="{box_mass:.3f}" diaginertia="{inertia:.4f} {inertia:.4f} {inertia:.4f}"/>
      <geom name="box_geom" type="box" size="{box_hx:.3f} {box_hy:.3f} {box_hz:.3f}" material="boxmat"/>
    </body>
  </worldbody>
  <equality>
    <weld name="weld_L" body1="carry_box" body2="arm_left_7_link" solref="0.02 1"/>
    <weld name="weld_R" body1="carry_box" body2="arm_right_7_link" solref="0.02 1"/>
  </equality>
</mujoco>
"""


# ============================ carry set-up helpers ============================
def carry_ik(m, d, tgtL_base, tgtR_base, iters=200, tol=2e-3):
    """IK amortie (14 DOF de bras) : amene main G a tgtL, main D a tgtR
    (cibles exprimees dans le repere base). Ne touche QUE les DOF de bras."""
    base = m.body("base_link").id
    hL = m.body(HAND["L"]).id; hR = m.body(HAND["R"]).id
    armj = [m.joint("arm_%s_%d_joint" % (s, i)) for s in ("left", "right") for i in range(1, 8)]
    qadr = [m.jnt_qposadr[j.id] for j in armj]; dadr = [m.jnt_dofadr[j.id] for j in armj]
    it = 0
    for it in range(iters):
        mujoco.mj_forward(m, d)
        pb = d.xpos[base]; Rb = d.xmat[base].reshape(3, 3)
        tL = pb + Rb @ tgtL_base; tR = pb + Rb @ tgtR_base
        eL = tL - d.xpos[hL]; eR = tR - d.xpos[hR]
        if max(np.linalg.norm(eL), np.linalg.norm(eR)) < tol:
            break
        JL = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, JL, None, d.xpos[hL], hL)
        JR = np.zeros((3, m.nv)); mujoco.mj_jac(m, d, JR, None, d.xpos[hR], hR)
        J = np.vstack([JL[:, dadr], JR[:, dadr]]); e = np.concatenate([eL, eR])
        dq = J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e)
        for a_, dv in zip(qadr, dq):
            d.qpos[a_] += 0.5 * float(dv)
    mujoco.mj_forward(m, d)
    return it, float(np.linalg.norm(tL - d.xpos[hL])), float(np.linalg.norm(tR - d.xpos[hR]))


def place_box_and_weld(m, d):
    """Place la boite au milieu des mains puis fixe le relpose des 2 welds
    (= box^-1 * main), layout eq_data verifie : [anchor3, pos3, quat4, torque1]."""
    box = m.body("carry_box").id
    hL = m.body(HAND["L"]).id; hR = m.body(HAND["R"]).id
    bq = m.jnt_qposadr[m.joint("box_free").id]
    ctr = 0.5 * (d.xpos[hL] + d.xpos[hR])
    d.qpos[bq:bq+3] = ctr; d.qpos[bq+3:bq+7] = [1, 0, 0, 0]
    mujoco.mj_forward(m, d)
    for name, hand in (("weld_L", hL), ("weld_R", hR)):
        eqid = m.equality(name).id
        pb = d.xpos[box]; Rb = d.xmat[box].reshape(3, 3)
        ph = d.xpos[hand]; Rh = d.xmat[hand].reshape(3, 3)
        p = Rb.T @ (ph - pb); R = Rb.T @ Rh; q = np.zeros(4); mujoco.mju_mat2Quat(q, R.ravel())
        m.eq_data[eqid, 0:3] = 0.0; m.eq_data[eqid, 3:6] = p
        m.eq_data[eqid, 6:10] = q; m.eq_data[eqid, 10] = 1.0
    mujoco.mj_forward(m, d)


# ============================ controller ============================
class S4CarryQS(QS.StairQS):
    """Sequenceur QS sur plan PLAT + tache d'organe terminal bi-manuelle (port)."""

    def __init__(self, m, d, dist, stride, w_ee, kp_ee, kd_ee, **kw):
        self._dist = dist; self._stride = stride            # lus par _build_moves (override)
        super().__init__(m, d, risers=[(1e9, 0.0)], tread=0.28, **kw)  # risers ignores (plan plat)
        # refs EE relatives-base FIGEES sur la pose de port (IK deja appliquee)
        self.hand = {s: m.body(HAND[s]).id for s in ("L", "R")}
        Rb = d.xmat[self.base].reshape(3, 3); pb = d.xpos[self.base].copy()
        self.ee_local = {s: Rb.T @ (d.xpos[self.hand[s]].copy() - pb) for s in ("L", "R")}
        self.w_ee, self.kp_ee, self.kd_ee = w_ee, kp_ee, kd_ee
        self.box = m.body("carry_box").id
        self.log["ee_L"] = []; self.log["ee_R"] = []; self.log["ee_regime"] = []; self.log["com_err"] = []

    # --- plan PLAT : marche quasi-statique alternee jusqu'a `dist` ---
    def _sh(self, x):
        return 0.0                                          # sol plat (pas d'escalier)

    def _build_moves(self):
        ly = self.ly; fz0 = float(self.fz)
        yof = lambda side: ly if side == self.left else -ly
        xs = {self.left: float(self.d.xpos[self.left][0]),
              self.right: float(self.d.xpos[self.right][0])}
        x0 = 0.5 * (xs[self.left] + xs[self.right]); target_x = x0 + self._dist
        self._n_approach = 1
        self.moves = []; side = self.left                   # 1er swing = G (appui initial = D)
        k = 0
        while min(xs.values()) < target_x and k < 400:
            newx = min(max(xs.values()) + self._stride, target_x)
            xs[side] = newx
            self.moves.append((side, np.array([newx, yof(side), fz0])))
            side = self.left if side == self.right else self.right
            k += 1
        # pas de fermeture : ramener le pied arriere a cote de l'avant
        lead = max(xs.values())
        self.moves.append((side, np.array([lead, yof(side), fz0])))
        print("[plan-QS-plat] %d moves | dist cible=%.2f m stride=%.3f" %
              (len(self.moves), self._dist, self._stride))

    # --- tache EE (mouvement relatif main<->base), identique a talos_s4_locomanip ---
    def _ee_task(self, s):
        m, d = self.m, self.d; nv = self.nv
        Rb = d.xmat[self.base].reshape(3, 3)
        p_ref = d.xpos[self.base] + Rb @ self.ee_local[s]
        p_main = d.xpos[self.hand[s]].copy()
        Jm = np.zeros((3, nv)); mujoco.mj_jac(m, d, Jm, None, p_main, self.hand[s])
        Jr = np.zeros((3, nv)); mujoco.mj_jac(m, d, Jr, None, p_ref, self.base)
        J_rel = Jm - Jr
        a_ee = self.kp_ee * (p_ref - p_main) - self.kd_ee * (J_rel @ d.qvel)
        return J_rel, a_ee, float(np.linalg.norm(p_main - p_ref))

    def _regime(self):
        if self.state == "SETTLE": return 0
        if self.state == "DONE":   return 2
        return 1

    # ============= control() : COPIE de DCMWalk.control() + taches EE =============
    # /!\ Copie fidele de talos_dcm_walk.py::DCMWalk.control (noyau QP inchange).
    #     Ajouts marques "### S4" : taches EE, log EE. (StairQS.control ne fait
    #     qu'appeler super().control() + logs ; on remplace par la copie + EE.)
    def control(self):
        t_perf = time.perf_counter()
        m, d = self.m, self.d; nv, nu = self.nv, self.nu
        active = list(self.feet) if self.phase == "DS" else [self.stance]
        Jc = self.contact_jac_feet(active); ncp = Jc.shape[0] // 3; nf = 3 * ncp; n = nv + nu + nf
        M = np.zeros((nv, nv)); W.fill_fullM(m, d, M); h = d.qfrc_bias.copy()
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, self.base)
        com = d.subtree_com[self.base]; com_v = Jcom @ d.qvel
        ref = np.array([self.com_ref_xy[0], self.com_ref_xy[1], self.zc])
        a_com = B.KP_COM_W * (ref - com) - B.KD_COM_W * com_v
        Jori = np.zeros((3, nv)); tmp = np.zeros((3, nv)); mujoco.mj_jacBody(m, d, tmp, Jori, self.base)
        q = d.qpos[3:7]; err = np.zeros(3); neg = np.zeros(4); res = np.zeros(4)
        mujoco.mju_negQuat(neg, q); mujoco.mju_mulQuat(res, np.array([1., 0, 0, 0]), neg); mujoco.mju_quat2Vel(err, res, 1.)
        a_ori = W.KP_ORI * err - W.KD_ORI * (Jori @ d.qvel)
        Jpos = np.zeros((nu, nv))
        for kk, dof in enumerate(self.act_dofs): Jpos[kk, dof] = 1.
        qcur = np.array([d.qpos[a] for a in self.act_qadr]); vcur = np.array([d.qvel[a] for a in self.act_dofs])
        a_pos = W.KP_POS * (self.home - qcur) - W.KD_POS * vcur
        G = np.zeros((n, n)); a = np.zeros(n)
        def add(J, rhs, w): G[:nv, :nv] += 2 * w * (J.T @ J); a[:nv] += 2 * w * (J.T @ rhs)
        add(Jcom, a_com, B.W_COM_W); add(Jori, a_ori, W.W_ORI); add(Jpos, a_pos, B.W_POS_W)
        if not self.use_hard_contact:
            add(Jc, -W.BAUMGARTE * (Jc @ d.qvel), W.W_CONTACT)
        if self.phase == "SS":
            fb = self.swing; p = d.xpos[fb]; Jsw = np.zeros((3, nv)); mujoco.mj_jac(m, d, Jsw, None, p, fb)
            a_sw = B.KP_SW * (self.swing_pos - p) - B.KD_SW * (Jsw @ d.qvel); add(Jsw, a_sw, B.W_SWING)
        # ### S4 : taches d'organe terminal (les DEUX mains, toutes phases) ---------
        e_ee = {}
        for s in ("L", "R"):
            J_rel, a_ee, e = self._ee_task(s); add(J_rel, a_ee, self.w_ee); e_ee[s] = e
        # --------------------------------------------------------------------------
        ramp = {fb: 1.0 for fb in active}
        if self.sl_imp or self.sl_ramp:
            for fb in active:
                ramp[fb] = float(np.clip((self.t - self.land_t.get(fb, -1e9)) / max(self.T_RAMP, 1e-6), 0.0, 1.0))
        if self.use_foot_ori:
            if self.w_foot_stance > 0:
                for fb in active:
                    if self.sl_imp and ramp[fb] < 1.0:
                        Jr, a_o = self.foot_ori_task(fb, W.KP_FOOT * (0.5 + 0.5 * ramp[fb]),
                                                     W.KD_FOOT * (1.3 - 0.3 * ramp[fb]))
                        add(Jr, a_o, max(self.w_foot_stance, 300.0))
                    else:
                        Jr, a_o = self.foot_ori_task(fb); add(Jr, a_o, self.w_foot_stance)
            if self.phase == "SS":
                Jr, a_o = self.foot_ori_task(self.swing); add(Jr, a_o, self.w_foot_swing)
        G[:nv, :nv] += 2 * W.EPS_QDD * np.eye(nv); G[nv:nv+nu, nv:nv+nu] += 2 * W.EPS_TAU * np.eye(nu); G[nv+nu:, nv+nu:] += 2 * W.EPS_F * np.eye(nf)
        Aeq = np.zeros((nv, n)); Aeq[:, :nv] = M; Aeq[:, nv:nv+nu] = -self.S; Aeq[:, nv+nu:] = -Jc.T; beq = -h
        if self.use_hard_contact:
            cr, cb = [], []
            for fb in active:
                Jp6 = np.zeros((3, nv)); Jr6 = np.zeros((3, nv)); mujoco.mj_jac(m, d, Jp6, Jr6, d.xpos[fb], fb)
                Js = (Jp6, Jr6) if (not self.sl_ramp or ramp[fb] >= 0.5) else (Jp6,)
                for J in Js:
                    row = np.zeros((3, n)); row[:, :nv] = J; cr.append(row); cb.append(-W.BAUMGARTE * (J @ d.qvel))
                if self.sl_ramp and ramp[fb] < 0.5 and not (self.use_foot_ori and self.w_foot_stance > 0):
                    Jr_, a_ = self.foot_ori_task(fb, W.KP_FOOT * (0.5 + 0.5 * ramp[fb]),
                                                 W.KD_FOOT * (1.3 - 0.3 * ramp[fb]))
                    add(Jr_, a_, 300.0)
            Aeq = np.vstack([Aeq] + cr); beq = np.concatenate([beq] + cb)
        rows, lb = [], []
        ci = 0
        for fb in active:
            fzmin = W.FZ_MIN * (ramp[fb] if self.sl_ramp else 1.0)
            for _cl in self.corners_local[fb]:
                b = nv + nu + 3 * ci
                v = np.zeros(n); v[b+2] = 1.; rows.append(v); lb.append(fzmin)
                for sg in (-1., 1.):
                    v = np.zeros(n); v[b+2] = W.MU; v[b+0] = sg; rows.append(v); lb.append(0.)
                    v = np.zeros(n); v[b+2] = W.MU; v[b+1] = sg; rows.append(v); lb.append(0.)
                ci += 1
        for j in range(nu):
            v = np.zeros(n); v[nv+j] = 1.; rows.append(v); lb.append(self.tau_min[j])
            v = np.zeros(n); v[nv+j] = -1.; rows.append(v); lb.append(-self.tau_max[j])
        try:
            x = W.solve_qp(G, a, Aeq, beq, np.vstack(rows), np.array(lb)); tau = x[nv:nv+nu]
        except Exception:
            tau = np.array([h[dof] for dof in self.act_dofs]) + W.KP_POS * (self.home - qcur) - W.KD_POS * vcur
            self._qp_fail = getattr(self, "_qp_fail", 0) + 1
        d.ctrl[:] = np.clip(tau, self.tau_min, self.tau_max)
        # ### S4 : logs
        self.log["ee_L"].append(e_ee["L"]); self.log["ee_R"].append(e_ee["R"])
        self.log["ee_regime"].append(self._regime())
        self.log["com_err"].append(float(np.linalg.norm(com[:2] - self.com_ref_xy)))
        self.log["ctrl_ms"].append((time.perf_counter() - t_perf) * 1e3)
        for fb in self.feet: self.log["grf"].append(self._grf_z(fb))


def _rmse(a):
    a = np.asarray(a, float); a = a[~np.isnan(a)]
    return float(np.sqrt(np.mean(a**2))) if a.size else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dist", type=float, default=3.0, help="distance de marche cible (m)")
    ap.add_argument("--stride", type=float, default=0.12, help="avance par pas (m)")
    # boite
    ap.add_argument("--payload", type=float, default=2.0, help="masse de la boite portee (kg)")
    ap.add_argument("--boxhx", type=float, default=0.12); ap.add_argument("--boxhy", type=float, default=0.18)
    ap.add_argument("--boxhz", type=float, default=0.12)
    ap.add_argument("--carryx", type=float, default=0.32, help="avance des mains (repere base) pour le port")
    ap.add_argument("--carryy", type=float, default=0.18, help="demi-ecart lateral des mains")
    ap.add_argument("--carryz", type=float, default=-0.15, help="hauteur des mains vs base")
    # taches EE
    ap.add_argument("--wee", type=float, default=W_EE_DEF); ap.add_argument("--kpee", type=float, default=KP_EE_DEF)
    ap.add_argument("--kdee", type=float, default=KD_EE_DEF)
    # sequenceur QS (memes defauts que talos_s3_stairs_qs)
    ap.add_argument("--rate", type=float, default=0.18); ap.add_argument("--zrate", type=float, default=0.35)
    ap.add_argument("--clear", type=float, default=0.10, help="garde swing au sol (m) — plat : 0.10 suffit")
    ap.add_argument("--tswing", type=float, default=0.65); ap.add_argument("--tsettle", type=float, default=0.8)
    ap.add_argument("--ttrmin", type=float, default=0.5); ap.add_argument("--ttrmax", type=float, default=3.0)
    ap.add_argument("--postol", type=float, default=0.035); ap.add_argument("--zcdrop", type=float, default=0.04)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--trace", action="store_true"); ap.add_argument("--traceevery", type=int, default=50)
    ap.add_argument("--viewer", action="store_true"); ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()

    # scene parametree
    with open("scene_carry.xml", "w") as f:
        f.write(build_carry_xml(a.boxhx, a.boxhy, a.boxhz, max(a.payload, 1e-3)))
    print("[scene] scene_carry.xml : boite %.2f x %.2f x %.2f m  masse %.2f kg"
          % (2*a.boxhx, 2*a.boxhy, 2*a.boxhz, a.payload))

    # gains QS (config gelee du succes S3-QS 3/3)
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0
    B.KP_COM_W, B.KD_COM_W, B.W_COM_W = 500.0, 180.0, 2500.0
    B.W_SWING = 2500.0
    W.KP_ORI, W.KD_ORI, W.W_ORI = 450.0, 80.0, 600.0
    W.MODEL = "scene_carry.xml"

    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    # masse de boite deja dans le XML ; on la force aussi ici si --payload change apres coup
    if a.payload > 0:
        m.body_mass[m.body("carry_box").id] = a.payload
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed); d.qvel[:m.nv] += 0.0  # bruit applique apres build
    com0 = d.subtree_com[m.body("base_link").id].copy(); x0m, z0m = float(com0[0]), float(com0[2])

    tgtL = np.array([a.carryx, +a.carryy, a.carryz]); tgtR = np.array([a.carryx, -a.carryy, a.carryz])

    def build(d_):
        T.apply_squat(m, d_, a.zcdrop)                       # jambes flechies
        it, eL, eR = carry_ik(m, d_, tgtL, tgtR)             # bras en pose de port
        print("[carry-IK] it=%d  err mains G=%.1f mm D=%.1f mm" % (it, eL*1e3, eR*1e3))
        place_box_and_weld(m, d_)                            # boite + welds (relpose runtime)
        if a.seed is not None:                               # bruit initial APRES mise en place
            rng = np.random.default_rng(a.seed)
            d_.qvel[:] += rng.normal(0.0, 0.01, m.nv); mujoco.mj_forward(m, d_)
        return S4CarryQS(m, d_, dist=a.dist, stride=a.stride,
                         w_ee=a.wee, kp_ee=a.kpee, kd_ee=a.kdee,
                         rate=a.rate, zrate=a.zrate, clear=a.clear, tswing=a.tswing,
                         t_settle=a.tsettle, t_tr_min=a.ttrmin, t_tr_max=a.ttrmax, postol=a.postol)

    c = build(d)
    dt = m.opt.timestep

    def trc(cc):
        com = d.subtree_com[cc.base]; bz = d.xpos[cc.box][2]
        print("[trc] t=%.2f %-8s mi=%d %-2s | CoM x=%.3f y=%.3f z=%.3f | box z=%.3f | EE L=%.3f R=%.3f | GRF L=%.0f R=%.0f"
              % (cc.t, cc.state, cc.mi, cc.phase, com[0], com[1], com[2], bz,
                 cc.log["ee_L"][-1] if cc.log["ee_L"] else 0.0,
                 cc.log["ee_R"][-1] if cc.log["ee_R"] else 0.0,
                 cc._grf_z(cc.left), cc._grf_z(cc.right)))

    if a.viewer:
        from mujoco import viewer as mjv
        with mjv.launch_passive(m, d) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            start = time.time()
            while viewer.is_running():
                c.update(dt); c.control(); mujoco.mj_step(m, d)
                if a.trace and int(round(c.t / dt)) % a.traceevery == 0: trc(c)
                viewer.cam.lookat[:] = d.xpos[c.base]; viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0: time.sleep(slp)
                if d.qpos[2] < 0.5:
                    print("[viewer] chute t=%.2f s (%s) — relance" % (c.t, c.state))
                    time.sleep(0.6); mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
                    c = build(d); start = time.time()
        return

    t_cap = c.T_SETTLE + len(c.moves) * (c.T_TR_MAX + c.T_SWING + c.T_SEEK_MAX) + 3.0
    fell_at = None; _ps = None
    while c.t < t_cap:
        c.update(dt); c.control(); mujoco.mj_step(m, d)
        if a.trace and (c.state != _ps or int(round(c.t / dt)) % a.traceevery == 0):
            trc(c); _ps = c.state
        if d.qpos[2] < 0.6: fell_at = c.t; (a.trace and trc(c)); break
        if c.state == "DONE" and (c.t - c.t_state) > 3.0: break

    # ---------- bilan S4 ----------
    com = d.subtree_com[c.base]; dist = float(com[0] - x0m)
    box_z = float(d.xpos[c.box][2]); box_held = box_z > (z0m - 0.35)   # boite pas tombee au sol
    upright = d.qpos[2] > 0.8 and fell_at is None
    eeL = np.array(c.log["ee_L"]); eeR = np.array(c.log["ee_R"]); reg = np.array(c.log["ee_regime"])
    cm = np.array(c.log["ctrl_ms"]); gr = np.array(c.log["grf"]) if c.log["grf"] else np.array([np.nan])
    cerr = np.array(c.log["com_err"]) if c.log["com_err"] else np.array([np.nan])
    walk = reg == 1
    ee_all = np.concatenate([eeL, eeR])
    ee_walk = np.concatenate([eeL[walk], eeR[walk]]) if walk.any() else ee_all
    ee_ok = (np.nanmax(ee_walk) < 0.05) if ee_walk.size else False
    reached = dist >= a.dist and c.state == "DONE"
    success = upright and reached and box_held and ee_ok
    success_func = bool(upright and reached and box_held)   # succes FONCTIONNEL (sans le gate EE)
    nq = int(getattr(c, "_qp_fail", 0)); ntick = max(len(cm), 1)
    print("=" * 72)
    print("S4-QS-CARRY  dist=%.1f stride=%.2f payload=%.1fkg W_EE=%.0f seed=%s"
          % (a.dist, a.stride, a.payload, a.wee, a.seed))
    print("  outcome    : %s%s | etat final=%s"
          % ("UPRIGHT" if upright else "FELL", "" if fell_at is None else "  (t=%.2f s)" % fell_at, c.state))
    print("  avancee x  : %.2f m (cible %.1f) | move mi=%d/%d | boite z=%.3f (%s)"
          % (dist, a.dist, c.mi, len(c.moves) - 1, box_z, "portee" if box_held else "TOMBEE"))
    print("  task success: fonctionnel=%s (debout+3m+DONE+boite) | strict=%s (+EE pic<5cm)"
          % ("SUCCESS" if success_func else "FAIL", "SUCCESS" if success else "FAIL"))
    print("  CoM tracking: RMSE %.1f mm | max %.1f mm" % (_rmse(cerr) * 1e3, (np.nanmax(cerr) if cerr.size else np.nan) * 1e3))
    print("  EE error    : RMSE %.1f mm | pic %.1f mm  (toutes phases, 2 mains)"
          % (_rmse(ee_all) * 1e3, (np.nanmax(ee_all) if ee_all.size else np.nan) * 1e3))
    print("  EE en MARCHE: RMSE %.1f mm | pic %.1f mm  (regime etabli — metrique H1)"
          % (_rmse(ee_walk) * 1e3, (np.nanmax(ee_walk) if ee_walk.size else np.nan) * 1e3))
    print("  GRF (pic)   : %.0f N | mean %.0f N" % (np.nanmax(gr), np.nanmean(gr)))
    if len(cm):
        print("  ctrl loop   : mean %.2f ms | p99 %.2f ms | QP feas %.1f%%"
              % (cm.mean(), np.percentile(cm, 99), 100.0 * (1 - nq / ntick)))
    print("=" * 72)
    if a.save:
        np.savez(a.save, ee_L=eeL, ee_R=eeR, ee_regime=reg, ctrl_ms=cm, grf=gr, com_err=cerr,
                 dist=dist, box_z=box_z, box_held=box_held, payload=a.payload, w_ee=a.wee,
                 success=success, success_func=success_func, done=bool(c.state == "DONE"),
                 upright=upright, fell_at=fell_at if fell_at is not None else -1.0,
                 seed=a.seed if a.seed is not None else -1)
        print("[ok] %s" % a.save)


if __name__ == "__main__":
    main()
