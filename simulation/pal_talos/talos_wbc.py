"""
TALOS — Whole-Body Control via QP for double-support balance + push recovery.

==> REFERENCE IMPLEMENTATION — NOT YET VALIDATED ON HARDWARE/SIM <==
    The compute sandbox was unavailable when this was written, so it has not been
    executed end-to-end. Treat it as a faithful first implementation of the
    formulation in WBC_methodology.md, to be debugged on first run. Most likely
    points to check: contact-corner offsets, selection matrix, QP conditioning.

Run from inside  mujoco_menagerie/pal_talos/  (loads scene_motor.xml):
    pip install mujoco numpy quadprog imageio imageio-ffmpeg matplotlib
    python talos_wbc.py                 # balance
    python talos_wbc.py --push          # balance + lateral push test
    python talos_wbc.py --video         # render skeleton (needs talos_sim.py alongside)

QP variables:  x = [ qdd(nv) , tau(nu) , f(3*ncontact) ]
"""
import argparse, numpy as np, mujoco
try:
    import quadprog
except ImportError:
    quadprog = None
try:
    import proxsuite                       # solveur QP rapide (< 1 ms, workspace persistant)
    _HAVE_PROXQP = True
except ImportError:
    _HAVE_PROXQP = False
_PROX_CACHE = {}                            # solveurs ProxQP indexes par (n, n_eq, n_in)

# ============================================================================
# INSTRUMENTATION DU SOLVEUR — ajoutee 2026-08-11
# Backup pre-patch : talos_wbc_PRE_QPSTATUS_20260811.py
#
# MOTIF (audit 2026-08-11). quadprog LEVAIT ValueError quand le QP etait
# infaisable ; ProxQP NE LEVE RIEN : il retourne le dernier itere. Le passage a
# ProxQP a donc silencieusement supprime la detection d'echec, et le compteur
# `_qp_fail` de control() ne voyait plus que les exceptions Python. La metrique
# « QP fallbacks: 0 / 100 % feasible » de Ch5 etait donc aveugle aux
# non-convergences du solveur.
#
# GARANTIE DE NEUTRALITE. Par defaut QP_STRICT=False -> _qp_record() ne fait que
# LIRE qp.results.info et n'altere jamais le vecteur retourne : les resultats
# sont bit-identiques a ceux d'avant le patch. Toute exception levee par
# l'instrumentation elle-meme est avalee (elle ne doit jamais casser la boucle).
#
# MODE STRICT (opt-in) : QP_STRICT=True, ou variable d'environnement
# WBC_QP_STRICT=1, fait lever QPNotConverged (sous-classe de ValueError, donc
# capturee par les DEUX sites de repli : `except (ValueError, LinAlgError)` de
# WBC.control() et `except Exception` de DCMWalk.control()). Cela RESTAURE
# exactement la semantique quadprog : non-convergence -> repli compte.
#
# Un statut PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE est compte comme un ECHEC :
# il signale un QP primal-infaisable pour lequel ProxQP a rendu le point
# faisable le plus proche — inacceptable pour un WBC (contraintes de contact et
# de couple violees sans signal).
# ============================================================================
import os

class QPNotConverged(ValueError):
    """Le solveur QP n'a pas converge (statut != PROXQP_SOLVED)."""


QP_STRICT = os.environ.get("WBC_QP_STRICT", "") not in ("", "0", "false", "False")

# NB : on ne loggue PAS info.solve_time — proxsuite ne le remplit que si
# settings.compute_timings=True, ce qui ajouterait un cout a la boucle de
# controle et casserait la garantie de neutralite. Le temps par tick est deja
# mesure par les scenarios (log["ctrl_ms"]).
QP_STATS = {"calls": 0, "solved": 0, "not_solved": 0, "statuses": {},
            "max_pri_res": 0.0, "max_dua_res": 0.0, "max_iter": 0}


def qp_stats_reset():
    QP_STATS.update(calls=0, solved=0, not_solved=0, statuses={},
                    max_pri_res=0.0, max_dua_res=0.0, max_iter=0)


def qp_stats_report():
    s = QP_STATS
    n = max(s["calls"], 1)
    return ("QP solver : %d appels | %d resolus | %d NON resolus (%.4f %%)\n"
            "            pri_res max %.3e | dua_res max %.3e | iter max %d\n"
            "            statuts : %s"
            % (s["calls"], s["solved"], s["not_solved"],
               100.0 * s["not_solved"] / n,
               s["max_pri_res"], s["max_dua_res"], s["max_iter"],
               s["statuses"] or "{}"))


def _qp_record(qp):
    """Lit le statut ProxQP. N'ALTERE JAMAIS la solution retournee."""
    try:
        info = qp.results.info
        st = str(getattr(info, "status", "?")).split(".")[-1]
        QP_STATS["calls"] += 1
        QP_STATS["statuses"][st] = QP_STATS["statuses"].get(st, 0) + 1
        for key, attr in (("max_pri_res", "pri_res"), ("max_dua_res", "dua_res"),
                          ("max_iter", "iter")):
            v = float(getattr(info, attr, 0.0) or 0.0)
            if v > QP_STATS[key]:
                QP_STATS[key] = v
        ok = (st == "PROXQP_SOLVED")
        QP_STATS["solved" if ok else "not_solved"] += 1
        if not ok and QP_STRICT:
            raise QPNotConverged(st)
    except QPNotConverged:
        raise
    except Exception:
        pass                                    # l'instrumentation ne casse jamais la boucle

MODEL = "scene_motor.xml"
MU = 0.7                      # friction coefficient
# Modeles de coins de contact ((sx pointe, sx talon), (sy gauche, sy droite)) :
# LEGACY   = modele historique — les configs gelees S2/S3/S4 ont ete reglees dessus
#            (sanity checks 2026-07-16 : les corriger regresse S3-QS 3/3->1/3 et S2).
# MEASURED = empreinte reelle mesuree (s1_diag 2026-07-16 : x [-0.105,+0.095] les 2
#            pieds ; y via plateau CoP lateral +10 mm et semelle 0.12 m). Utilise par
#            S1 : enveloppe nominale +13% sagittal / +11% lateral, vol post-poussee
#            elimine, pics d'atterrissage 3-12 kN -> ~1 kN.
CORNERS_LEGACY   = ((+0.11, -0.07), (+0.05, -0.05))
CORNERS_MEASURED = ((+0.095, -0.105), (+0.06, -0.06))
FZ_MIN = 1.0                 # min normal force per contact (N)
# task gains
KP_COM, KD_COM = 240.0, 30.0
KP_ORI, KD_ORI = 300.0, 35.0
KP_POS, KD_POS = 60.0, 12.0
KP_FOOT, KD_FOOT = 200.0, 20.0   # tache d'orientation de pied (semelle a plat)
# task weights
W_COM, W_ORI, W_POS = 100.0, 40.0, 1.0
W_CONTACT = 1.0e4            # soft contact no-slip (keeps feet planted)
# regularization
EPS_QDD, EPS_TAU, EPS_F = 1e-3, 1e-4, 1e-5
BAUMGARTE = 25.0            # contact velocity damping
# ---- levier hanche S1 (2026-07-17, OPTIONNEL, defaut OFF -> S2-S5 et batchs intacts) ----
# Strategie "lean into push" : quand l'excursion CoM depasse HIP_ON (CoP proche
# saturation), la REFERENCE d'orientation du bassin est biaisee dans la direction
# de l'excursion (K_LEAN rad/m, plafond LEAN_MAX) -> le tronc pivote = recrutement
# du moment cinetique centroidal (hip strategy), puis se redresse quand l'excursion
# repasse sous HIP_OFF (hysteresis ; retour naturel via la meme tache, gains intacts).
# Valide sandbox 2026-07-17 : enveloppe sagittale nominale 340->380 N (+12 %),
# peak dev REDUIT a 340 N (44 vs 63 mm), gate inactif aux poussees moderees
# (150 N : 0 tick actif -> comportement nominal identique). Backup pre-patch :
# talos_wbc_PRE_HIP_20260717.py
HIP_ON, HIP_OFF = 0.030, 0.012   # hysteresis sur ||dev CoM|| (m)
K_LEAN, LEAN_MAX = 10.0, 0.5     # rad de lean par m d'excursion ; plafond (rad)


def solve_qp(G, a, Aeq, beq, Aineq, bineq):
    """min 0.5 xᵀ G x - aᵀ x  s.t. Aeq x = beq, Aineq x >= bineq.

    Solveur par defaut : ProxQP (proxsuite), workspace persistant indexe par
    dimensions -> init() au 1er appel, update() ensuite (< 1 ms/cycle). Repli quadprog
    si proxsuite absent. Convention ProxQP : min 0.5 xᵀHx + gᵀx  s.t. Ax=b, l<=Cx<=u."""
    n = G.shape[0]
    G = 0.5 * (G + G.T) + 1e-8 * np.eye(n)         # symetrise + PD
    if _HAVE_PROXQP:
        neq, nin = Aeq.shape[0], Aineq.shape[0]
        u = 1e20 * np.ones(nin)                     # Aineq x >= bineq  ->  l=bineq, u=+inf
        key = (n, neq, nin)
        qp = _PROX_CACHE.get(key)
        if qp is None:
            qp = proxsuite.proxqp.dense.QP(n, neq, nin)
            qp.settings.eps_abs = 1e-6
            qp.init(G, -a, Aeq, beq, Aineq, bineq, u)
            _PROX_CACHE[key] = qp
        else:
            qp.update(H=G, g=-a, A=Aeq, b=beq, C=Aineq, l=bineq, u=u)
        qp.solve()
        _qp_record(qp)                          # instrumentation : lecture seule (voir en-tete)
        return np.asarray(qp.results.x)
    # --- repli quadprog ---
    C = np.vstack([Aeq, Aineq]).T                   # quadprog: Cᵀ x >= b
    b = np.concatenate([beq, bineq]); meq = Aeq.shape[0]
    return quadprog.solve_qp(G, a, C, b, meq)[0]


_FULLM_NEW = None
def fill_fullM(m, d, dst):
    """Dense mass matrix into dst, compatible across MuJoCo versions.
    MuJoCo >= 3.10:  mj_fullM(m, d, dst).   Older:  mj_fullM(m, dst, d.qM).
    The right form is probed once and then cached."""
    global _FULLM_NEW
    if _FULLM_NEW is None:
        try:
            mujoco.mj_fullM(m, d, dst); _FULLM_NEW = True; return
        except (TypeError, ValueError):
            mujoco.mj_fullM(m, dst, d.qM); _FULLM_NEW = False; return
    if _FULLM_NEW:
        mujoco.mj_fullM(m, d, dst)
    else:
        mujoco.mj_fullM(m, dst, d.qM)


class WBC:
    def __init__(self, m, d, corners=None, hip=False):
        self.m, self.d = m, d
        # --- instrumentation energetique (ajout 2026-08-25) ---------------------
        # Compteurs d'integrale. Ecrits en fin de control(), APRES le clip et donc
        # sur le couple reellement applique. Relus par AUCUN chemin de controle :
        # strictement inertes vis-a-vis de la dynamique et de la config gelee.
        self.E_mech = 0.0                    # int |tau . qdot| dt   [J]
        self.E_sq   = 0.0                    # int ||tau||^2   dt   [N^2 m^2 s]
        self.mass   = float(mujoco.mj_getTotalmass(m))   # [kg], pour le CoT
        self._corners_model = corners if corners is not None else CORNERS_LEGACY
        self.hip = hip                   # levier hanche (S1 uniquement, defaut OFF)
        self._hip_active = False
        self.hip_ticks = 0
        self.nv, self.nu = m.nv, m.nu
        self.base = m.body("base_link").id
        # actuator -> dof selection (Sᵀ: nv x nu)
        self.S = np.zeros((m.nv, m.nu))
        for i in range(m.nu):
            jn = m.actuator(i).name.replace("_torque", "")
            dof = m.jnt_dofadr[m.joint(jn).id]
            self.S[dof, i] = 1.0
            # actuated joint dof list for posture task / home pose
        self.act_dofs = [m.jnt_dofadr[m.joint(m.actuator(i).name.replace("_torque","")).id] for i in range(m.nu)]
        self.act_qadr = [m.jnt_qposadr[m.joint(m.actuator(i).name.replace("_torque","")).id] for i in range(m.nu)]
        self.tau_min = m.actuator_forcerange[:, 0].copy()
        self.tau_max = m.actuator_forcerange[:, 1].copy()
        bad = self.tau_min >= self.tau_max
        self.tau_min[bad], self.tau_max[bad] = -300.0, 300.0
        # feet + contact corners (local offsets, foot frame)
        self.feet = [m.body("leg_left_6_link").id, m.body("leg_right_6_link").id]
        self.corners_local = {}
        mujoco.mj_forward(m, d)
        for fb in self.feet:
            R = d.xmat[fb].reshape(3, 3); p = d.xpos[fb]
            sole_world = np.array([p[0], p[1], 0.0])       # ground under the foot at home
            sole_local = R.T @ (sole_world - p)
            offs = []
            sxs, sys = self._corners_model                  # defaut LEGACY (voir constantes)
            for sx in sxs:                                  # pointe / talon
                for sy in sys:
                    offs.append(sole_local + np.array([sx, sy, 0.0]))
            self.corners_local[fb] = offs
        self.ncp = sum(len(v) for v in self.corners_local.values())
        # references
        self.home = np.array([d.qpos[a] for a in self.act_qadr])
        self.com_ref = d.subtree_com[self.base].copy()
        self.foot_home_quat = {fb: d.xquat[fb].copy() for fb in self.feet}  # orientation 'a plat'
        self.hard_contact = True    # no-slip: True=egalite dure 6D/pied (defaut, mieux conditionne), False=cout mou w=1e4
        self._Aineq, self._bineq = self._build_ineq()   # inegalites constantes (friction + couples) -> precalcul

    def _build_ineq(self):
        """Inegalites constantes (pyramide de friction + limites de couple), pre-calculees une fois."""
        nv, nu, ncp = self.nv, self.nu, self.ncp; n = nv + nu + 3 * ncp
        rows, lb = [], []
        for c in range(ncp):
            b = nv + nu + 3 * c
            v = np.zeros(n); v[b+2] = 1.0; rows.append(v); lb.append(FZ_MIN)
            for sg in (-1.0, 1.0):
                v = np.zeros(n); v[b+2] = MU; v[b+0] = sg; rows.append(v); lb.append(0.0)
                v = np.zeros(n); v[b+2] = MU; v[b+1] = sg; rows.append(v); lb.append(0.0)
        for j in range(nu):
            v = np.zeros(n); v[nv+j] = 1.0; rows.append(v); lb.append(self.tau_min[j])
            v = np.zeros(n); v[nv+j] = -1.0; rows.append(v); lb.append(-self.tau_max[j])
        return np.vstack(rows), np.array(lb)

    # ---- helpers ----
    def contact_jac(self):
        m, d = self.m, self.d
        rows, pts = [], []
        for fb in self.feet:
            R = d.xmat[fb].reshape(3, 3); p = d.xpos[fb]
            for cl in self.corners_local[fb]:
                pw = p + R @ cl
                Jp = np.zeros((3, m.nv))
                mujoco.mj_jac(m, d, Jp, None, pw, fb)
                rows.append(Jp); pts.append(pw)
        return np.vstack(rows), pts                          # (3ncp x nv)

    def foot_ori_task(self, fb, kp=None, kd=None):
        """Jacobien rotationnel + acceleration desiree pour garder le pied fb a plat
        (orientation = pose home). Retourne (Jr 3xnv, a_ori 3).
        kp/kd optionnels : impedance programmee au poser (--softland) ; defaut =
        gains module KP_FOOT/KD_FOOT (comportement historique inchange)."""
        m, d = self.m, self.d
        Jp = np.zeros((3, m.nv)); Jr = np.zeros((3, m.nv))
        mujoco.mj_jac(m, d, Jp, Jr, d.xpos[fb], fb)          # Jr = jacobien d'orientation
        q = d.xquat[fb]; err = np.zeros(3); neg = np.zeros(4); res = np.zeros(4)
        mujoco.mju_negQuat(neg, q)
        mujoco.mju_mulQuat(res, self.foot_home_quat[fb], neg)
        mujoco.mju_quat2Vel(err, res, 1.0)                    # rotation courant -> home
        a = (KP_FOOT if kp is None else kp) * err \
            - (KD_FOOT if kd is None else kd) * (Jr @ d.qvel)
        return Jr, a

    def control(self):
        m, d = self.m, self.d
        nv, nu, ncp = self.nv, self.nu, self.ncp
        nf = 3 * ncp
        n = nv + nu + nf
        # dynamics quantities
        M = np.zeros((nv, nv)); fill_fullM(m, d, M)   # version-agnostic dense mass matrix
        h = d.qfrc_bias.copy()
        Jc, _ = self.contact_jac()
        # ---- tasks ----
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, self.base)
        com = d.subtree_com[self.base]; com_v = Jcom @ d.qvel
        a_com = KP_COM * (self.com_ref - com) - KD_COM * com_v
        Jori = np.zeros((3, nv)); tmp = np.zeros((3, nv))
        mujoco.mj_jacBody(m, d, tmp, Jori, self.base)        # jacr = Jori
        # base orientation error (upright): use rotation vector of base quat
        q = d.qpos[3:7]; err = np.zeros(3)
        dq = np.array([1.0, 0, 0, 0]); neg = np.zeros(4)
        mujoco.mju_negQuat(neg, q); res = np.zeros(4)
        mujoco.mju_mulQuat(res, dq, neg); mujoco.mju_quat2Vel(err, res, 1.0)
        omega = Jori @ d.qvel
        a_ori = KP_ORI * err - KD_ORI * omega
        # ---- levier hanche optionnel : biais de reference d'orientation (lean into push) ----
        if self.hip:
            devv = com[:2] - self.com_ref[:2]
            dev = float(np.linalg.norm(devv))
            if dev > HIP_ON: self._hip_active = True
            elif dev < HIP_OFF: self._hip_active = False
            if self._hip_active:
                self.hip_ticks += 1
                lean = min(K_LEAN * dev, LEAN_MAX)
                ax = np.array([-devv[1], devv[0], 0.0])   # axe de rotation = z x dev
                nx = np.linalg.norm(ax)
                if nx > 1e-9:
                    err = err + lean * (ax / nx)          # cible penchee VERS la poussee
                    a_ori = KP_ORI * err - KD_ORI * omega
        # posture (actuated joints)
        Jpos = np.zeros((nu, nv))
        for k, dof in enumerate(self.act_dofs):
            Jpos[k, dof] = 1.0
        qcur = np.array([d.qpos[a] for a in self.act_qadr])
        vcur = np.array([d.qvel[a] for a in self.act_dofs])
        a_pos = KP_POS * (self.home - qcur) - KD_POS * vcur

        # ---- build QP cost over x=[qdd,tau,f] ----
        G = np.zeros((n, n)); a = np.zeros(n)
        def add(J, rhs, w):                                  # w*||J qdd - rhs||²
            G[:nv, :nv] += 2 * w * (J.T @ J)
            a[:nv]      += 2 * w * (J.T @ rhs)
        add(Jcom, a_com, W_COM)
        add(Jori, a_ori, W_ORI)
        add(Jpos, a_pos, W_POS)
        # contact no-slip : cout mou (defaut) OU egalite dure (hard_contact, voir plus bas)
        if not self.hard_contact:
            add(Jc, -BAUMGARTE * (Jc @ d.qvel), W_CONTACT)
        # regularization
        G[:nv, :nv]            += 2 * EPS_QDD * np.eye(nv)
        G[nv:nv+nu, nv:nv+nu]  += 2 * EPS_TAU * np.eye(nu)
        G[nv+nu:, nv+nu:]      += 2 * EPS_F * np.eye(nf)

        # ---- equality: dynamics ONLY (full row rank)  M qdd - Sᵀ tau - Jcᵀ f = -h ----
        Aeq = np.zeros((nv, n)); Aeq[:, :nv] = M
        Aeq[:, nv:nv+nu] = -self.S
        Aeq[:, nv+nu:] = -Jc.T
        beq = -h
        if self.hard_contact:                          # no-slip en egalite dure (pied fige : Jfoot qdd = -baumgarte*Jfoot qvel)
            cr, cb = [], []
            for fb in self.feet:
                Jp6 = np.zeros((3, nv)); Jr6 = np.zeros((3, nv))
                mujoco.mj_jac(m, d, Jp6, Jr6, d.xpos[fb], fb)
                for J in (Jp6, Jr6):
                    row = np.zeros((3, n)); row[:, :nv] = J
                    cr.append(row); cb.append(-BAUMGARTE * (J @ d.qvel))
            Aeq = np.vstack([Aeq] + cr); beq = np.concatenate([beq] + cb)

        # ---- inequalities (Aineq x >= bineq) : constantes, pre-calculees en __init__ ----
        Aineq, bineq = self._Aineq, self._bineq

        try:
            x = solve_qp(G, a, Aeq, beq, Aineq, bineq)
            tau = x[nv:nv+nu]
        except (ValueError, np.linalg.LinAlgError):
            # repli : compensation de gravite + PD de posture sur les joints actionnes
            tau = np.array([h[dof] for dof in self.act_dofs]) \
                + KP_POS * (self.home - qcur) - KD_POS * vcur
            self._qp_fail = getattr(self, "_qp_fail", 0) + 1
        tau = np.clip(tau, self.tau_min, self.tau_max)
        # --- accumulation energetique (inerte) --------------------------------
        _dt = m.opt.timestep                 # 1 control() par mj_step : pas de decimation
        self.E_mech += abs(float(tau @ vcur)) * _dt
        self.E_sq   += float(tau @ tau) * _dt
        d.ctrl[:] = tau
        return tau


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--push", action="store_true")
    ap.add_argument("--force", type=float, default=150.0, help="amplitude de la poussee (N)")
    ap.add_argument("--axis", choices=["x", "y"], default="y", help="direction: y=lateral, x=sagittal")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--viewer", action="store_true", help="viewer 3D MuJoCo temps reel + fleches de force de contact")
    ap.add_argument("--push-delay", type=float, default=2.0, help="instant (s, temps sim) de la 1ere poussee")
    ap.add_argument("--push-every", type=float, default=4.0, help="periode entre poussees (s, temps sim)")
    ap.add_argument("--freefall", action="store_true", help="coupe le WBC: chute libre (compare la vraie vitesse de chute)")
    ap.add_argument("--torques", action="store_true", help="fleches de couple a chaque articulation (longueur ~ |tau|)")
    ap.add_argument("--dur", type=float, default=8.0)
    ap.add_argument("--corners", choices=["legacy", "measured"], default="measured",
                    help="coins de contact: measured = config S1 validee 2026-07-16 ; "
                         "legacy = modele historique S2-S5 (defaut des IMPORTS inchange)")
    ap.add_argument("--hip", action="store_true",
                    help="levier hanche S1 (lean-into-push) : etend l'enveloppe sagittale "
                         "au-dela de la saturation CoP (valide sandbox +12 %%)")
    args = ap.parse_args()
    if not _HAVE_PROXQP and quadprog is None:
        raise SystemExit("Installer un solveur QP:  pip install proxsuite  (ou quadprog)")

    m = mujoco.MjModel.from_xml_path(MODEL); d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    wbc = WBC(m, d, corners=CORNERS_MEASURED if args.corners == "measured" else CORNERS_LEGACY,
              hip=args.hip)
    dt = m.opt.timestep

    # ---- viewer 3D temps reel avec fleches de force de contact ----
    if args.viewer:
        import time
        from mujoco import viewer as mjv
        axis = 0 if args.axis == "x" else 1
        m.vis.map.force = 0.002          # echelle des fleches de force (longueur par Newton)
        m.vis.scale.forcewidth = 0.04
        IMPULSE = 0.1                    # duree PHYSIQUE de la poussee (s) = test valide (impulsion = force x 0.1 s)
        ARROW_SHOW = 0.6                 # duree d'AFFICHAGE de la fleche (s), decouplee de la physique
        ARROW_SCALE = 0.0015             # longueur de la fleche bassin par Newton
        TQ_SCALE = 0.012                 # longueur des fleches de couple (m par N.m)
        with mjv.launch_passive(m, d) as viewer:
            viewer.cam.distance = 4.5; viewer.cam.elevation = -12; viewer.cam.azimuth = 120
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True   # fleches GRF aux pieds
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_PERTFORCE] = True      # fleche perturbation (Ctrl+glisser)
            if args.freefall:
                d.qvel[1] = 0.05              # petit desequilibre lateral pour amorcer la bascule
                mujoco.mj_forward(m, d)
            start = time.time(); next_push = args.push_delay
            while viewer.is_running():
                if args.freefall:
                    d.ctrl[:] = 0.0          # aucun couple -> chute libre
                else:
                    wbc.control()
                d.xfrc_applied[wbc.base, :] = 0.0
                if args.push and next_push <= d.time < next_push + IMPULSE:    # PHYSIQUE : 0.1 s seulement
                    d.xfrc_applied[wbc.base, axis] = args.force
                elif args.push and d.time >= next_push + ARROW_SHOW:
                    next_push += args.push_every
                mujoco.mj_step(m, d)
                # --- fleche de la force appliquee sur le BASSIN ---
                # tail a l'EXTERIEUR du corps, pointe (arrowhead) sur le bassin -> bien visible
                sc = viewer.user_scn; sc.ngeom = 0
                fvec = np.zeros(3)                                            # AFFICHAGE prolonge (0.6 s)
                if args.push and next_push <= d.time < next_push + ARROW_SHOW:
                    fvec[axis] = args.force
                nf = np.linalg.norm(fvec)
                if nf > 1e-6:
                    dir = fvec / nf
                    L = nf * ARROW_SCALE
                    p_head = d.xpos[wbc.base].copy()
                    p_tail = p_head - dir * (L + 0.25)          # part de ~25 cm hors du torse
                    g = sc.geoms[sc.ngeom]
                    mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                        np.zeros(3), np.eye(3).ravel(),
                                        np.array([1.0, 0.45, 0.0, 1.0], dtype=np.float32))   # orange
                    mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.05, p_tail, p_head)
                    sc.ngeom += 1
                if args.torques:                       # fleches de couple par articulation (le long de l'axe)
                    for ai in range(m.nu):
                        if sc.ngeom >= sc.maxgeom: break
                        jid = m.actuator_trnid[ai, 0]
                        tau = float(d.ctrl[ai]); Lt = tau * TQ_SCALE
                        if abs(Lt) < 0.01: continue
                        anc = d.xanchor[jid]; p1 = anc + d.xaxis[jid] * Lt
                        mag = min(abs(tau) / 40.0, 1.0)    # 0..1 -> couleur vert->rouge
                        g = sc.geoms[sc.ngeom]
                        mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3),
                                            np.zeros(3), np.eye(3).ravel(),
                                            np.array([mag, 1.0 - mag, 0.1, 1.0], dtype=np.float32))
                        mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_ARROW, 0.012, anc, p1)
                        sc.ngeom += 1
                viewer.cam.lookat[0] = float(d.xpos[wbc.base][0]); viewer.cam.lookat[1] = float(d.xpos[wbc.base][1])
                viewer.sync()
                slp = (start + d.time) - time.time()
                if slp > 0: time.sleep(slp)
        return
    frames = []
    render = None
    if args.video:
        import talos_sim   # reuse skeleton renderer
        render = talos_sim.render
    n = int(args.dur / dt); fe = max(1, int(1.0 / (30 * dt)))
    peak = 0.0
    for i in range(n):
        wbc.control()
        dev = float(np.linalg.norm(d.subtree_com[wbc.base][:2] - wbc.com_ref[:2]))
        peak = max(peak, dev)
        # disturbance: 150 N lateral impulse for 0.1 s at t=3 s
        if args.push and 3.0 <= d.time < 3.1:
            d.xfrc_applied[wbc.base, :] = 0.0
            d.xfrc_applied[wbc.base, 0 if args.axis == "x" else 1] = args.force
        else:
            d.xfrc_applied[wbc.base, :] = 0.0
        mujoco.mj_step(m, d)
        if args.video and i % fe == 0:
            frames.append(render(m, d))
    com = d.subtree_com[wbc.base]
    fails = getattr(wbc, "_qp_fail", 0)
    print("end: base z=%.3f  CoM=(%.3f,%.3f,%.3f)  %s  | peak CoM dev=%.3f m | QP fallbacks: %d/%d" % (
        d.qpos[2], com[0], com[1], com[2], "UPRIGHT" if d.qpos[2] > 0.8 else "FELL", peak, fails, n))
    if args.video:
        import imageio.v2 as imageio
        imageio.mimsave("talos_wbc.mp4", frames, fps=30); print("[ok] talos_wbc.mp4")


if __name__ == "__main__":
    main()
