"""
s1_baselines.py — executeurs ALTERNATIFS pour le protocole S1 (test de H3).

H3 : contact modeling + task-priority > torque control naif
(CoM deviation et contact force variance plus faibles ; falsifie si metriques
equivalentes ou pires vs baseline).

Deux baselines, MEME interface que talos_wbc.WBC (.control(), .base, .com_ref,
._qp_fail) pour brancher dans validate_s1_batch.py sans toucher au protocole :

  1) PDGrav — PD articulaire gravite-compense ("torque control naif") :
         tau = h_act + KP*(q_home - q) - KD*qdot
     C'est EXACTEMENT le repli du QP-WBC (talos_wbc.control, branche except)
     promu en controleur principal. Aucune notion de contact, de CoM ni de cone.

  2) NullSpaceTP — task-priority par projections d'espace nul strictes
     (Sentis & Khatib 2005 ; recursion de Siciliano & Slotine 1991), avec
     inverses dynamiquement consistantes J# = M^-1 J^T (J M^-1 J^T)^+ :
         priorites : contact (pieds figes) > CoM > orientation base > posture
         qdd = somme des contributions projetees ; puis couples par moindres
         carres sur  S^T tau + Jc^T f = M qdd + h  (dynamique corps flottant).
     MEMES gains de tache que le QP (importes de talos_wbc) — la SEULE
     difference est la mecanique de resolution : PAS de cone de friction,
     PAS d'unilateralite fz>=0, PAS de limites de couple dans la resolution
     (clip a posteriori seulement). C'est precisement le contraste teste par H3.

Aucune modification de talos_wbc.py. Les coins de contact (LEGACY/MEASURED) ne
concernent que le QP ; les baselines n'en consomment pas.
"""
import numpy as np, mujoco
import talos_wbc as W


class _S1Executor:
    """Base commune : selection actionneurs, references home/CoM, interface batch."""

    def __init__(self, m, d, corners=None):        # corners accepte et ignore (uniformite harnais)
        self.m, self.d = m, d
        self.nv, self.nu = m.nv, m.nu
        self.base = m.body("base_link").id
        self.S = np.zeros((m.nv, m.nu))
        for i in range(m.nu):
            jn = m.actuator(i).name.replace("_torque", "")
            dof = m.jnt_dofadr[m.joint(jn).id]
            self.S[dof, i] = 1.0
        self.act_dofs = [m.jnt_dofadr[m.joint(m.actuator(i).name.replace("_torque", "")).id]
                         for i in range(m.nu)]
        self.act_qadr = [m.jnt_qposadr[m.joint(m.actuator(i).name.replace("_torque", "")).id]
                         for i in range(m.nu)]
        self.tau_min = m.actuator_forcerange[:, 0].copy()
        self.tau_max = m.actuator_forcerange[:, 1].copy()
        bad = self.tau_min >= self.tau_max
        self.tau_min[bad], self.tau_max[bad] = -300.0, 300.0
        self.feet = [m.body("leg_left_6_link").id, m.body("leg_right_6_link").id]
        mujoco.mj_forward(m, d)
        self.home = np.array([d.qpos[a] for a in self.act_qadr])
        self.com_ref = d.subtree_com[self.base].copy()
        self._qp_fail = 0                       # pas de QP -> jamais de repli

    # -- taches identiques au QP (memes gains, memes erreurs) --
    def _posture(self):
        d = self.d
        q = np.array([d.qpos[a] for a in self.act_qadr])
        v = np.array([d.qvel[a] for a in self.act_dofs])
        return q, v, W.KP_POS * (self.home - q) - W.KD_POS * v

    def _ori_err(self):
        d = self.d
        q = d.qpos[3:7]; err = np.zeros(3)
        neg = np.zeros(4); res = np.zeros(4)
        mujoco.mju_negQuat(neg, q)
        mujoco.mju_mulQuat(res, np.array([1.0, 0, 0, 0]), neg)
        mujoco.mju_quat2Vel(err, res, 1.0)
        return err


class PDGrav(_S1Executor):
    """Baseline 1 — PD articulaire gravite-compense (= repli QP promu controleur).

    scale : raidissement des gains (KP*scale, KD*sqrt(scale)). scale=1 = gains
    apparies a la tache posture du QP (60/12) — SOUS-REGLE : tombe seul en ~2.4 s
    (pendule non stabilise). scale=10 (KP=600) = baseline DEFENDABLE : tient debout
    et survit a 300 N nominal (verifie sandbox 2026-07-16) car S1 est statiquement
    stable — le PD raide n'a toujours AUCUNE notion de CoM ni de contact."""

    def __init__(self, m, d, corners=None, scale=1.0):
        super().__init__(m, d, corners)
        self.kp = W.KP_POS * scale
        self.kd = W.KD_POS * float(np.sqrt(scale))

    def control(self):
        d = self.d
        h = d.qfrc_bias
        q, v, _ = self._posture()
        tau = np.array([h[dof] for dof in self.act_dofs]) \
            + self.kp * (self.home - q) - self.kd * v
        tau = np.clip(tau, self.tau_min, self.tau_max)
        d.ctrl[:] = tau
        return tau


class NullSpaceTP(_S1Executor):
    """Baseline 2 — task-priority null-space (Sentis & Khatib), sans QP ni cone."""

    RCOND = 1e-8    # pinv (rang deficient aux singularites)

    def _dyn_pinv(self, J, Minv):
        """Inverse dynamiquement consistante J# = M^-1 J^T (J M^-1 J^T)^+."""
        Lam_inv = J @ Minv @ J.T
        return Minv @ J.T @ np.linalg.pinv(Lam_inv, rcond=self.RCOND)

    def control(self):
        m, d = self.m, self.d
        nv, nu = self.nv, self.nu
        M = np.zeros((nv, nv)); W.fill_fullM(m, d, M)
        Minv = np.linalg.inv(M + 1e-9 * np.eye(nv))
        h = d.qfrc_bias.copy()
        # -- priorite 1 : contact (pieds figes 6D, Baumgarte identique au QP) --
        cr = []
        for fb in self.feet:
            Jp = np.zeros((3, nv)); Jr = np.zeros((3, nv))
            mujoco.mj_jac(m, d, Jp, Jr, d.xpos[fb], fb)
            cr += [Jp, Jr]
        Jc = np.vstack(cr)                                   # 12 x nv
        a_c = -W.BAUMGARTE * (Jc @ d.qvel)
        # -- priorite 2 : CoM --
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, self.base)
        com = d.subtree_com[self.base]
        a_com = W.KP_COM * (self.com_ref - com) - W.KD_COM * (Jcom @ d.qvel)
        # -- priorite 3 : orientation base --
        tmp = np.zeros((3, nv)); Jori = np.zeros((3, nv))
        mujoco.mj_jacBody(m, d, tmp, Jori, self.base)
        a_ori = W.KP_ORI * self._ori_err() - W.KD_ORI * (Jori @ d.qvel)
        # -- priorite 4 : posture --
        Jpos = np.zeros((nu, nv))
        for k, dof in enumerate(self.act_dofs):
            Jpos[k, dof] = 1.0
        _, _, a_pos = self._posture()
        # -- recursion stricte d'espace nul (acceleration) --
        qdd = np.zeros(nv); N = np.eye(nv)
        for J, a in ((Jc, a_c), (Jcom, a_com), (Jori, a_ori), (Jpos, a_pos)):
            Jr_ = J @ N
            Jh = self._dyn_pinv(Jr_, Minv)               # inverse dyn. consistante
            qdd = qdd + Jh @ (a - J @ qdd)
            N = N @ (np.eye(nv) - Jh @ Jr_)              # projecteur dyn. consistant
        # -- couples : moindres carres sur la dynamique corps flottant --
        #    S^T tau + Jc^T f = M qdd + h   (AUCUNE contrainte sur f : pas de cone,
        #    pas de fz>=0 — le contraste H3)
        A = np.hstack([self.S, Jc.T])                        # nv x (nu+12)
        b = M @ qdd + h
        z, *_ = np.linalg.lstsq(A, b, rcond=None)
        tau = np.clip(z[:nu], self.tau_min, self.tau_max)
        d.ctrl[:] = tau
        return tau


EXECUTORS = {"pd": PDGrav, "ns": NullSpaceTP}
