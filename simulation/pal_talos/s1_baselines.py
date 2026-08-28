"""
s1_baselines.py — ALTERNATIVE executors for the S1 protocol (test of H3).

H3: contact modeling + task-priority > naive torque control
(lower CoM deviation and contact force variance; falsified if the metrics are
equivalent to or worse than the baseline).

Two baselines, SAME interface as talos_wbc.WBC (.control(), .base, .com_ref,
._qp_fail) so they plug into validate_s1_batch.py without touching the protocol:

  1) PDGrav — gravity-compensated joint PD ("naive torque control"):
         tau = h_act + KP*(q_home - q) - KD*qdot
     This is EXACTLY the QP-WBC fallback (talos_wbc.control, except branch)
     promoted to primary controller. No notion of contact, CoM or cone.

  2) NullSpaceTP — task-priority with strict null-space projections
     (Sentis & Khatib 2005; recursion of Siciliano & Slotine 1991), with
     dynamically consistent inverses J# = M^-1 J^T (J M^-1 J^T)^+:
         priorities: contact (feet locked) > CoM > base orientation > posture
         qdd = sum of the projected contributions; then torques by least
         squares on  S^T tau + Jc^T f = M qdd + h  (floating-base dynamics).
     SAME task gains as the QP (imported from talos_wbc) — the ONLY
     difference is the solution mechanics: NO friction cone,
     NO unilaterality fz>=0, NO torque limits in the solve
     (a posteriori clip only). This is precisely the contrast tested by H3.

No modification to talos_wbc.py. The contact corners (LEGACY/MEASURED) concern
only the QP; the baselines do not consume them.
"""
import numpy as np, mujoco
import talos_wbc as W


class _S1Executor:
    """Common base: actuator selection, home/CoM references, batch interface."""

    def __init__(self, m, d, corners=None):        # corners accepted and ignored (harness uniformity)
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
        self._qp_fail = 0                       # no QP -> never a fallback

    # -- tasks identical to the QP (same gains, same errors) --
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
    """Baseline 1 — gravity-compensated joint PD (= QP fallback promoted to controller).

    scale: gain stiffening (KP*scale, KD*sqrt(scale)). scale=1 = gains
    matched to the QP posture task (60/12) — UNDER-TUNED: falls on its own in ~2.4 s
    (unstabilised pendulum). scale=10 (KP=600) = DEFENSIBLE baseline: stays standing
    and survives 300 N nominal (sandbox check 2026-07-16) because S1 is statically
    stable — the stiff PD still has NO notion of CoM or contact."""

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
    """Baseline 2 — null-space task-priority (Sentis & Khatib), without QP or cone."""

    RCOND = 1e-8    # pinv (rank deficient at singularities)

    def _dyn_pinv(self, J, Minv):
        """Dynamically consistent inverse J# = M^-1 J^T (J M^-1 J^T)^+."""
        Lam_inv = J @ Minv @ J.T
        return Minv @ J.T @ np.linalg.pinv(Lam_inv, rcond=self.RCOND)

    def control(self):
        m, d = self.m, self.d
        nv, nu = self.nv, self.nu
        M = np.zeros((nv, nv)); W.fill_fullM(m, d, M)
        Minv = np.linalg.inv(M + 1e-9 * np.eye(nv))
        h = d.qfrc_bias.copy()
        # -- priority 1: contact (feet locked 6D, Baumgarte identical to the QP) --
        cr = []
        for fb in self.feet:
            Jp = np.zeros((3, nv)); Jr = np.zeros((3, nv))
            mujoco.mj_jac(m, d, Jp, Jr, d.xpos[fb], fb)
            cr += [Jp, Jr]
        Jc = np.vstack(cr)                                   # 12 x nv
        a_c = -W.BAUMGARTE * (Jc @ d.qvel)
        # -- priority 2: CoM --
        Jcom = np.zeros((3, nv)); mujoco.mj_jacSubtreeCom(m, d, Jcom, self.base)
        com = d.subtree_com[self.base]
        a_com = W.KP_COM * (self.com_ref - com) - W.KD_COM * (Jcom @ d.qvel)
        # -- priority 3: base orientation --
        tmp = np.zeros((3, nv)); Jori = np.zeros((3, nv))
        mujoco.mj_jacBody(m, d, tmp, Jori, self.base)
        a_ori = W.KP_ORI * self._ori_err() - W.KD_ORI * (Jori @ d.qvel)
        # -- priority 4: posture --
        Jpos = np.zeros((nu, nv))
        for k, dof in enumerate(self.act_dofs):
            Jpos[k, dof] = 1.0
        _, _, a_pos = self._posture()
        # -- strict null-space recursion (acceleration) --
        qdd = np.zeros(nv); N = np.eye(nv)
        for J, a in ((Jc, a_c), (Jcom, a_com), (Jori, a_ori), (Jpos, a_pos)):
            Jr_ = J @ N
            Jh = self._dyn_pinv(Jr_, Minv)               # dyn. consistent inverse
            qdd = qdd + Jh @ (a - J @ qdd)
            N = N @ (np.eye(nv) - Jh @ Jr_)              # dyn. consistent projector
        # -- torques: least squares on the floating-base dynamics --
        #    S^T tau + Jc^T f = M qdd + h   (NO constraint on f: no cone,
        #    no fz>=0 — the H3 contrast)
        A = np.hstack([self.S, Jc.T])                        # nv x (nu+12)
        b = M @ qdd + h
        z, *_ = np.linalg.lstsq(A, b, rcond=None)
        tau = np.clip(z[:nu], self.tau_min, self.tau_max)
        d.ctrl[:] = tau
        return tau


EXECUTORS = {"pd": PDGrav, "ns": NullSpaceTP}
