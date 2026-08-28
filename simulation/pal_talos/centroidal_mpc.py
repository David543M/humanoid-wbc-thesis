"""
centroidal_mpc.py — CONVEX CENTROIDAL MPC planning layer (S2bis).

Written 2026-08-11. Answers literally the layer announced in the research
question (Ch1 l.168) and specified in Ch3 §3.3, never implemented until now.

DEPENDENCIES: numpy + proxsuite ONLY. No MuJoCo import, no import of the frozen
S2/S3/S4 files -> this module cannot regress the existing campaigns. It is
validatable and validated OFFLINE (test_centroidal_mpc.py).

===========================================================================
1. MODEL
===========================================================================
EXACT centroidal dynamics (Orin et al. 2013), state
    x = [ c (3) ; l (3) ; k (3) ]
with c the CoM position, l = m*c_dot the linear momentum, k the centroidal
angular momentum (about the CoM):

    c_dot = l / m
    l_dot = sum_i f_i + m*g
    k_dot = sum_i (p_i - c) x f_i                       <-- BILINEAR in (c, f)

The only non-convex term is the lever arm (p_i - c). It is linearised about a
reference trajectory c_hat(t) known in advance
(Di Carlo et al. 2018, "Dynamic Locomotion in the MIT Cheetah 3 via Convex
Model-Predictive Control"):

    k_dot ~ sum_i (p_i - c_hat_k) x f_i

The problem becomes a CONVEX QP in the contact forces. That is the fundamental
difference with the LIPM: neither constant CoM height nor zero angular
momentum. Both LIPM assumptions are lifted.

===========================================================================
2. DISCRETISATION (exact, no further approximation)
===========================================================================
A is nilpotent (A^2 = 0) since only the c row depends on l. Hence
    A_d = I + A*dt                       exact
    B_d = B*dt + A*B*dt^2/2              exact
    g_d = [ g*dt^2/2 ; m*g*dt ; 0 ]      exact

===========================================================================
3. CONDENSED QP
===========================================================================
Variables: U = [u_0 ... u_{N-1}], u_k = forces stacked at the N_c points.
    min  sum_k ||x_k - x_k^ref||^2_Q + ||u_k||^2_R   (+ terminal Q_N)
    s.t. 0 <= f_z,i <= f_max * s_ik        (s_ik = contact schedule)
         |f_x,i| <= mu * f_z,i , |f_y,i| <= mu * f_z,i    (pyramid, 4 facets)

Unilaterality per foot CORNER implicitly constrains the CoP to the support
polygon: this is the same construction as the QP-WBC executor, so the two
layers share the same contact model ("inter-layer compatibility" claimed in
Ch3 §3.4).

===========================================================================
4. OUTPUTS FOR THE QP-WBC EXECUTOR
===========================================================================
    c_ref, cdot_ref, cddot_ref  -> CoM task
    f_ref                       -> regularisation mu_f*||f - f_ref||^2
                                   (Ch3 eq:qp-cost WRITES f_ref; the current
                                   code implements f_ref = 0. The MPC supplies it.)
    k_ref                       -> angular momentum task (not present in the
                                   current executor; exposed for Ch6)

===========================================================================
5. SOLVER STATUS
===========================================================================
Lesson from the 2026-08-11 audit: ProxQP DOES NOT RAISE on infeasibility, it
returns its last iterate. Here the status is read AT EVERY call and a
non-converged solve is reported, counted, and replaced by the previous solution
(explicit fallback strategy, never silent).

===========================================================================
6. VALIDATION RESULT AND STRUCTURAL LIMIT  (2026-08-11)
===========================================================================
Validated offline against the exact NONLINEAR centroidal dynamics
(test_centroidal_mpc.py). What WORKS:
  * static equilibrium: gravity compensated EXACTLY (932.0 N for 95 kg),
    CoM drift zero to within 1e-4 m over 3 s;
  * walking over ~4 s (about 7 steps): CoM RMSE 45-53 mm, HEIGHT deviation
    2.5-4.3 mm (the LIPM would force it to 0 by assumption), angular momentum
    bounded at 6.5-6.8 kg.m2/s (the LIPM ignores it);
  * friction cone and unilaterality satisfied 100 % by construction;
  * solve time N=10, dt=50 ms: mean 4.9 ms, p99 7.9 ms
    -> meets the target set in the thesis ("linear MPC < 10 ms").

What DOES NOT WORK, and why it is not a bug:
  * beyond ~4-6 s the trajectory diverges (RMSE 2.9 m at 8 s);
  * tolerance to a lateral impulse: 50 N absorbed (RMSE 65 mm),
    100 N marginal (237 mm), 150 N divergent.

CAUSE. The footsteps are FIXED by the plan: the only rejection authority is
moving the CoP inside the footprint (~0.20 x 0.12 m). Since the lateral
dynamics is an inverted pendulum (omega ~ 3.4 rad/s, time constant 0.3 s), any
residual error grows exponentially, the CoP saturates, and divergence follows.
This is a KNOWN result: it is exactly why Herdt et al. (2010) make the FOOTSTEP
LOCATION a decision variable, and why the S2 walker of this thesis adapts the
step DURATION (Khadiv et al.).

In other words, this implementation establishes quantitatively that the
centroidal MPC layer ALONE is not enough: it needs step adaptation. The
event-based lever of S2 is therefore not a trick, it is the minimum adaptivity
required. ANY walking conclusion requires first adding footstep-location
adaptation, then the MuJoCo integration.
"""
import time
import numpy as np

try:
    import proxsuite
    _HAVE_PROXQP = True
except ImportError:                                     # pragma: no cover
    _HAVE_PROXQP = False

G = np.array([0.0, 0.0, -9.81])


# ---------------------------------------------------------------------------
#  Utilities
# ---------------------------------------------------------------------------
def skew(r):
    """S(r) such that S(r) @ f = r x f."""
    return np.array([[0.0, -r[2], r[1]],
                     [r[2], 0.0, -r[0]],
                     [-r[1], r[0], 0.0]])


class GaitSchedule:
    """Footstep plan + contact schedule, aligned on the S2 convention.

    Foot geometry identical to talos_wbc.CORNERS_MEASURED: 4 corners per foot.
    Sequence: initial DS, then alternation (brief DS, SS) like the DCM walker.
    """

    def __init__(self, step_len=0.08, t_step=0.50, ds_ratio=0.12, ly=0.085,
                 n_steps=70, corners_x=(+0.095, -0.105), corners_y=(+0.06, -0.06),
                 t_settle=0.8):
        self.step_len, self.t_step, self.ds_ratio = step_len, t_step, ds_ratio
        self.ly, self.n_steps, self.t_settle = ly, n_steps, t_settle
        self.corners = [(sx, sy) for sx in corners_x for sy in corners_y]
        self.n_pts_per_foot = len(self.corners)
        self.n_contacts = 2 * self.n_pts_per_foot          # 8 points in total

    def foot_centres(self, t):
        """Centres (x, y) of both feet at time t. Index 0 = left."""
        if t < self.t_settle:
            return np.array([0.0, +self.ly]), np.array([0.0, -self.ly])
        tau = t - self.t_settle
        k = int(tau / self.t_step)                          # step number
        k = min(k, self.n_steps - 1)
        # the right foot starts; at each step the swing foot advances by 2*step_len
        n_r = (k + 1) // 2                                  # number of right-foot placements
        n_l = k // 2
        xr = n_r * 2 * self.step_len
        xl = n_l * 2 * self.step_len
        return np.array([xl, +self.ly]), np.array([xr, -self.ly])

    def contact_points(self, t):
        """World positions of the N_c contact points (N_c x 3)."""
        cl, cr = self.foot_centres(t)
        pts = []
        for centre in (cl, cr):
            for sx, sy in self.corners:
                pts.append([centre[0] + sx, centre[1] + sy, 0.0])
        return np.asarray(pts)

    def contact_active(self, t):
        """Boolean vector (N_c,): 1 if the point bears load at time t."""
        n = self.n_pts_per_foot
        s = np.ones(self.n_contacts)
        if t < self.t_settle:
            return s                                        # initial double support
        tau = t - self.t_settle
        k = int(tau / self.t_step)
        phase = (tau - k * self.t_step) / self.t_step
        if phase < self.ds_ratio:
            return s                                        # transfer double support
        # single support: the advancing foot is in the air
        swing_is_right = (k % 2 == 0)
        if swing_is_right:
            s[n:] = 0.0
        else:
            s[:n] = 0.0
        return s

    def limit_cycle_state(self, mass, zc, t_start=None, lat_gain=0.75):
        """Initial state ON the periodic lateral limit cycle.

        A biped with pre-planned steps cannot start from rest: the lateral
        dynamics is an inverted pendulum, and any departure from the periodic
        cycle grows as e^{omega t} (omega ~ 3.4 rad/s, i.e. a 0.3 s time
        constant). Without this initialisation, divergence is guaranteed
        whatever the planner.

        Derivation (lateral LIPM, support alternating at +-ly, duration T): by
        requiring the state after one step to be the mirror image of the initial
        state, the system solves in closed form and gives
            y(0) = 0           (the CoM is on the median axis)
            ydot(0) = p_y * omega * tanh(omega*T/2)
        where p_y is the stance foot. This is the same result as the switching
        value xi_s = ly*tanh(omega*T/2) used by the DCM walker
        (talos_dcm_walk_timing.py l.87), obtained here independently.

        CALIBRATION of `lat_gain` (measured by sweep, 2026-08-11). The formula
        above assumes an alternation of pure SINGLE supports. The real plan
        contains 12 % double support, during which the lateral pendulum is not
        the one in the model -> the formula OVERESTIMATES vy. Sweep over 4 s:

            mult   0.70   0.75   0.80   0.85   0.90   1.00   1.20
            RMSE   45.1   46.8   52.7   63.2   82.7  240.6 1634.3  mm

        Flat optimum over [0.70, 0.80], abrupt divergence beyond: the signature
        of an unstable manifold. Default kept at 0.75, at the centre of the
        plateau.
        """
        omega = np.sqrt(9.81 / zc)
        t0 = self.t_settle if t_start is None else t_start
        # first single support: the right foot swings (k=0) -> LEFT stance, p_y = +ly
        p_y = +self.ly
        vy = lat_gain * p_y * omega * np.tanh(omega * self.t_step / 2.0)
        c_ref, v_ref = self.com_reference(t0, zc)
        c = np.array([c_ref[0], 0.0, zc])
        v = np.array([v_ref[0], vy, 0.0])
        return np.r_[c, mass * v, np.zeros(3)], t0

    def _support_centre(self, t):
        """Centre of the instantaneous support polygon (a square wave: discontinuous at DS->SS)."""
        cl, cr = self.foot_centres(t)
        act = self.contact_active(t)
        n = self.n_pts_per_foot
        if act[:n].any() and act[n:].any():
            return 0.5 * (cl + cr)
        return cl.copy() if act[:n].any() else cr.copy()

    def com_reference(self, t, zc, n_smooth=9):
        """CoM reference = the support plan SMOOTHED over one step period.

        The instantaneous support centre is a square wave: a biped's CoM cannot
        follow it, and taking it as the reference penalises the MPC for an error
        it is right to make. It is therefore averaged over +-T_step/2, which is
        the discrete equivalent of the low-pass filtering that relates the ZMP
        plan to the CoM trajectory. No LIPM assumption is introduced here: the
        smoothing applies to the REFERENCE, not to the model.
        """
        half = 0.5 * self.t_step
        ts = np.linspace(t - half, t + half, n_smooth)
        xy = np.mean([self._support_centre(max(tt, 0.0)) for tt in ts], axis=0)
        v = 0.0 if t < self.t_settle - half else self.step_len / self.t_step
        return np.array([xy[0], xy[1], zc]), np.array([v, 0.0, 0.0])


# ---------------------------------------------------------------------------
#  MPC
# ---------------------------------------------------------------------------
class CentroidalMPC:
    """Convex receding-horizon centroidal MPC over the contact forces."""

    def __init__(self, mass, schedule, horizon=12, dt=0.04, mu=0.7,
                 fz_max=1500.0, zc=0.86,
                 q_pos=(4.0e3, 4.0e3, 4.0e3), q_lin=(2.0e1, 2.0e1, 2.0e1),
                 q_ang=(2.0e2, 2.0e2, 2.0e2), r_force=1.0e-3,
                 terminal_scale=10.0, eps_abs=1e-6):
        if not _HAVE_PROXQP:
            raise SystemExit("[ABORT] proxsuite requis : pip install proxsuite")
        self.m, self.sched = float(mass), schedule
        self.N, self.dt, self.mu = int(horizon), float(dt), float(mu)
        self.fz_max, self.zc = float(fz_max), float(zc)
        self.nc = schedule.n_contacts
        self.nu = 3 * self.nc
        self.nx = 9
        self.Q = np.diag(np.r_[q_pos, q_lin, q_ang]).astype(float)
        self.QN = terminal_scale * self.Q
        self.R = r_force * np.eye(self.nu)
        self.eps_abs = eps_abs
        # --- etat solveur ---
        self._qp = None
        self._u_prev = np.zeros(self.N * self.nu)
        self.stats = dict(calls=0, solved=0, not_solved=0, statuses={},
                          t_ms=[], max_pri_res=0.0)

    # ---------------- dynamique discrete ----------------
    def _discrete(self, pts, c_hat):
        """(A_d, B_d, g_d) au point de linearisation c_hat, contacts pts."""
        dt, m = self.dt, self.m
        A = np.zeros((9, 9)); A[0:3, 3:6] = np.eye(3) / m
        B = np.zeros((9, self.nu))
        for i in range(self.nc):
            B[3:6, 3*i:3*i+3] = np.eye(3)
            B[6:9, 3*i:3*i+3] = skew(pts[i] - c_hat)
        bg = np.zeros(9); bg[3:6] = m * G
        Ad = np.eye(9) + A * dt                      # A^2 = 0 -> exact
        AB = A @ B
        Bd = B * dt + AB * (dt * dt / 2.0)           # exact
        gd = bg * dt + (A @ bg) * (dt * dt / 2.0)    # exact
        return Ad, Bd, gd

    # ---------------- construction et resolution ----------------
    def solve(self, x0, t0):
        """Solve the MPC at time t0 from the state x0 = [c, l, k].

        Retourne un dict : f_ref (nc x 3), c_ref, cdot_ref, cddot_ref, k_ref,
        status, solve_ms.
        """
        N, nu, nx, dt = self.N, self.nu, self.nx, self.dt
        # --- references and linearisation along the horizon ---
        ts = [t0 + (k + 1) * dt for k in range(N)]
        xref = np.zeros((N, nx))
        pts_k, act_k, chat_k = [], [], []
        for k, tk in enumerate(ts):
            c_r, v_r = self.sched.com_reference(tk, self.zc)
            xref[k, 0:3] = c_r
            xref[k, 3:6] = self.m * v_r
            xref[k, 6:9] = 0.0                        # moment cinetique cible nul
            pts_k.append(self.sched.contact_points(tk))
            act_k.append(self.sched.contact_active(tk))
            chat_k.append(c_r)                        # point de linearisation

        # --- condensation : x_k = Phi_k x0 + sum_j Gamma[k][j] u_j + gam_k ---
        Phi = np.zeros((N, nx, nx)); Gam = np.zeros((N, N, nx, nu)); gam = np.zeros((N, nx))
        Aprev = np.eye(nx)
        for k in range(N):
            Ad, Bd, gd = self._discrete(pts_k[k], chat_k[k])
            if k == 0:
                Phi[0] = Ad; gam[0] = gd; Gam[0, 0] = Bd
            else:
                Phi[k] = Ad @ Phi[k-1]
                gam[k] = Ad @ gam[k-1] + gd
                for j in range(k):
                    Gam[k, j] = Ad @ Gam[k-1, j]
                Gam[k, k] = Bd
            Aprev = Ad

        # --- Hessien et gradient ---
        H = np.zeros((N * nu, N * nu)); g = np.zeros(N * nu)
        for k in range(N):
            Qk = self.QN if k == N - 1 else self.Q
            e0 = Phi[k] @ x0 + gam[k] - xref[k]       # residu independant de U
            for j in range(k + 1):
                Gkj = Gam[k, j]
                g[j*nu:(j+1)*nu] += 2.0 * (Gkj.T @ Qk @ e0)
                for i in range(k + 1):
                    H[j*nu:(j+1)*nu, i*nu:(i+1)*nu] += 2.0 * (Gkj.T @ Qk @ Gam[k, i])
        for k in range(N):
            H[k*nu:(k+1)*nu, k*nu:(k+1)*nu] += 2.0 * self.R
        H = 0.5 * (H + H.T) + 1e-9 * np.eye(N * nu)

        # --- inequalities: 1 fz row (bounded) + 4 friction rows per point ---
        rows, lo, up = [], [], []
        for k in range(N):
            for i in range(self.nc):
                b = k * nu + 3 * i
                v = np.zeros(N * nu); v[b+2] = 1.0
                rows.append(v); lo.append(0.0); up.append(self.fz_max * act_k[k][i])
                for sg in (-1.0, 1.0):
                    v = np.zeros(N * nu); v[b+2] = self.mu; v[b+0] = sg
                    rows.append(v); lo.append(0.0); up.append(1e20)
                    v = np.zeros(N * nu); v[b+2] = self.mu; v[b+1] = sg
                    rows.append(v); lo.append(0.0); up.append(1e20)
        C = np.vstack(rows); l = np.array(lo); u = np.array(up)

        # --- resolution (statut LU systematiquement, cf. audit 2026-08-11) ---
        n_var, n_in = N * nu, C.shape[0]
        if self._qp is None:
            self._qp = proxsuite.proxqp.dense.QP(n_var, 0, n_in)
            self._qp.settings.eps_abs = self.eps_abs
            self._qp.settings.eps_primal_inf = 1e-12      # cf. faux positif S1
            self._qp.settings.eps_dual_inf = 1e-12
            self._qp.init(H, g, np.zeros((0, n_var)), np.zeros(0), C, l, u)
        else:
            self._qp.update(H=H, g=g, C=C, l=l, u=u)
        t_start = time.perf_counter()
        self._qp.solve()
        dt_ms = (time.perf_counter() - t_start) * 1e3

        info = self._qp.results.info
        st = str(info.status).split(".")[-1]
        self.stats["calls"] += 1
        self.stats["statuses"][st] = self.stats["statuses"].get(st, 0) + 1
        self.stats["t_ms"].append(dt_ms)
        self.stats["max_pri_res"] = max(self.stats["max_pri_res"], float(info.pri_res))
        if st == "PROXQP_SOLVED":
            self.stats["solved"] += 1
            U = np.asarray(self._qp.results.x)
            self._u_prev = U.copy()
        else:
            self.stats["not_solved"] += 1
            U = self._u_prev.copy()                  # repli EXPLICITE, jamais silencieux

        u0 = U[:nu].reshape(self.nc, 3)
        cddot = (u0.sum(axis=0) + self.m * G) / self.m
        c_r, v_r = self.sched.com_reference(t0 + dt, self.zc)
        return dict(f_ref=u0, c_ref=c_r, cdot_ref=v_r, cddot_ref=cddot,
                    k_ref=np.zeros(3), status=st, solve_ms=dt_ms,
                    U=U, xref0=xref[0])

    # ---------------- rapport ----------------
    def report(self):
        s = self.stats
        t = np.array(s["t_ms"]) if s["t_ms"] else np.array([np.nan])
        return ("MPC centroidal : %d appels | %d resolus | %d NON resolus\n"
                "                 solve  mean %.2f ms | p99 %.2f ms | max %.2f ms\n"
                "                 pri_res max %.2e | statuts %s"
                % (s["calls"], s["solved"], s["not_solved"],
                   np.nanmean(t), np.nanpercentile(t, 99), np.nanmax(t),
                   s["max_pri_res"], s["statuses"]))
