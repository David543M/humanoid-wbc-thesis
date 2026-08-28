"""
lipm_mpc.py — receding-horizon LIPM MPC with OPTIMISED FOOTSTEP PLACEMENT.

Written 2026-08-11, after centroidal_mpc.py. Formulation of Herdt et al.
(2010), "Online walking motion generation with automatic footstep placement".

DEPENDENCIES: numpy + proxsuite. No MuJoCo, no import of the frozen files.

===========================================================================
WHY THIS FILE EXISTS
===========================================================================
`centroidal_mpc.py` answers "centroidal MPC" literally (linear AND angular
momentum, free CoM height) but, with FIXED footsteps, its only rejection
authority is the CoP inside the footprint: it diverges beyond ~4-6 s and
tolerates only a ~50 N lateral impulse (measurements 2026-08-11).

The LIPM has a property the centroidal model lacks: the ZMP is a LINEAR
function of the state,
        z = c - (zc/g) * c_ddot
Taking the state x = [c, c_dot, c_ddot] and the JERK as control, everything
is linear AND THE x AND y AXES DECOUPLE. The QP drops from 240 variables to
~19 PER AXIS. At that price, the FOOTSTEP POSITIONS can be added as decision
variables -- exactly the adaptation the centroidal MPC lacks.

The trade-off is explicit: constant CoM height, zero angular momentum. This
planner therefore does NOT answer the word "centroidal" in the research
question; it answers "receding horizon + explicit constraints".

===========================================================================
FORMULATION (per axis, horizon N, period T)
===========================================================================
    x_{k+1} = A x_k + B u_k ,  u = jerk
    A = [[1, T, T^2/2], [0, 1, T], [0, 0, 1]] ,  B = [T^3/6, T^2/2, T]^T
    z_k = C x_k ,               C = [1, 0, -zc/g]

Condensed form (constant matrices, precomputed once):
    Z = P_zs x_0 + P_zu U        (predicted ZMP)
    V = P_vs x_0 + P_vu U        (predicted velocity)

Decision variables: X = [ U (N) ; F (m) ]  ->  N + m ~ 19
    U = jerks, F = positions of the next m footsteps on this axis.

Cost:
    a/2 ||U||^2                      jerk regularisation
  + b/2 ||V - V_ref||^2              velocity tracking (the walking command)
  + c/2 ||Z - Vc*f_c - Vf*F||^2      ZMP centred in the support foot
  + d/2 ||F - F_nom||^2              stride regularity

HARD constraints:
  (1) ZMP in the footprint:   -h <= Z - Vc*f_c - Vf*F <= h
  (2) sagittal stride:        |F_j - F_{j-1}| <= stride_max
  (3) lateral separation:     F_j - F_{j-1} in [+min_sep, +max_sep] or
                              [-max_sep, -min_sep] depending on the foot -- this
                              is what forbids leg crossing.

SOLVER STATUS read at every call (cf. ProxQP audit of 2026-08-11).

===========================================================================
OFFLINE VALIDATION RESULTS  (2026-08-11, test_lipm_mpc.py)
===========================================================================
Protocol: start AT REST, without initialisation on the limit cycle (the
centroidal MPC does not start without it). Retained tuning: T=0.05 s, N=32.

ESTABLISHED, AND REPRODUCIBLE
  * 15 steps over 8 s WITHOUT DIVERGENCE, starting from standstill; lateral
    oscillation |y|max 62 mm (compare with the separation ly = 85 mm);
  * 100 % of QPs converged, status checked at every call;
  * solve time for BOTH AXES: 0.45 ms (0.17 ms at T=0.1 s), against
    4.8 ms for the centroidal MPC -- ~19x cheaper, for ~35 variables per
    axis instead of 240;
  * IMPULSE TOLERANCE, lateral, post-push excursion:
        0 N -> 49 mm | 100 N -> 52 | 200 N -> 81 | 300 N -> 244 | 400 N -> 688
    against the centroidal MPC with FIXED STEPS: 50 N absorbed, 100 N
    marginal, 150 N DIVERGENT. Adapting the step LOCATION is therefore worth
    about a factor 4 on the disturbance margin. That is THE result of this file.

NOT ESTABLISHED -- do not present as settled
  * THE ZMP CONSTRAINT IS NOT PROPERLY SATISFIED. The measured rate of
    "CoP inside the footprint" varies from 65 % to 86 % depending on how the
    double-support phase is counted. The definition of the metric changed
    three times during development, so THIS FIGURE IS NOT RELIABLE and the
    slack is active by an unquantified amount. An MPC whose main constraint
    is not verified cannot be declared functional.
  * residual lateral drift of a few centimetres over 8 s;
  * speed 10-15 % above the commanded value;
  * PLANT = nominal LIPM + disturbance: outside the push, the optimised model
    and the simulated model coincide. Strictly weaker validation than that of
    the centroidal MPC, which runs against the exact nonlinear centroidal
    dynamics;
  * no coupling with the QP-WBC executor (requires MuJoCo, untested).

DEVELOPMENT HISTORY -- five defects found by MEASUREMENT, none by reasoning:
support-foot engagement offset by one step; double-support polygon too narrow
at start-up; velocity command active during the settling phase (the CoM ran
ahead of the support foot); hard ZMP constraint without a slack variable,
which sent ProxQP to 10 000 iterations; and above all a PLANT inconsistent
with the QP -- z(t+T) was constrained while c(t) - (zc/g) c_ddot(t+T) was
simulated. That last point alone explained most of the sagittal violations.

HONEST CONCLUSION. This file establishes a COMPARISON -- plan adaptivity
dominates model richness in the regime tested (flat ground, low speed). It
does not establish a walker.
"""
import time
import numpy as np

try:
    import proxsuite
    _HAVE_PROXQP = True
except ImportError:                                     # pragma: no cover
    _HAVE_PROXQP = False

GRAV = 9.81


class LIPMWalkMPC:
    """Receding-horizon LIPM MPC, footstep placement as decision variables."""

    def __init__(self, zc=0.86, T=0.1, N=16, t_step=0.5, t_settle=0.8,
                 step_len=0.08, ly=0.085,
                 foot_hx=0.085, foot_hy=0.045,
                 stride_max=0.30, min_sep=0.11, max_sep=0.42, ds_ratio=0.12,
                 w_jerk=1e-5, w_vel=1.0, w_zmp=1e-2, w_step=10.0, w_slack=1e4,
                 k_ypos=2.0, v_lat_max=0.15,
                 eps_abs=1e-6, max_iter=400):
        if not _HAVE_PROXQP:
            raise SystemExit("[ABORT] proxsuite required: pip install proxsuite")
        self.zc, self.T, self.N = float(zc), float(T), int(N)
        self.t_step, self.t_settle = float(t_step), float(t_settle)
        self.step_len, self.ly = float(step_len), float(ly)
        self.foot_h = {"x": float(foot_hx), "y": float(foot_hy)}
        self.stride_max, self.min_sep, self.max_sep = stride_max, min_sep, max_sep
        self.ds_ratio = float(ds_ratio)
        self.w = dict(jerk=w_jerk, vel=w_vel, zmp=w_zmp, step=w_step, slack=w_slack)
        self.eps_abs = eps_abs; self.max_iter = int(max_iter)
        self.k_ypos, self.v_lat_max = float(k_ypos), float(v_lat_max)
        self.m = int(np.ceil(self.N * self.T / self.t_step)) + 1   # future steps seen
        self._build_prediction()
        self._qp = {}
        self.stats = dict(calls=0, solved=0, not_solved=0, statuses={}, t_ms=[])

    # ------------------------------------------------------------------
    def _build_prediction(self):
        """Condensed prediction matrices (constant)."""
        N, T, zc = self.N, self.T, self.zc
        P_ps = np.zeros((N, 3)); P_pu = np.zeros((N, N))
        P_vs = np.zeros((N, 3)); P_vu = np.zeros((N, N))
        P_as = np.zeros((N, 3)); P_au = np.zeros((N, N))
        for k in range(N):
            kk = k + 1
            P_ps[k] = [1.0, kk * T, kk * kk * T * T / 2.0]
            P_vs[k] = [0.0, 1.0, kk * T]
            P_as[k] = [0.0, 0.0, 1.0]
            for j in range(k + 1):
                d = k - j
                P_pu[k, j] = (1.0 + 3.0 * d + 3.0 * d * d) * T**3 / 6.0
                P_vu[k, j] = (1.0 + 2.0 * d) * T**2 / 2.0
                P_au[k, j] = T
        self.P_zs = P_ps - (zc / GRAV) * P_as
        self.P_zu = P_pu - (zc / GRAV) * P_au
        self.P_vs, self.P_vu = P_vs, P_vu

    # ------------------------------------------------------------------
    def _selectors(self, t):
        """Which foot supports at each sample of the horizon?

        Returns (Vc, Vf, k0, side0):
          Vc (N,)   1 if the sample is supported by the CURRENT foot (known),
          Vf (N,m)  one-hot of the FUTURE supporting step (decision variable),
          k0        index of the current step, side0 the current support foot
                    (+1 = left, -1 = right).
        """
        N, T = self.N, self.T
        Vc = np.zeros(N); Vf = np.zeros((N, self.m))
        k0 = self.step_index(t)
        for k in range(N):
            j = self.step_index(t + (k + 1) * T)
            if j <= k0:
                Vc[k] = 1.0                            # current foot, known position
            else:
                Vf[k, min(j - k0 - 1, self.m - 1)] = 1.0   # future step, variable
        # GaitSchedule convention: the RIGHT foot swings on even steps
        # -> LEFT support (+1) on even steps, RIGHT (-1) on odd ones
        side0 = +1.0 if (max(k0, 0) % 2 == 0) else -1.0
        return Vc, Vf, k0, side0

    def step_index(self, t):
        """-1 during the initial settling phase, then 0, 1, 2, ..."""
        if t < self.t_settle:
            return -1
        return int((t - self.t_settle) // self.t_step)

    # ------------------------------------------------------------------
    def _solve_axis(self, axis, x0, Vc, Vf, f_c, F_nom, v_ref, sides, h_vec):
        """One QP for one axis. X = [U (N) ; F (m)].

        h_vec (N,): half-footprint ALLOWED to the ZMP at each sample. It is not
        constant: during the initial double support the polygon is that of BOTH
        feet, markedly wider than that of a single foot. Imposing the
        single-foot footprint from the start forces a violent start-up and
        makes the first step diverge.
        """
        N, m = self.N, self.m
        n = N + m + N                        # U | F | S (slacks on the ZMP)
        Pzs, Pzu, Pvs, Pvu = self.P_zs, self.P_zu, self.P_vs, self.P_vu
        Z = np.zeros((N, N))

        z0 = Pzs @ x0                       # ZMP part independent of U
        v0 = Pvs @ x0
        # --- linear residuals ---
        Azmp = np.hstack([Pzu, -Vf, Z])     # r_zmp = Pzu U - Vf F + (z0 - Vc f_c)
        bzmp = z0 - Vc * f_c
        Avel = np.hstack([Pvu, np.zeros((N, m)), Z])
        bvel = v0 - np.asarray(v_ref) * np.ones(N)
        Astep = np.hstack([np.zeros((m, N)), np.eye(m), np.zeros((m, N))])
        bstep = -F_nom
        Ajerk = np.hstack([np.eye(N), np.zeros((N, m)), Z])
        Aslack = np.hstack([np.zeros((N, N)), np.zeros((N, m)), np.eye(N)])

        w = self.w
        H = (w["jerk"] * Ajerk.T @ Ajerk
             + w["vel"] * Avel.T @ Avel
             + w["zmp"] * Azmp.T @ Azmp
             + w["step"] * Astep.T @ Astep
             + w["slack"] * Aslack.T @ Aslack)
        g = (w["vel"] * Avel.T @ bvel
             + w["zmp"] * Azmp.T @ bzmp
             + w["step"] * Astep.T @ bstep)
        H = 0.5 * (H + H.T) + 1e-9 * np.eye(n)

        # --- constraints ---
        rows, lo, up = [], [], []
        # (1) ZMP in the footprint, RELAXED by a heavily penalised slack s_k >= 0.
        #     An MPC with HARD ZMP constraints and a frozen step duration becomes
        #     infeasible as soon as a disturbance exceeds the CoP authority: the
        #     solver then runs to the iteration limit. The slack keeps the QP
        #     always feasible and makes the violation MEASURABLE instead of fatal.
        for k in range(N):
            a = Azmp[k].copy(); a[N + m + k] = -1.0        # ... - s_k <= h
            rows.append(a); lo.append(-1e20); up.append(h_vec[k] - bzmp[k])
            b = -Azmp[k].copy(); b[N + m + k] = -1.0       # -(...) - s_k <= h
            rows.append(b); lo.append(-1e20); up.append(h_vec[k] + bzmp[k])
            c = np.zeros(n); c[N + m + k] = 1.0            # s_k >= 0
            rows.append(c); lo.append(0.0); up.append(1e20)
        # (2) sagittal stride / (3) lateral separation, between consecutive steps.
        # The row expresses  F_j - F_{j-1}  (or F_0 - f_c for j = 0); the constant
        # term f_c therefore moves to the right-hand side for j = 0 only.
        for j in range(m):
            r = np.zeros(n); r[N + j] = 1.0
            offset = f_c if j == 0 else 0.0
            if j > 0:
                r[N + j - 1] = -1.0
            if axis == "x":
                d_lo, d_hi = -self.stride_max, +self.stride_max
            else:
                s = sides[j]                     # landing foot: +1 left, -1 right
                d_lo, d_hi = (s * self.min_sep, s * self.max_sep)
                if s < 0:                        # order the bounds
                    d_lo, d_hi = d_hi, d_lo
            rows.append(r); lo.append(offset + d_lo); up.append(offset + d_hi)
        C = np.vstack(rows); l = np.array(lo); u = np.array(up)

        key = (axis, n, C.shape[0])
        qp = self._qp.get(key)
        if qp is None:
            qp = proxsuite.proxqp.dense.QP(n, 0, C.shape[0])
            qp.settings.eps_abs = self.eps_abs
            qp.settings.eps_primal_inf = 1e-12
            qp.settings.eps_dual_inf = 1e-12
            qp.settings.max_iter = self.max_iter
            qp.init(H, g, np.zeros((0, n)), np.zeros(0), C, l, u)
            self._qp[key] = qp
        else:
            qp.update(H=H, g=g, C=C, l=l, u=u)
        t0 = time.perf_counter(); qp.solve(); ms = (time.perf_counter() - t0) * 1e3
        st = str(qp.results.info.status).split(".")[-1]
        return np.asarray(qp.results.x), st, ms

    # ------------------------------------------------------------------
    def solve(self, xs, ys, t, foot_c):
        """xs, ys: states [c, c_dot, c_ddot]. foot_c: (fx, fy) of the support foot.

        Returns dict(jerk, footsteps, zmp_pred, status, solve_ms).
        """
        Vc, Vf, k0, side0 = self._selectors(t)
        # nominal plan for the next m steps
        Fx_nom, Fy_nom, sides = [], [], []
        for j in range(self.m):
            Fx_nom.append(foot_c[0] + (j + 1) * self.step_len)
            s = -side0 if (j % 2 == 0) else side0      # strict alternation
            sides.append(s)
            Fy_nom.append(s * self.ly)
        Fx_nom = np.array(Fx_nom); Fy_nom = np.array(Fy_nom)

        # half-footprint per sample: polygon of BOTH feet as long as the
        # sample falls inside the initial double support
        # DOUBLE-SUPPORT samples: initial settling, AND the start of each step
        # (ds_ratio overlap, like the S2 walker). Without this phase the ZMP
        # would have to jump from one foot to the other through the midpoint,
        # which belongs to neither footprint: a structural lateral violation at
        # every transition (measured: 19 % of ticks).
        def _is_ds(tt):
            if tt < self.t_settle:
                return 1.0
            ph = ((tt - self.t_settle) % self.t_step) / self.t_step
            return 1.0 if ph < self.ds_ratio else 0.0
        in_ds = np.array([_is_ds(t + (k + 1) * self.T) for k in range(self.N)])
        hx_vec = self.foot_h["x"] * np.ones(self.N)
        hy_vec = self.foot_h["y"] + in_ds * self.ly

        # ZERO velocity command during settling: commanding the cruise speed
        # from t=0 sends the CoM ahead of the support foot before the first
        # step; the ZMP then saturates at the front of the footprint and the
        # walker falls forward taking maximum strides.
        v_ref_x = (self.step_len / self.t_step) * (1.0 - in_ds)
        Xx, stx, msx = self._solve_axis("x", xs, Vc, Vf, foot_c[0], Fx_nom,
                                        v_ref_x, sides, hx_vec)
        # Outer lateral POSITION loop. A Herdt MPC tracks a VELOCITY; a zero
        # lateral command is satisfied by any constant offset, and the gait then
        # drifts diagonally (measured: -0.08 to -0.23 m over 8 s, insensitive to
        # w_step). Position is therefore closed by a proportional term,
        # saturated so it never dominates the lateral cycle.
        v_ref_y = float(np.clip(-self.k_ypos * ys[0], -self.v_lat_max, self.v_lat_max))
        Xy, sty, msy = self._solve_axis("y", ys, Vc, Vf, foot_c[1], Fy_nom,
                                        v_ref_y, sides, hy_vec)

        self.stats["calls"] += 1
        for st in (stx, sty):
            self.stats["statuses"][st] = self.stats["statuses"].get(st, 0) + 1
        ok = (stx == "PROXQP_SOLVED" and sty == "PROXQP_SOLVED")
        self.stats["solved" if ok else "not_solved"] += 1
        self.stats["t_ms"].append(msx + msy)

        N = self.N
        return dict(jerk=np.array([Xx[0], Xy[0]]),
                    footsteps=np.vstack([Xx[N:N + self.m], Xy[N:N + self.m]]).T,
                    slack=float(max(np.abs(Xx[N + self.m:]).max(), np.abs(Xy[N + self.m:]).max())),
                    status=(stx, sty), solve_ms=msx + msy,
                    k0=k0, side0=side0)

    # ------------------------------------------------------------------
    def report(self):
        s = self.stats
        t = np.array(s["t_ms"]) if s["t_ms"] else np.array([np.nan])
        return ("LIPM-MPC: %d calls | %d solved | %d NOT solved\n"
                "           solve (2 axes) mean %.3f ms | p99 %.3f ms | max %.3f ms\n"
                "           statuses %s"
                % (s["calls"], s["solved"], s["not_solved"],
                   np.nanmean(t), np.nanpercentile(t, 99), np.nanmax(t),
                   s["statuses"]))
