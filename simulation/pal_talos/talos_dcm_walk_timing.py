"""
TALOS — S2: 3 m walk with ADAPTIVE STEP TIMING (event-based touchdown).

Direct complement to WBC_planning_diagnosis.md §4: the closed-loop --walk-cl
(DCM feedback + capture-point + cpswing) doubles survival but falls around
the 6th step because the decision remains ON A FIXED SCHEDULE. This file makes
the walking cycle EVENT-DRIVEN (Khadiv et al., step timing adaptation, T-RO 2020):

  E1. START: walking does not start at a fixed time but when the
      measured DCM reaches the entry condition of the lateral limit cycle
      (d_lat <= ly - xi_s) during weight transfer.
  E2. TOUCHDOWN: during single support the lateral DCM diverges as
      d(tau) = d0*e^{omega*tau}; touchdown is triggered when d reaches
      the switching value d* = ly + xi_s:
          T_touchdown = tau + ln(d*/d)/omega   (can only SHORTEN the step)
  E3. FOOT GUARD: the contact switch only happens if the swing
      foot is actually near the ground (< 1.5 cm) — never a
      hard no-slip constraint on a foot in the air.

The QP-WBC (talos_wbc.py, validated on S1) and talos_dcm_walk.py remain UNCHANGED —
everything is overridden in a subclass. The June laws (--feedback,
--footfb, --cpswing, --footori) are on by default, disengageable with --no-*.

To run from pal_talos/:
    python talos_dcm_walk_timing.py                 # S2: 40 steps x 0.08 = 3.2 m
    python talos_dcm_walk_timing.py --steps 10      # short debug run
    python talos_dcm_walk_timing.py --no-timing     # ablation (= walk-cl-like)
    python talos_dcm_walk_timing.py --viewer        # real-time MuJoCo viewer
    python talos_dcm_walk_timing.py --seed 3 --save s2.npz
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B

T_END     = 2.5    # final settling balance after the last step (s)
XFER_RATE = 0.10   # weight transfer rate (m/s) in the initial phase
XFER_TMIN = 0.40   # minimum transfer duration (s)
XFER_TMAX = 4.0    # transfer timeout (s) — starts anyway + warning
FOOT_TOL  = 0.010  # foot height tolerance to allow touchdown (m)
Z_LAND    = 0.60   # descent rate of the REFERENCE in landing mode (m/s)
Z_DESC    = 0.25   # observed PHYSICAL descent rate of the foot (m/s) — for anticipation
MIN_SEP   = 0.11   # min lateral separation between foot centers (m) — anti-crossing
MAX_SEP   = 0.42   # max lateral separation (m) — anti-overstride (wide: recovery authority)
SAG_MAX   = 0.13   # sagittal capture limit: DCM ahead of stance (m) -> immediate touchdown
# --- soft landing (--softland) : anti-ankle-wobble at touchdown ---
DEBOUNCE_N = 2     # consecutive contact ticks required before switching stance (4 ms @ 500 Hz)
ZLAND_K    = 15.0  # ease-in: v_descent ref = K*h (1/s) — decelerates ONLY below ZLAND_MIN/K
                   # (~1.2 cm): touchdown delay ~+45 ms vs baseline. v2-v3 (K=8, decel from 2.2 cm
                   # + ~2 cm foot PD lag): touchdown ~0.15 s late -> DCM overshoot (FELL)
ZLAND_MIN  = 0.18  # impact velocity floor (m/s) — v1=0.10 REJECTED (touchdown far too late)
ZDESC_SOFT = 0.18  # observed physical descent rate with ease-in (for t_lat anticipation)
LAND_WIN   = 0.10  # landing-metrics window (s) — logged WITH or WITHOUT --softland (A/B)


def apply_squat(m, d, drop):
    """Bends the knees in the INITIAL CONFIGURATION, bypassing the
    WBC: hip -a, knee +2a, ankle -a (feet flat), base lowered
    to keep the soles on the ground. The controller then builds its
    home posture, zc and omega on this bent pose — clear of the
    straight-leg singularity from the first tick."""
    if drop <= 1e-6:
        return
    L_LEG = 0.76                        # TALOS thigh+shin (approx.; base numerically corrected)
    alpha = float(np.arccos(np.clip(1.0 - drop / L_LEG, 0.2, 1.0)))
    feet = [m.body("leg_left_6_link").id, m.body("leg_right_6_link").id]
    z0 = min(float(d.xpos[b][2]) for b in feet)
    for side in ("left", "right"):
        for num, dq in (("3", -alpha), ("4", 2.0 * alpha), ("5", -alpha)):
            j = m.joint("leg_%s_%s_joint" % (side, num))
            d.qpos[m.jnt_qposadr[j.id]] += dq
    mujoco.mj_forward(m, d)
    z1 = min(float(d.xpos[b][2]) for b in feet)
    d.qpos[2] -= (z1 - z0)              # bring the soles back to the ground
    mujoco.mj_forward(m, d)
    print("[squat] knee=%.1f deg  base_z=%.3f m  CoM_z=%.3f m"
          % (np.degrees(2 * alpha), d.qpos[2],
             float(d.subtree_com[m.body("base_link").id][2])))


class DCMWalkT(B.DCMWalk):
    """Event-driven footstep state machine + adaptive timing on top of DCMWalk."""

    def __init__(self, m, d, t_nom, t_min, t_max, verbose=True):
        super().__init__(m, d)
        # NB: if apply_squat() was called beforehand, zc and omega (measured by
        # the base class on the bent pose) are already consistent.
        self.T_nom, self.T_MIN, self.T_MAX = t_nom, t_min, t_max
        self.xi_s = self.ly * np.tanh(self.omega * t_nom / 2.0)
        self.use_timing = True
        # --- LEVER L1 (--sagfb): sagittal adjustment of the step LOCATION, coupled
        # to the adaptive duration Tk. OFF BY DEFAULT -> S2 (frozen config, batch N=50)
        # and S5 (230 trials) remain BIT-IDENTICAL.
        # Motivation: audit_timing_vs_reactive_planners.md -> the June law does
        # have the duration law (Khadiv2020) but NO sagittal adjustment of the
        # location and no location<->duration coupling; S5 measures sagittal I50
        # 20-30 N.s vs lateral 45-50+ = signature of a sagittal channel with no active mechanism.
        # b_sag = sagittal DCM offset of the nominal cycle's fixed point:
        #   b * e^{omega*T} = STEP_LEN + b   =>   b = L / (e^{omega*T} - 1)
        # At nominal regime u_x = xi_pred_x - b_sag == plan: the lever is
        # a NO-OP as long as the sagittal DCM does not deviate (low regression risk).
        self.sag_fb = False
        self.sag_dev = 0.06            # ecart max autorise vs plan (m)
        self.b_sag = float(B.STEP_LEN / (np.exp(self.omega * t_nom) - 1.0))
        self.verbose = verbose
        self.k = 0; self._last_k = -1
        self.t0 = None                 # start of the current step (None = transfer)
        self.Tk = t_nom                # (adaptive) duration of the current step
        self.swing_pos = None
        self.ended = None              # time of entry into the END phase
        # final target: last stance + closing foot side by side
        self.y0 = 0.5 * (float(d.xpos[self.left][1]) + float(d.xpos[self.right][1]))
        last = self.zmp[-1]
        close_y = self.ly if self.support[-1] == self.right else -self.ly
        self.closing = np.array([last[0], close_y])
        self.end_target = 0.5 * (last + self.closing)
        self.log = dict(com_err=[], xi_err=[], ctrl_ms=[], t_steps=[],
                        land_wobble=[], land_grf=[], land_mkbrk=[])
        # timing-side soft-landing mechanisms (opt-in; sl_imp/sl_ramp are in the base class)
        self.sl_ease = False           # decelerated descent before impact
        self.sl_debounce = False       # contact confirmed over DEBOUNCE_N ticks before switching
        # state for landing metrics + contact debounce
        self._cnt_touch = 0
        self._lw_foot = None; self._lw_t0 = 0.0
        self._lw_w = 0.0; self._lw_grf = 0.0; self._lw_mb = 0; self._lw_prev = True

    def _foot_contact(self, fb):
        """Contact of body fb with the GROUND (worldbody geoms, body 0) — stays
        robust to a tilted foot (toe touching, ankle raised).
        v4 2026-07-08: previously, ANY contact of the body counted (foot-foot,
        foot-leg) -> phantom touchdowns at dz=+0.03..0.046 observed in run70h."""
        d = self.d
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                return True
        return False

    def _grf_z(self, fb):
        """Sum of foot-GROUND contact normal forces (N) on body fb."""
        d = self.d; tot = 0.0; f6 = np.zeros(6)
        for i in range(d.ncon):
            c = d.contact[i]
            b1 = self.m.geom_bodyid[c.geom1]; b2 = self.m.geom_bodyid[c.geom2]
            if (b1 == fb and b2 == 0) or (b2 == fb and b1 == 0):
                mujoco.mj_contactForce(self.m, d, i, f6)
                tot += abs(float(f6[0]))            # normal component (contact frame)
        return tot

    def _land_metrics_tick(self):
        """LAND_WIN window after each touchdown: peak angular velocity of the foot
        (wobble), peak GRF (impact), make/break transitions (bounce).
        Logged WITH and WITHOUT --softland -> direct A/B comparison for Ch5/Ch6."""
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
        """Lateral distance DCM->stance k, positive toward the inside of the robot.
        The stance side comes from support[k] (L/R), NOT from the sign of its
        world position — which flips as soon as the robot drifts laterally."""
        p_y = self.zmp[k][1]
        zeta = 1.0 if self.support[k] == self.left else -1.0
        return (xi_y - p_y) * (-zeta)

    # ---------- footstep state machine ----------
    def update(self, dt):
        self.t += dt
        com, com_v, xi_meas = self._measured_dcm()
        self._land_metrics_tick()                    # landing metrics (all phases)

        # ---- E1: event-driven weight transfer (replaces fixed SETTLE) ----
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
                    tag = "TIMEOUT — forced start" if self.t > XFER_TMAX else "entry condition reached"
                    print("[gait] start t=%.2f s (%s: dlat=%.3f, entry=%.3f)"
                          % (self.t, tag, d0, entry))
            return

        tau = self.t - self.t0
        k = self.k

        # ---- E2/E3: touchdown event = actual CONTACT ----
        # As soon as the foot touches during the landing window, we switch
        # the stance IMMEDIATELY: otherwise the foot on the ground stays outside
        # the contact set and gets dragged around by the still-active swing tasks.
        if self.ended is None:
            foot_z = float(self.d.xpos[self.swing][2])
            raw = (foot_z <= self.fz + FOOT_TOL) or self._foot_contact(self.swing)
            # debounce (--softland): a micro-bounce must not make the
            # DS/SS contact set oscillate between two ticks
            self._cnt_touch = self._cnt_touch + 1 if raw else 0
            touched = self._cnt_touch >= (DEBOUNCE_N if self.sl_debounce else 1)
            due = (tau >= self.Tk) and (touched or tau >= self.T_MAX)
            early = bool(getattr(self, "_landing", False)) and touched
            if due or early:
                if self.swing_pos is not None:
                    self.swing_from[self.swing] = self.swing_pos.copy()
                self.log["t_steps"].append(min(tau, self.T_MAX))
                # start of the contact ramp (--softland) + metrics window
                self.land_t[self.swing] = self.t
                self._flush_land_metrics()           # previous window not closed (step < 100 ms)
                self._lw_foot = self.swing; self._lw_t0 = self.t
                self._lw_w = 0.0; self._lw_grf = 0.0; self._lw_mb = 0; self._lw_prev = True
                self._cnt_touch = 0
                if self.verbose:
                    dl = self._dlat(xi_meas[1], k)
                    print("[step %2d] touchdown %s  T=%.2fs  in=%.3f out=%.3f (d*=%.3f)  foot_dz=%+.3f"
                          % (k, "G" if self.swing == self.left else "D", tau,
                             getattr(self, "d_in", -1.0), dl,
                             self.ly + self.xi_s, foot_z - self.fz))
                self._landing = False
                self._force_land = False
                if k + 1 < self.N:
                    # align THE ENTIRE REMAINING PLAN to reality: the next
                    # stance becomes the foot that actually touched down, and
                    # future steps (+ the plan's nominal geometry) are shifted
                    # by the same amount — no more targets in a phantom frame
                    p_real = self.d.xpos[self.swing]
                    sx = float(p_real[0]) - self.zmp[k + 1][0]
                    self.zmp[k + 1][0] = float(p_real[0])
                    self.zmp[k + 1][1] = float(p_real[1])
                    # future steps: x shifts onto reality, but y RE-CENTERS
                    # toward the starting line (<= 2 cm/step) -> walks
                    # in a straight line despite recovery drift
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
                    # final target = midpoint of the ACTUAL feet (not the plan)
                    pl = self.d.xpos[self.left][:2]; pr = self.d.xpos[self.right][:2]
                    self.end_target = 0.5 * (np.asarray(pl) + np.asarray(pr))

        # ---- final phase: static balance between the two feet ----
        if self.ended is not None:
            self.phase = "DS"; self.stance = self.support[-1]
            self.swing = self.left if self.stance == self.right else self.right
            self.com_ref_xy = self.com_ref_xy + \
                (self.end_target - self.com_ref_xy) * min(dt / 0.3, 1.0)
            self.xi_ref = self.end_target.copy()
            self.log["xi_err"].append(float(np.linalg.norm(xi_meas - self.xi_ref)))
            self.log["com_err"].append(float(np.linalg.norm(com - self.com_ref_xy)))
            return

        # ---- start of step: placement decision (capture point, base) ----
        if k != self._last_k:
            if self.use_foot_fb:
                delta = self.k_foot * (xi_meas - self.xi_ini[k])
                if self.foot_lat_only: delta[0] = 0.0
                self.set_footstep_offset(delta)
            self._apply_offset(k)                        # + _recompute_dcm()
            self._last_k = k
            # per-step replanning: the lateral reference restarts from
            # the MEASURED DCM entry (consistency between timing rule <-> reference)
            d0 = self._dlat(xi_meas[1], k)
            self.d_in = float(np.clip(d0, 0.010, 0.9 * (self.ly + self.xi_s)))
            self._force_land = False
            # RECOVERY mode: degenerate entry (DCM on the wrong side or
            # saturated) -> the next step is purely LATERAL, forward
            # progress stops until the cycle is recovered
            self._recover = (d0 <= 0.005) or (d0 >= 0.85 * (self.ly + self.xi_s))
            if self._recover and self.verbose:
                print("[recov] k=%d degenerate entry (d0=%+.3f) -> pure lateral step" % (k, d0))

        self.stance = self.support[k]
        self.swing = self.left if self.stance == self.right else self.right
        s = min(max(tau / self.Tk, 0.0), 1.0)

        # ---- E2: adaptive timing (lateral channel, single support) ----
        # Track the CURRENT estimate of the switching time: a DCM that
        # runs away advances touchdown; a well-behaved DCM can extend up to T_MAX.
        if self.use_timing and s >= B.DS_OVL:
            dlat = self._dlat(xi_meas[1], k)
            dstar = self.ly + self.xi_s
            dsag = xi_meas[0] - self.zmp[k][0]
            # BOTH channels are predictive: touchdown happens when the first of the
            # two DCM components (lateral or sagittal) reaches its switching value
            if dlat > 1e-4:
                t_lat_rem = np.log(max(dstar / dlat, 1e-9)) / self.omega
            else:
                # DCM on the wrong side of the stance -> touch down IMMEDIATELY
                # (and lower the foot right away, without going back through the arc)
                t_lat_rem = 0.0
                self._force_land = True
            if dsag > 1e-4:
                t_sag_rem = np.log(max(SAG_MAX / dsag, 1e-9)) / self.omega
            else:
                t_sag_rem = 1e9
            t_rem = min(t_lat_rem, max(t_sag_rem, 0.0))
            self.Tk = float(np.clip(tau + max(t_rem, 0.0), self.T_MIN, self.T_MAX))

        # ---- DCM reference: x = recursion (virtual time), y = limit cycle ----
        tv = s * self.T_nom                              # sagittal virtual time
        xi_x = self.zmp[k][0] + (self.xi_ini[k][0] - self.zmp[k][0]) * \
            np.exp(self.omega * tv)
        p_y = self.zmp[k][1]
        zeta = 1.0 if self.support[k] == self.left else -1.0
        d_in = getattr(self, "d_in", self.ly - self.xi_s)
        xi_y = p_y + (-zeta) * d_in * np.exp(self.omega * min(tau, self.Tk))
        xi = np.array([xi_x, xi_y])
        self.xi_ref = xi.copy()

        # ---- DCM feedback -> STABLE CoM reference (form validated in June) ----
        if self.use_dcm_fb:
            e = np.clip(xi - xi_meas, -self.fb_clip, self.fb_clip)
            xi_cmd = xi + self.k_dcm * e
        else:
            xi_cmd = xi
        self.com_ref_xy = self.com_ref_xy + dt * self.omega * (xi_cmd - self.com_ref_xy)

        # ---- swing foot (variable duration handled by s = tau/Tk) ----
        if k + 1 < self.N:
            goal = np.array([self.zmp[k+1][0], self.zmp[k+1][1], self.fz])
        else:
            goal = np.array([self.closing[0], self.closing[1], self.fz])
        if getattr(self, "_recover", False):
            goal[0] = self.zmp[k][0]          # recovery: purely lateral step,
                                              # no sagittal progress
        # phase: step ends in DS ONLY if the foot is actually down —
        # never a no-slip constraint on a foot in the air
        foot_z = float(self.d.xpos[self.swing][2])
        landed = (foot_z <= self.fz + FOOT_TOL) or self._foot_contact(self.swing)
        if s < B.DS_OVL:
            self.phase = "DS"
        elif landed and s > 1 - B.DS_OVL:
            self.phase = "DS"
        else:
            self.phase = "SS"
        if self.use_cp_swing and self.phase == "SS":
            # place the foot BEYOND the measured DCM, offset by off_lat:
            # reproduces the cycle entry at the next step (Khadiv). off_lat >
            # nominal (ly - xi_s) to compensate for the double-support push
            # (ZMP between the feet -> the DCM drifts toward the new stance).
            sgn = 1.0 if self.swing == self.left else -1.0
            # target the PREDICTED DCM at touchdown time: during the foot's
            # descent (~0.15 s) the DCM keeps diverging (~x1.6) — targeting the
            # current DCM lands the foot "late", eating into the offset
            t_go = min(max(self.Tk - tau, 0.0), 0.30)
            p_yst = self.zmp[k][1]
            xi_pred = p_yst + (xi_meas[1] - p_yst) * np.exp(self.omega * t_go)
            off = getattr(self, "off_lat", self.ly - self.xi_s)
            if getattr(self, "_recover", False):
                off = max(off, 0.11)          # recovery: land further beyond the DCM
            gy = xi_pred + sgn * off
            # bounds RELATIVE TO THE ACTUAL STANCE FOOT: never less than min_sep
            # between the feet (anti-crossing), never more than MAX_SEP
            p_st = float(self.d.xpos[self.stance][1])
            msep = getattr(self, "min_sep", MIN_SEP)
            if self.swing == self.left:
                gy = min(max(gy, p_st + msep), p_st + MAX_SEP)
            else:
                gy = max(min(gy, p_st - msep), p_st - MAX_SEP)
            # ---- L1: sagittal adjustment of the step location (capture point) ----
            # u_x = xi_pred_x(t_go) - b_sag, with the SAME t_go as the lateral
            # channel -> the LOCATION is coupled to the adaptive DURATION Tk (this
            # is the location<->duration coupling absent from the June law).
            # Guardrails: (1) inactive in recovery mode (the pure lateral step
            # stays priority); (2) bounded to +-sag_dev around the plan = a
            # CORRECTION, not a replan; (3) never behind the current
            # stance -> monotonic sagittal progress.
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
            # LANDING requested: active descent, FROZEN target (no more DCM
            # tracking near the end of flight), swing task active until contact
            if not getattr(self, "_landing", False):
                self._landing = True
                self._land_goal = goal.copy()
            g = self._land_goal
            z_prev = self.swing_pos[2] if self.swing_pos is not None else self.fz + B.STEP_H
            vz = Z_LAND
            if self.sl_ease:
                # ease-in: the reference decelerates near the ground -> impact ~ZLAND_MIN
                # instead of ~Z_DESC (impact energy drops by a factor ~(0.25/0.10)^2)
                vz = float(np.clip(ZLAND_K * (foot_z - self.fz), ZLAND_MIN, Z_LAND))
            self.swing_pos = np.array([g[0], g[1], max(self.fz, z_prev - vz * dt)])
        else:
            frm = self.swing_from[self.swing]
            self.swing_pos = (1 - s) * frm + s * goal
            self.swing_pos[2] = self.fz + B.STEP_H * np.sin(np.pi * s)

        # ---- debug: timing state every ~0.1 s ----
        if getattr(self, "debug", False):
            self._dbg = getattr(self, "_dbg", 0) + 1
            if self._dbg % 50 == 0:
                print("[dbg] k=%d tau=%.2f Tk=%.2f s=%.2f dlat=%+.3f dsag=%+.3f dz=%+.3f %s%s"
                      % (k, tau, self.Tk, s, self._dlat(xi_meas[1], k),
                         xi_meas[0] - self.zmp[k][0],
                         float(self.d.xpos[self.swing][2]) - self.fz, self.phase,
                         " LAND" if getattr(self, "_landing", False) else ""))

        # ---- metrics ----
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
    ap.add_argument("--tstep", type=float, default=0.5, help="nominal step duration (s)")
    ap.add_argument("--tmin", type=float, default=0.30)
    ap.add_argument("--tmax", type=float, default=None, help="defaut: 1.6 x tstep")
    ap.add_argument("--dsovl", type=float, default=0.25)
    ap.add_argument("--offlat", type=float, default=0.05,
                    help="lateral touchdown offset beyond the predicted DCM (m); nominal ly-xi_s~0.028, increased to compensate for double support")
    ap.add_argument("--minsep", type=float, default=0.14,
                    help="minimum lateral separation between the two foot centres (m)")
    ap.add_argument("--sagfb", action="store_true",
                    help="LEVER L1: sagittal adjustment of the foothold (u_x = xi_pred_x - b_sag) "
                         "coupled to the adaptive duration Tk. OFF by default -> frozen S2 config and "
                         "S5 campaign bit-identical. A/B: --sagfb against without, same seeds.")
    ap.add_argument("--sagdev", type=float, default=0.06,
                    help="L1: maximum sagittal departure allowed from the plan (m) - bounds the correction")
    ap.add_argument("--sagsub", action="store_true",
                    help="L1 in SUBSTITUTION mode: disables the sagittal term of footfb "
                         "(foot_lat_only=True) so that L1 REPLACES the existing sagittal loop "
                         "instead of superposing on it. Without this flag the two laws fight: "
                         "footfb pushes the target forward (once per step), L1 pulls it back to the "
                         "capture point (continuously) -> systematic braking, N=50 batch of 2026-07-20 = 0/50.")
    ap.add_argument("--bsagk", type=float, default=1.0,
                    help="L1 : facteur de CALIBRATION de b_sag. b_sag theorique (LIPM ideal) "
                         "OVERESTIMATES the sagittal DCM offset of the real gait by ~26 %% (measured: real "
                         "offset 0.0415 m against LIPM fixed point 0.0563 m) -> the lever brakes at every "
                         "step (median correction -17 mm on an 80 mm step). 0.74 = calibrated.")
    ap.add_argument("--kdcm", type=float, default=1.0)
    ap.add_argument("--kfoot", type=float, default=1.0)
    ap.add_argument("--offmax", type=float, default=0.20)
    ap.add_argument("--dist", type=float, default=3.0, help="distance cible S2 (m)")
    ap.add_argument("--zcdrop", type=float, default=0.04,
                    help="initial knee flexion by lowering (m) the starting CONFIGURATION (0 = straight legs)")
    ap.add_argument("--wfoot", type=float, default=6000.0,
                    help="weight of the flat-sole task on the swing foot (base: 1500)")
    ap.add_argument("--wfootst", type=float, default=500.0,
                    help="flat-sole weight on the STANCE foot (complement to the hard constraint)")
    ap.add_argument("--softland", action="store_true",
                    help="poser doux COTE IMPACT (anti-vacillement cheville) : descente deceleree, "
                         "contact debounce, scheduled orientation impedance - the load transfer "
                         "(hard constraints, FZ_MIN) stays INSTANTANEOUS as in the baseline")
    ap.add_argument("--sl-ramp", dest="slramp", action="store_true",
                    help="EXPERIMENTAL : rampe de transfert de charge (rotation dure differee + "
                         "FZ_MIN ramp). Desynchronises the transfer from the instantaneous-switch ZMP plan "
                         "-> oscillation periode-2 (FELL k=7 v1, k=11 v2, 2026-07-08). Ablation Ch6.")
    ap.add_argument("--tramp", type=float, default=0.06,
                    help="duration of the ramp / impedance window (s)")
    ap.add_argument("--zlk", type=float, default=15.0,
                    help="ease-in: descent gain v=K*h (1/s); deceleration below zlmin/K metres")
    ap.add_argument("--zlmin", type=float, default=0.18,
                    help="ease-in : vitesse plancher d'impact (m/s)")
    ap.add_argument("--floortc", type=float, default=0.0,
                    help="softens the GROUND contact: solref timeconst (s), e.g. 0.02; 0 = default "
                         "model. Contact-model sensitivity test (Ch3 par.3.5) - "
                         "independant de --softland")
    ap.add_argument("--no-timing", dest="timing", action="store_false")
    ap.add_argument("--no-feedback", dest="feedback", action="store_false")
    ap.add_argument("--no-footfb", dest="footfb", action="store_false")
    ap.add_argument("--no-cpswing", dest="cpswing", action="store_false")
    ap.add_argument("--no-footori", dest="footori", action="store_false")
    ap.add_argument("--limit-cycle", dest="limit_cycle", action="store_true",
                    help="initial qvel injection (optional - event-based start is normally sufficient)")
    ap.add_argument("--debug", action="store_true", help="internal timing trace every ~0.1 s")
    ap.add_argument("--seed", type=int, default=None, help="bruit initial qvel (essais batch)")
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--viewer", action="store_true", help="viewer MuJoCo temps reel (relance auto si chute)")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()
    if a.tmax is None: a.tmax = 1.6 * a.tstep
    global ZLAND_K, ZLAND_MIN
    ZLAND_K, ZLAND_MIN = a.zlk, a.zlmin
    # gait parameters via the base module's globals (established convention)
    B.N_STEPS = a.steps; B.STEP_LEN = a.steplen; B.T_STEP = a.tstep; B.DS_OVL = a.dsovl
    B.STEP_H = 0.04     # lower swing arc: less distance to descend at touchdown
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0   # stiff ankles (900/90 + wfootst 1500
                                          # tested 2026-07-06: FELL — steals the ankle
                                          # torque needed for the ZMP)
    B.KP_SW, B.KD_SW = 420.0, 42.0       # faster swing: large lateral
                                          # targets reachable in < 0.3 s

    m = mujoco.MjModel.from_xml_path(W.MODEL); d = mujoco.MjData(m)
    if a.floortc > 0:                     # softer ground: solref timeconst of the worldbody geoms
        nfl = 0
        for g in range(m.ngeom):
            if m.geom_bodyid[g] == 0:
                m.geom_solref[g][0] = a.floortc; nfl += 1
        print("[floor] solref timeconst=%.3f s applied to %d ground geom(s)" % (a.floortc, nfl))
    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
    if a.seed is not None:
        rng = np.random.default_rng(a.seed)
        d.qvel[:] += rng.normal(0.0, 0.01, m.nv)
        mujoco.mj_forward(m, d)
    x0 = float(d.subtree_com[m.body("base_link").id][0])
    y0m = float(d.subtree_com[m.body("base_link").id][1])

    def build(d_):
        apply_squat(m, d_, a.zcdrop)     # bend the knees BEFORE building the WBC
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
        c_.sag_fb = a.sagfb                  # LEVER L1 (default False = frozen config)
        c_.sag_dev = a.sagdev
        c_.b_sag *= a.bsagk                  # calibration of the fixed point on the ACTUAL gait
        if a.sagsub: c_.foot_lat_only = True # L1 replaces (instead of doubling up) the sagittal footfb
        c_.sl_ease = c_.sl_debounce = c_.sl_imp = a.softland
        c_.sl_ramp = a.slramp
        c_.T_RAMP = a.tramp
        c_.debug = a.debug
        if a.limit_cycle: B.init_limit_cycle(c_, d_)
        return c_

    c = build(d)
    dt = m.opt.timestep

    # ---------- real-time viewer (same pattern as view_talos_dcm.py) ----------
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
                if d.qpos[2] < 0.5:                      # fall -> auto restart
                    print("[viewer] fall at t=%.2f s (step k=%d) — restarting" % (c.t, c.k))
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

    # ---------- S2 summary ----------
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
    print("  lever L1     : sagfb=%s%s" %
          (a.sagfb, ("  (b_sag=%.4f m [k=%.2f], dev max %.3f m)" % (c.b_sag, a.bsagk, a.sagdev)) if a.sagfb else ""))
    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, step k=%d)" % (fell_at, c.k)))
    print("  distance     : %.2f m (target %.1f) -> %s | lateral drift %.2f m" %
          (dist, a.dist, "SUCCESS" if success else "FAIL", float(com[1] - y0m)))
    print("  steps reached: k=%d / %d | steps taken=%d" % (c.k, a.steps - 1, len(ts)))
    if len(ts):
        print("  step duration: mean %.2f s | min %.2f | max %.2f (adaptive)" %
              (ts.mean(), ts.min(), ts.max()))
    print("  CoM err (xy) : RMSE %.1f mm | max %.1f mm" %
          (np.sqrt(np.nanmean(ce**2)) * 1e3, np.nanmax(ce) * 1e3))
    print("  DCM err      : mean %.1f mm | max %.1f mm" %
          (np.nanmean(xe) * 1e3, np.nanmax(xe) * 1e3))
    lw = np.array(c.log["land_wobble"]); lg = np.array(c.log["land_grf"])
    lmb = np.array(c.log["land_mkbrk"], dtype=float)
    if len(lw):
        print("  touchdown 100 ms: |w_foot| peak %.2f rad/s (mean %.2f) | GRF peak %.0f N (mean %.0f)"
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
