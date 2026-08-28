"""
TALOS — S3: STAIR CLIMBING (3 steps) via a PLATEAU-BASED LIPM approach.

See s3_design.md. The event-driven S2 walker (talos_dcm_walk_timing.DCMWalkT)
assumes flat ground: scalar foot height `self.fz` and CoM height `self.zc`
frozen (omega=sqrt(g/zc) computed once). This file lifts the flat-ground
assumption WITHOUT touching the core (talos_wbc, talos_dcm_walk,
talos_dcm_walk_timing UNCHANGED) via a subclass:

  1. `self.fz` becomes a PROPERTY returning the height of the TARGET step of
     the swing foot (stair_height(x_target)). All the swing, ground clearance
     and touchdown detection logic in DCMWalkT.update() uses it as-is
     -> correct swing arc and touchdown on every step, without
     reimplementing update().
  2. `self.zc` (vertical CoM reference, read in DCMWalk.control() line 219)
     is RAMPED UP toward the height of the STANCE step -> the CoM actually
     gains altitude instead of crouching. omega stays fixed (quasi-static
     plateau, cf. s3_design.md par.3; omega_k = future-work refinement).
  3. NEW metric `foot clearance`: minimum gap sole<->surface under the
     foot during flight (negative = collision with the step nosing).

The DCM footstep plan stays 2D (xy): the stairs only add the z component.

Stair profile = must stay in sync with scene_stairs.xml.

Usage (from pal_talos/, conda base with mujoco):
    python talos_s3_stairs.py                    # deterministic bring-up
    python talos_s3_stairs.py --viewer           # MuJoCo viewer (restarts on fall)
    python talos_s3_stairs.py --seed 0 --save s3_run.npz
    python talos_s3_stairs.py --hriser 0.05      # design sweep
"""
import argparse, time, numpy as np, mujoco
import talos_wbc as W
import talos_dcm_walk as B
import talos_dcm_walk_timing as T

# --- stair profile: (front_x, top_z) ---
# The REFERENCE profile (make_stair_height) and the physical SCENE
# (build_stairs_xml) are derived from the SAME parameters (x0, tread, h_riser, n),
# which guarantees their consistency regardless of the sweep.
EPS = 1e-6              # edge tolerance (avoids floating-point artifacts at step noses)
PLATFORM = 0.40        # depth of the landing platform (m)


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
        # beyond the landing platform, drop back to ground level (safety: the robot
        # must not exceed top_x, otherwise the step disappears from under the foot)
        return h if x <= top_x + EPS else 0.0
    return stair_height


def build_stairs_xml(x0=0.30, tread=0.28, h=0.10, n=3):
    """Generates the MuJoCo scene (n BOX steps) from the same parameters as
    the reference profile -> physics and reference always stay in sync."""
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
    """Stair climbing via plateaus, layered on top of the event-driven S2 walker."""
    _stairs = None          # class default: disabled (self.fz == flat ground) until
                            # the stair config is set at the end of __init__

    def __init__(self, m, d, stair_height, risers, tread, max_mount=0.32,
                 zc_rate=0.6, **kw):
        super().__init__(m, d, **kw)        # DCMWalk.__init__ does self.fz = p0[...]
                                            #  -> goes through the setter -> _fz0
        self._stairs = None                 # flat while rebuilding the plan
        self.zc0 = self.zc                  # CoM height on the ground (reference)
        # ankle->sole offset (for the clearance metric)
        self._ankle_off = float(d.xpos[self.left][2]) - self._fz0
        self.ZC_RATE = zc_rate              # vertical CoM ramp rate (m/s)
        # ADAPTIVE ground clearance: flat = frozen S2 value (0.04, must NOT break the
        # valid limit cycle); riser crossing = increased clearance.
        # NB: self.fz = height of the TARGET step already lifts the entire swing arc to
        # the upper level -> STEP_H only needs to cover the PD lag above the nose,
        # NOT the riser height (1st-draft bug: 0.14 flat everywhere -> fall on flat ground).
        self.FLAT_CLEAR = 0.04
        self.CROSS_CLEAR = 0.10             # overridable via --steph
        self.COM_RATE = 0.9                 # CoM co-drive rate while climbing (m/s)
        self.T_SWING_ARC = 0.45             # duration of the up-over-down arc in single support (s)
        self._climb_k = -1; self._ss_t0 = None
        self._risers = risers; self._tread = tread; self._max_mount = max_mount
        self._k_prev = -1; self._pin_k = -1
        self._fz_takeoff = self._fz0
        self.log["clearance"] = []
        self._sw_clear = None; self._sw_active = False
        self._max_climb = 0.0               # plus haute marche d'appui atteinte
        # STAIR-MODE: rebuild the footstep plan (small-step approach + step-together
        # climb = one foot steps onto the tread, the other joins it). The
        # stair footholds are PINNED by geometry (no drift replanning
        # like on flat ground) -> see re-pin in update().
        self._build_stair_plan()
        self._stairs = stair_height         # ACTIVATES the stair logic

    def _rebuild_xi(self):
        """DCM backward recursion: xi_ini at the start of each stance (identical to
        DCMWalk.__init__, recomputed on the current stair plan)."""
        E = np.exp(self.omega * B.T_STEP)
        xi_ini = [None] * self.N
        xi_end = self.zmp[-1].copy()
        for k in range(self.N - 1, -1, -1):
            xi_ini[k] = self.zmp[k] + (xi_end - self.zmp[k]) / E
            xi_end = xi_ini[k]
        self.xi_ini = xi_ini

    def _build_stair_plan(self):
        """Stair-mode footstep plan: flat approach then step-together climb.
        Replaces the base class's uniform 0.08 m plan."""
        d = self.d; ly = self.ly
        rx = float(d.xpos[self.right][0])
        risers = self._risers; tread = self._tread
        yof = lambda side: ly if side == self.left else -ly
        zmp = []; support = []
        side = self.right                              # initial stance = right foot
        zmp.append(np.array([rx, yof(side)])); support.append(side)
        x_cur = rx
        ctr0 = risers[0][0] + 0.5 * tread              # center of tread 1 (ankle target)
        # anti-collision: the TOE of the sole (ankle + TOE_EXT) of the last approach
        # foot must NOT go past the nose of the 1st riser, otherwise the foot
        # lands inside the step (the "overshoots" bug). The ankle is clamped accordingly.
        TOE_EXT = 0.075; MARGIN = 0.03
        x_app_max = risers[0][0] - TOE_EXT - MARGIN     # max ankle x of an approach foot
        # APPROACH: small flat steps until within climbing range of tread 1
        guard = 0
        while (ctr0 - x_cur) > self._max_mount and guard < 80:
            side = self.left if side == self.right else self.right
            x_cur = min(x_cur + B.STEP_LEN, x_app_max)
            zmp.append(np.array([x_cur, yof(side)])); support.append(side)
            guard += 1
            if x_cur >= x_app_max - 1e-9:              # anti-collision edge reached -> climb
                break
        self._n_approach = len(zmp)
        # CLIMB step-together: mount (step onto the tread) then join (the other foot follows)
        for t in range(len(risers)):
            ctr = risers[t][0] + 0.5 * tread
            side = self.left if side == self.right else self.right   # mount
            zmp.append(np.array([ctr, yof(side)])); support.append(side)
            side = self.left if side == self.right else self.right   # join
            zmp.append(np.array([ctr, yof(side)])); support.append(side)
        self.zmp = zmp; self.support = support; self.N = len(zmp)
        self._plan0 = [p.copy() for p in zmp]          # pinned plan (re-pin)
        self._rebuild_xi()
        last = self.zmp[-1]
        close_y = ly if self.support[-1] == self.right else -ly
        self.closing = np.array([last[0], close_y])
        self.end_target = 0.5 * (last + self.closing)
        # log the plan (targeted footholds)
        sh = self._stairs_probe
        print("[plan] %d footholds: approach %d small-steps + climb %d (step-together)"
              % (self.N, self._n_approach, self.N - self._n_approach))
        for i, (p, s) in enumerate(zip(self.zmp, self.support)):
            tag = "app" if i < self._n_approach else "CLIMB"
            print("   z%02d %s x=%.3f y=%+.3f z=%.2f  [%s]"
                  % (i, "G" if s == self.left else "D", p[0], p[1], sh(float(p[0])), tag))

    def _stairs_probe(self, x):
        """Stair height at point x (independent of self._stairs activation)."""
        h = 0.0
        for xe, z in self._risers:
            if x >= xe - EPS:
                h = z
        return h

    # ---- self.fz: height of the TARGET step of the swing foot ----
    @property
    def fz(self):
        if getattr(self, "_stairs", None) is None:
            return self._fz0                # flat ground (before activation / fallback)
        k = self.k
        gx = self.zmp[k + 1][0] if (k + 1 < self.N) else self.closing[0]
        return self._fz0 + self._stairs(float(gx))

    @fz.setter
    def fz(self, v):
        self._fz0 = float(v)                # reference height of the flat ground

    # ---- vertical CoM reference: ramps to follow the STANCE step ----
    def update(self, dt):
        # adaptive ground clearance: set BEFORE the base gait reads B.STEP_H.
        # crossing = the swing foot changes level (fz_land > fz_takeoff).
        # NB self.swing only exists after the 1st super().update() -> hasattr guard
        # (before the 1st step, no swing: STEP_H=0.04 default, correct).
        sw = getattr(self, "swing", None)
        if getattr(self, "_stairs", None) is not None and sw is not None:
            k = self.k
            if k != self._k_prev:           # new step: remember the takeoff height
                self._fz_takeoff = float(self.d.xpos[sw][2]) - self._ankle_off
                self._k_prev = k
            gx = self.zmp[k + 1][0] if (k + 1 < self.N) else self.closing[0]
            fz_land = self._fz0 + self._stairs(float(gx))
            crossing = (fz_land - self._fz_takeoff) > 1e-4
            B.STEP_H = self.CROSS_CLEAR if crossing else self.FLAT_CLEAR

        super().update(dt)                  # unchanged event-driven S2 gait
        if getattr(self, "_stairs", None) is None:
            return
        # RE-PIN: the base class shifts future footholds by the touchdown error (flat-
        # ground drift). On stairs the footholds are PINNED by geometry -> restore
        # the pinned plan for upcoming steps (the current stance keeps its actual touchdown).
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
        # STAIR-MODE (climb): co-drive the CoM QUASI-STATICALLY. The DCM plan alone
        # leaves the CoM trailing (~270 mm) behind the large footholds -> the leg
        # cannot reach the step -> it hits it. So we explicitly advance the
        # CoM (xy) reference over the stance, driving it toward the midpoint of stance->next
        # foothold with the step phase. Replaces the DCM reference on climbing steps.
        self._climb_com_codrive(dt)
        # STAIR-MODE: override the foot trajectory on CLIMB steps
        # (replaces the flat swing arc + the lateral-only recovery mode, both unsuitable).
        self._climb_swing_override()
        self._clearance_tick()

    def _climb_com_codrive(self, dt):
        """Quasi-static CoM reference during the climb: follows the stance foot and moves
        toward the midpoint of (stance, next foothold) according to the phase -> the body climbs
        WITH the feet instead of trailing. Slowing down (--tstep) = more quasi-static."""
        if self.ended is not None:
            return
        k = self.k
        # stair-mode active as soon as the step TARGET (zmp[k+1]) is a stair
        # foothold -> k+1 >= _n_approach (otherwise the 1st mount stays on the flat gait
        # = overshoot + hits the step: the observed "overshoots" indexing bug).
        if (k + 1) < self._n_approach or self.t0 is None:
            return
        # STRICT QUASI-STATIC: the CoM reference FOLLOWS THE STANCE FOOT (x AND y).
        # Trace #7 showed that targeting the MIDPOINT stance->next foothold pushed
        # the CoM AHEAD of the single foot on the ground during single support (CoM 0.44 vs stance
        # 0.19) -> forward tip-over (pitch +65, DCM_x->1.8, fall). By staying above
        # the stance foot, the CoM only advances once that (more advanced) foot becomes the stance
        # at the next step -> climbing with "CoM follows the stance", without pitching forward.
        st = np.asarray(self.d.xpos[self.stance][:2], dtype=float)
        tgt = np.array([float(st[0]), float(st[1])])
        rate = self.COM_RATE * dt
        self.com_ref_xy = self.com_ref_xy + np.clip(tgt - self.com_ref_xy, -rate, rate)
        self.xi_ref = self.com_ref_xy.copy()

    def _climb_swing_override(self):
        """Dedicated swing trajectory for step crossing: lifts first (peak at
        s~0.45, no jump at the start), advances next (delayed), descends onto the CENTER
        of the pinned tread. Phase indexed on T_nom (NOT on the adaptive timing which
        shortened the step -> short foot -> hits the riser). Also bypasses
        the base class's recovery mode (lateral-only = fatal on stairs)."""
        if self.ended is not None:
            return
        k = self.k
        # active as soon as the TARGET (zmp[k+1]) is a stair foothold; the 1st mount
        # is step k=_n_approach-1 (indexing bug fixed: "3rd step overshoots").
        if (k + 1) < self._n_approach or (k + 1) >= self.N:
            return
        # NB: GATING (holding the foot on the ground during DS) was REMOVED (stair-mode #9):
        # the flat gait's adaptive timing shortens the step to T_MIN when the DCM drifts,
        # ending the step WHILE holding -> mount never executes (0/3). Architectural
        # incompatibility documented (Ch5 §5.6 / Ch6). Config kept = "CoM follows the stance".
        if self.t0 is None or getattr(self, "phase", None) == "DS":
            return                                       # double support: handled by the base class
        frm = self.swing_from.get(self.swing) if hasattr(self.swing_from, "get") else None
        if frm is None:
            return
        land_xy = self._plan0[k + 1]
        land_z = self._fz0 + self._stairs(float(land_xy[0]))
        takeoff_z = float(frm[2])
        peak = max(takeoff_z, land_z) + self.CROSS_CLEAR
        s = min(max((self.t - self.t0) / self.T_nom, 0.0), 1.0)
        # UP-OVER-DOWN profile: rise VERTICALLY above the takeoff point (phase 1),
        # advance AT PEAK HEIGHT (phase 2, the sole is already above the nose),
        # descend onto the target tread (phase 3). Horizontal motion happens
        # ONLY at peak height -> the sole no longer scrapes the riser (scraping bug).
        # NB self.swing_pos is the ankle; the sole is ~_ankle lower, the CROSS_CLEAR
        # clearance (peak) covers this offset + the PD lag.
        LIFT, FWD = 0.30, 0.72
        if s < LIFT:                                     # 1) rise vertically
            xfrac = 0.0
            z = takeoff_z + (peak - takeoff_z) * np.sin(np.pi / 2 * (s / LIFT))
        elif s < FWD:                                    # 2) advance at peak height
            u = (s - LIFT) / (FWD - LIFT)
            xfrac = u * u * (3.0 - 2.0 * u)              # horizontal smootherstep
            z = peak
        else:                                            # 3) descend onto the tread
            xfrac = 1.0
            u = (s - FWD) / (1.0 - FWD)
            z = land_z + (peak - land_z) * np.cos(np.pi / 2 * u)
        x = float(frm[0]) + (float(land_xy[0]) - float(frm[0])) * xfrac
        y = float(frm[1]) + (float(land_xy[1]) - float(frm[1])) * xfrac
        self.swing_pos = np.array([x, y, z])

    def _clearance_tick(self):
        """Minimum gap sole<->surface directly under the foot during flight."""
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
    """Pelvis pitch (deg): + = leaning FORWARD (pitching forward)."""
    import math
    w, x, y, z = float(d.qpos[3]), float(d.qpos[4]), float(d.qpos[5]), float(d.qpos[6])
    return math.degrees(math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x)))))


def trace_row(m, d, c):
    """Detailed state snapshot (1 trace row)."""
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
    # --- stair geometry (must stay in sync with scene_stairs.xml) ---
    ap.add_argument("--x0", type=float, default=0.30, help="x of the first step nosing (m)")
    ap.add_argument("--tread", type=float, default=0.28, help="giron (m)")
    ap.add_argument("--hriser", type=float, default=0.10, help="contremarche (m) — balayage {0.05,0.10,0.15}")
    ap.add_argument("--zcrate", type=float, default=0.6, help="vertical CoM ramp speed (m/s)")
    ap.add_argument("--maxmount", type=float, default=0.28,
                    help="maximum climbing step allowed (m): the approach ends when the centre of tread 1 is within reach")
    ap.add_argument("--sagmax", type=float, default=0.50,
                    help="sagittal capture SAG_MAX (m): widened against S2 (0.13) to allow large climbing steps")
    ap.add_argument("--comrate", type=float, default=0.6,
                    help="quasi-static CoM co-driving speed while climbing (m/s); raise it if the CoM lags")
    # --- gait: defaults = frozen S2 config; tstep increased for large steps ---
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
                    help="swing-foot ground clearance on the CROSSING step (m); "
                         "flat ground keeps the frozen S2 value 0.04 (adaptive per step). "
                         "NB self.fz already raises the bell to the upper level: steph only covers the PD lag above the nosing")
    ap.add_argument("--no-timing", dest="timing", action="store_false")
    ap.add_argument("--no-feedback", dest="feedback", action="store_false")
    ap.add_argument("--no-footfb", dest="footfb", action="store_false")
    ap.add_argument("--no-cpswing", dest="cpswing", action="store_false")
    ap.add_argument("--no-footori", dest="footori", action="store_false")
    ap.add_argument("--debug", action="store_true")
    ap.add_argument("--trace", action="store_true",
                    help="DETAILED per-tick LOG -> s3_trace.csv + compact console trace during the climb")
    ap.add_argument("--tracefile", type=str, default="s3_trace.csv")
    ap.add_argument("--traceevery", type=int, default=40,
                    help="period (ticks) of console trace printing during the climb")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--video", action="store_true")
    ap.add_argument("--viewer", action="store_true")
    ap.add_argument("--save", type=str, default=None)
    a = ap.parse_args()
    if a.tmax is None: a.tmax = 1.6 * a.tstep

    risers = make_risers(a.x0, a.tread, a.hriser)
    top_x = top_x_of(a.x0, a.tread, len(risers))
    stair_height = make_stair_height(risers, top_x)
    top_target = risers[-1][1]              # top height (= n*h_riser)
    # generate the physical scene from the SAME parameters (consistency guaranteed)
    with open("scene_stairs.xml", "w") as f:
        f.write(build_stairs_xml(a.x0, a.tread, a.hriser))
    print("[scene] scene_stairs.xml generated: x0=%.2f tread=%.2f h=%.2f n=%d (top_x=%.2f)"
          % (a.x0, a.tread, a.hriser, len(risers), top_x))

    # gait parameters via the base module's globals (established convention)
    B.N_STEPS = a.steps; B.STEP_LEN = a.steplen; B.T_STEP = a.tstep; B.DS_OVL = a.dsovl
    B.STEP_H = 0.04                          # flat clearance = frozen S2 value; adapted per step in DCMWalkS3.update()
    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0
    T.ZLAND_K, T.ZLAND_MIN = 15.0, 0.18
    T.SAG_MAX = a.sagmax                     # widened sagittal capture (large climbing steps)

    W.MODEL = "scene_stairs.xml"             # <-- stairs instead of flat ground
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
                    print("[viewer] fall t=%.2f s (step k=%d) — restarting" % (c.t, c.k))
                    time.sleep(0.6)
                    mujoco.mj_resetDataKeyframe(m, d, 0); mujoco.mj_forward(m, d)
                    c = build(d); start = time.time()
        return

    t_cap = 6.0 + c.N * a.tmax + T.T_END     # duration based on the length of the stair plan
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
                _trace_print(trace_row(m, d, c)); print("[trc] ===== FALL (base_z<0.6) =====")
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
        print("[ok] %s (%d lines)" % (a.tracefile, len(trace_rows)))

    # ---------- S3 summary ----------
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
    print("S3 STAIRS  steps=%d len=%.2f Tnom=%.2f [%.2f,%.2f] | stairs x0=%.2f tread=%.2f h=%.2f steph=%.2f seed=%s"
          % (a.steps, a.steplen, a.tstep, a.tmin, a.tmax, a.x0, a.tread, a.hriser, a.steph, a.seed))
    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, step k=%d)" % (fell_at, c.k)))
    print("  climb        : %d/%d steps (max stance %.2f m) | CoM z gain %.3f m -> %s"
          % (n_climbed, len(risers), c._max_climb, climb, "SUCCESS" if success else "FAIL"))
    print("  x progress   : %.2f m | steps taken=%d / %d" % (dist, len(ts), a.steps - 1))
    print("  CLEARANCE    : min %.3f m | mean %.3f m | crossings=%d%s"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clearance"]),
             "  ⚠ COLLISION" if (len(c.log["clearance"]) and np.nanmin(cl) < 0) else ""))
    print("  CoM err (xy) : RMSE %.1f mm | max %.1f mm"
          % (np.sqrt(np.nanmean(ce**2)) * 1e3, np.nanmax(ce) * 1e3))
    if len(c.log["land_grf"]):
        print("  landing GRF  : peak %.0f N | mean %.0f N" % (np.nanmax(lg), np.nanmean(lg)))
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
