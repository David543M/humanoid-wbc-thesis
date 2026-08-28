"""
TALOS — S3: STAIR CLIMBING via a QUASI-STATIC SEQUENCER.

Motivation (see Ch5 §5.6): the flat-ground
DCM walker (talos_dcm_walk_timing) relies on EVENT-DRIVEN TIMING (adaptive
touchdown, capture-point, recovery) which turned out to be FUNDAMENTALLY
INCOMPATIBLE with the large, deliberate weight transfers of a staircase:
the timing shortens the step to T_MIN as soon as the DCM drifts, ending the
mount before the body has shifted onto the step.

This file ABANDONS that timing. It keeps ONLY the validated QP-WBC
executor (DCMWalk.control(): hard rigid-body dynamics, friction cone,
torque limits, CoM / orientation / posture / foot tasks; PROXQP) and
drives an EXPLICIT quasi-static STATE MACHINE:

    SETTLE                     : both feet on the ground, CoM centered, posture established
    for each step of the plan:
      TRANSFER (double support) : both feet BEAR load (hard contact in the QP),
                                 the CoM shifts (x,y) onto the STANCE foot +
                                 the CoM height rises to the stance step,
                                 until convergence (no race against a
                                 clock: the transfer is awaited).
      SWING   (single support)  : ONLY the stance foot bears load; the free
                                 foot follows an UP-OVER-DOWN arc toward the
                                 center of the pinned tread; the CoM stays over the stance.
      -> touchdown (contact) -> next step.
    DONE                       : final balance on the landing.

The key point: in DS, control() puts BOTH feet in the
contact set (hard no-slip constraint on each) -> the lateral+vertical
weight transfer happens on two feet, which the flat-ground walker lacked.
No step clock can interrupt the transfer: the DS->SS transition
only happens once the CoM has actually shifted.

Usage (from pal_talos/, conda base):
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
    """Quasi-static stair-climbing sequencer layered on top of the QP executor."""

    def __init__(self, m, d, risers, tread, max_mount=0.28,
                 rate=0.5, zrate=0.4, clear=0.14, tswing=0.8,
                 t_settle=0.8, t_tr_min=0.4, t_tr_max=1.6, postol=0.05):
        B.N_STEPS = 2; B.STEP_LEN = 0.1        # base class's flat plan: ignored
        super().__init__(m, d)                 # W.WBC + control() + feet/home/zc/omega
        self.dt = float(m.opt.timestep)
        self._risers = risers; self._tread = tread; self._max_mount = max_mount
        self.fz0 = float(self.fz)              # ankle height of the grounded foot (~0.104)
        self.zc0 = float(self.zc)
        # --- executor flags (read by control()) ---
        self.use_hard_contact = True; self.use_foot_ori = True
        # w_foot_stance RAISED 500->1400 (POLISH #6a): the STANCE foot must stay
        # FULLY flat/planted during the swing. At 500 (vs swing 2500) the reaction of
        # the lifting foot makes the stance sole TIP onto an edge (rocking, the
        # foot "leaves the ground a little"). We stiffen the load-bearing foot task
        # (flat sole position+orientation) without touching the swing trajectory or the CoM (2500). Bonus:
        # on the top landing, a stiffer stance foot gives the base more support
        # against the pitch of the final junction (helps recovery at the top).
        self.w_foot_swing = 2500.0; self.w_foot_stance = 1400.0  # swing < CoM (balance takes priority)
        self.sl_imp = False; self.sl_ramp = False; self.T_RAMP = 0.06; self.land_t = {}
        self.use_dcm_fb = False
        # --- sequencer settings ---
        self.RATE = rate; self.ZRATE = zrate; self.CROSS_CLEAR = clear
        self.T_SWING = tswing; self.POS_TOL = postol
        self.T_SETTLE = t_settle; self.T_TR_MIN = t_tr_min; self.T_TR_MAX = t_tr_max
        self.VTOL = 0.03                       # max CoM speed to finish the transfer (m/s)
        # landing SEEK: if the arc finishes without contact, the foot is pressed toward
        # the tread (up to SEEK_MAX below the target) instead of switching to DONE with the foot airborne.
        self.SEEK_RATE = 0.10; self.SEEK_MAX = 0.06; self.T_SEEK_MAX = 1.6
        # FORWARD bias of the CoM target: the ankle is not at the center of the sole
        # (heel -0.125 / toe +0.075) and climbing tends to leave the CoM TRAILING
        # (tips backward at the top). We aim slightly toward the toe.
        self.COM_FWD = 0.05
        self._com_prev = self.d.subtree_com[self.base][:2].copy()
        self._com_speed = 0.0
        # --- physical foot state (current planted positions) ---
        self.foot_pos = {fb: self.d.xpos[fb].copy() for fb in self.feet}
        self._build_moves()
        # --- state machine ---
        self.state = "SETTLE"; self.t = 0.0; self.t_state = 0.0; self.mi = -1
        self.phase = "DS"; self.stance = self.right; self.swing = self.left
        self.swing_pos = self.d.xpos[self.left].copy()
        self.com_ref_xy = self.d.subtree_com[self.base][:2].copy()
        self.swing_from = self.foot_pos[self.left].copy()
        self.target = self.foot_pos[self.left].copy()
        self._maxclimb = 0.0
        self.log = dict(clear=[], grf=[], ctrl_ms=[])
        self._swmin = None; self._swact = False

    # ---------- stair geometry ----------
    def _sh(self, x):
        h = 0.0
        for xe, z in self._risers:
            if x >= xe - 1e-6:
                h = z
        return h

    def _build_moves(self):
        """Sequence of foot moves: small-step approach (foot clamped
        before the 1st riser) then step-together climb (center of the tread)."""
        ly = self.ly; tread = self._tread; risers = self._risers
        yof = lambda side: ly if side == self.left else -ly
        rx = float(self.d.xpos[self.right][0])
        foots = []                              # (side, x); index 0 = initial stance (not a move)
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
        self._n_approach = len(foots)           # number of approach footholds (incl. initial)
        for t in range(len(risers)):
            ctr = risers[t][0] + 0.5 * tread
            side = self.left if side == self.right else self.right; foots.append((side, ctr))
            side = self.left if side == self.right else self.right; foots.append((side, ctr))
        self.moves = []
        for (side, x) in foots[1:]:
            self.moves.append((side, np.array([x, yof(side), self.fz0 + self._sh(x)])))
        print("[plan-QS] %d moves (initial stance R@x=%.3f):" % (len(self.moves), rx))
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
        """UP-OVER-DOWN ankle trajectory: rises vertically above the start,
        advances at peak height (sole above the nose), descends onto the tread."""
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

    # ---------- state machine ----------
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
            st_live = np.asarray(self.d.xpos[self.stance][:2], float)   # LIVE foot
            tgt = np.array([st_live[0], st_live[1]])                    # CoM over the live stance
            self._approach(tgt, self.zc0 + self._sh(float(self.d.xpos[self.stance][0])))
            self.swing_pos = self.foot_pos[self.swing].copy()
            tau = self.t - self.t_state
            com = self.d.subtree_com[self.base][:2]
            com_ok = np.linalg.norm(com - tgt) < self.POS_TOL
            settled = self._com_speed < self.VTOL      # CoM STOPPED over the stance (anti-coast)
            if (tau > self.T_TR_MIN and com_ok and settled) or tau > self.T_TR_MAX:
                self.state = "SWING"; self.t_state = self.t

        elif st == "SWING":
            self.phase = "SS"
            tau = self.t - self.t_state
            s = min(tau / self.T_SWING, 1.0)
            self.swing_pos = self._arc(self.swing_from, self.target, s)
            st_live = self.d.xpos[self.stance][:2]
            # SINGLE SUPPORT: CoM strictly above the load-bearing foot (no forward bias).
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
            # FINAL SETTLE — recovery confined to DONE (moves 0-7 = clean 3/3 climb,
            # untouched). The last swing foot (m08) may NOT have landed: on the
            # top landing the pelvis pitches and the world-frame target is missed (GRF=0, foot
            # airborne) -> the old version tipped backward onto a SINGLE foot.
            # Two stages:
            #   (1) free foot NOT down -> PRESS it toward the tread (seek z) AND hold
            #       the CoM firmly over the STANCE foot (already on the landing) + forward
            #       bias: this UN-pitches the base and brings the foot down to contact;
            #   (2) both feet on the ground -> CoM at the center of the top support polygon.
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
        for fb in self.feet:                    # GRF tracking (peak)
            self.log["grf"].append(self._grf_z(fb))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--x0", type=float, default=0.30)
    ap.add_argument("--tread", type=float, default=0.28)
    ap.add_argument("--hriser", type=float, default=0.10)
    ap.add_argument("--maxmount", type=float, default=0.28)
    ap.add_argument("--rate", type=float, default=0.18, help="CoM xy reference speed (m/s) - slow = the CoM follows without overshoot")
    ap.add_argument("--zrate", type=float, default=0.35, help="vitesse rampe hauteur CoM (m/s)")
    ap.add_argument("--clear", type=float, default=0.14, help="swing clearance above the nosing (m)")
    ap.add_argument("--tswing", type=float, default=0.65, help="swing arc duration (s) - 0.65 = clean 3/3 climb; LONGER (1.0) reintroduces the lateral drift in single support (QS #5b regression)")
    ap.add_argument("--tsettle", type=float, default=0.8)
    ap.add_argument("--ttrmin", type=float, default=0.5)
    ap.add_argument("--ttrmax", type=float, default=3.0, help="maximum transfer time (waits for the CoM to stop)")
    ap.add_argument("--postol", type=float, default=0.035, help="CoM-to-support tolerance for completing the transfer (m)")
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
    print("[scene] scene_stairs.xml generated: x0=%.2f tread=%.2f h=%.2f n=%d"
          % (a.x0, a.tread, a.hriser, len(risers)))

    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0
    B.KP_SW, B.KD_SW = 420.0, 42.0
    # CoM = PRIORITY in single support. Trace #1 showed that the swing-foot
    # tasks (W_SWING/w_foot_swing = 6000) OVERWHELM the CoM task (400) -> the CoM drifts
    # off the load-bearing foot (half-sole ±0.06 m) and the inverted pendulum (omega~3.3/s)
    # diverges. The CoM is given authority comparable to the swing + stiffened/damped gains,
    # and the swing weight is lowered so it doesn't steal the balance.
    B.KP_COM_W, B.KD_COM_W, B.W_COM_W = 500.0, 180.0, 2500.0
    B.W_SWING = 2500.0
    # FLAT PELVIS: the default W_ORI (40) is negligible vs CoM/swing (2500) -> the
    # pelvis pitches during the large climbing swings, which OVERSHOOTS the
    # swing foot past its target (collision, cf trace stair-3). We stiffen it heavily.
    # 600 = the value of the CLEAN 3/3 climb (QS #5). Raising to 1100 did NOT help and
    # coincided with the lateral regression (QS #5b) -> 600 is kept. The transient
    # -19 deg pitch of the swing-junction m08 is caught by SEEK (the foot LANDS) +
    # two-foot DONE recovery, not by a global orientation stiffening.
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
                    print("[viewer] fall t=%.2f s (%s) — restarting" % (c.t, c.state))
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
            if a.trace: print("[trc] ===== FALL ====="); trc(c)
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
    print("S3-QS  stairs x0=%.2f tread=%.2f h=%.2f | moves=%d rate=%.2f tswing=%.2f seed=%s"
          % (a.x0, a.tread, a.hriser, len(c.moves), a.rate, a.tswing, a.seed))
    print("  outcome    : %s%s | final state=%s"
          % ("UPRIGHT" if upright else "FELL",
             "" if fell_at is None else "  (t=%.2f s)" % fell_at, c.state))
    print("  climb      : %d/%d steps (max stance %.2f m) | CoM z gain %.3f m -> %s"
          % (n_climbed, len(risers), c._maxclimb, climb, "SUCCESS" if success else "FAIL"))
    print("  x progress : %.2f m | move reached mi=%d/%d" % (dist, c.mi, len(c.moves) - 1))
    print("  clearance  : min %.3f m | mean %.3f m | crossings=%d"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clear"])))
    print("  GRF (pic)  : %.0f N | mean %.0f N" % (np.nanmax(gr), np.nanmean(gr)))
    print("  ctrl loop  : mean %.2f ms | p99 %.2f ms | QP feas %.1f%%"
          % (np.nanmean(cm), np.nanpercentile(cm, 99), 100.0 * (1 - nq / ntick)))
    print("=" * 70)
    if a.save:
        # NB: 2026-08-06 addition of REPORTING keys for s3_batch.py. Post-loop block,
        # after the simulation ends -> strictly inert with respect to the dynamics
        # and the controller (no line of control()/update() touched). The frozen config
        # of the 3/3 success (QS #5c + w_foot_stance=1400) remains intact.
        np.savez(a.save, clear=cl, grf=gr, ctrl_ms=cm, dist=dist, climb=climb,
                 e_mech=c.E_mech, e_sq=c.E_sq, mass=c.mass,
                 n_climbed=n_climbed, max_climb=c._maxclimb, success=success,
                 fell_at=fell_at or -1.0, hriser=a.hriser,
                 upright=upright, reached_top=reached_top, state=c.state,
                 mi=c.mi, n_moves=len(c.moves) - 1, n_risers=len(risers),
                 qp_fail=nq, n_ticks=ntick)
        print("[ok] %s" % a.save)


if __name__ == "__main__":
    main()
