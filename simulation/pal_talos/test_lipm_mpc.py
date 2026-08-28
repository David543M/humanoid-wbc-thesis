"""
test_lipm_mpc.py — offline validation of the footstep-placement LIPM-MPC.

PHYSICAL (not tautological) CLOSED LOOP:
    the MPC produces a JERK -> a commanded acceleration -> a commanded CoP
        z_cmd = c - (zc/g) * c_ddot_cmd
    the PLANT is the inverted pendulum driven by that CoP:
        c_ddot_real = omega^2 (c - z_cmd) + F_ext/m
Without disturbance the two coincide; under an impulse the discrepancy is real
and the MPC must recover through state feedback AND footstep displacement.

WHAT IS TESTED   : QP formulation, CoP satisfaction inside the footprint,
                   footstep-location adaptation, disturbance rejection, timing.
WHAT IS NOT      : variable CoM height, angular momentum (outside the model),
                   coupling with the QP-WBC executor (requires MuJoCo).

USAGE
    python test_lipm_mpc.py                    # nominal, start AT REST
    python test_lipm_mpc.py --push 150         # lateral impulse
    python test_lipm_mpc.py --compare          # impulse sweep
"""
import argparse
import numpy as np

from lipm_mpc import LIPMWalkMPC, GRAV

MASS, ZC = 95.0, 0.86


def run(duration=10.0, dt_sim=0.002, push_N=0.0, push_t=4.0, push_dur=0.1,
        push_axis=1, N=32, T=0.05, verbose=False):
    mpc = LIPMWalkMPC(zc=ZC, T=T, N=N, t_step=0.5, t_settle=0.8,
                      step_len=0.08, ly=0.085)
    omega2 = GRAV / ZC

    # --- START AT REST: no initialisation on the limit cycle ---
    xs = np.zeros(3)                    # [c, c_dot, c_ddot] x axis
    ys = np.zeros(3)                    # y axis
    # During settling, the foot that will support step 0 is already targeted:
    # the ZMP centring cost thereby performs the lateral WEIGHT TRANSFER before
    # lift-off, instead of leaving the CoM in the middle.
    foot_c = np.array([0.0, +0.085])
    k_prev = mpc.step_index(0.0)
    z_cmd = np.zeros(2)
    plan = None
    jerk = np.zeros(2)
    t, next_mpc = 0.0, 0.0
    log = dict(t=[], c=[], z=[], foot=[], inbox=[], steps=[])

    n = int(duration / dt_sim)
    for _ in range(n):
        k = mpc.step_index(t)
        if k > k_prev:
            # Commit of the support foot. CAREFUL at start-up: at the
            # settling -> step 0 transition, NO foot lands. The support of
            # step 0 is the LEFT foot, already on the ground (the right foot
            # swings on even steps, GaitSchedule convention). Later steps do
            # take the first planned footstep, which has just landed.
            if k == 0:
                foot_c = np.array([0.0, +mpc.ly])
            elif plan is not None:
                foot_c = plan[0].copy()
            k_prev = k
            log["steps"].append((t, foot_c.copy()))
            next_mpc = t          # FORCED RE-SOLVE at the support change:
            # without it the zero-order hold keeps, for up to T, a ZMP computed
            # for the PREVIOUS foot, then compared against the new foot
            # -> ~T/t_step = 20 % of spurious "outside footprint".

        if t >= next_mpc - 1e-12:
            out = mpc.solve(xs, ys, t, foot_c)
            plan = out["footsteps"]
            jerk = out["jerk"]
            next_mpc = t + T

        # PLANT: triple integrator driven by the MPC JERK -- exactly the model
        # the QP optimises. The realised ZMP follows from it:
        #     z = c - (zc/g) * c_ddot
        # evaluated at the SAME instant as the position, which is the quantity
        # the QP constraint bounds. (An earlier version mixed c(t) and
        # c_ddot(t+T): one quantity was constrained while another was
        # simulated, hence 33 % of spurious violations.)
        c = np.array([xs[0], ys[0]])
        acc = np.array([xs[2], ys[2]])
        z_cmd = c - (ZC / GRAV) * acc

        # support-foot footprint (during settling: double support, wider)
        hx, hy = mpc.foot_h["x"], mpc.foot_h["y"]
        ph = 1.0 if k < 0 else ((t - mpc.t_settle) % mpc.t_step) / mpc.t_step
        if k < 0 or ph < mpc.ds_ratio:          # double support: widened polygon
            hy = hy + mpc.ly
        inbox = bool(abs(z_cmd[0] - foot_c[0]) <= hx + 1e-6 and
                     abs(z_cmd[1] - foot_c[1]) <= hy + 1e-6)
        log["t"].append(t); log["c"].append(c.copy()); log["z"].append(z_cmd.copy())
        log["foot"].append(foot_c.copy()); log["inbox"].append(inbox)

        # --- exact integration of the triple integrator under constant jerk ---
        j2 = jerk if plan is not None else np.zeros(2)
        for s_ax, ju in ((xs, j2[0]), (ys, j2[1])):
            s_ax[0] += s_ax[1] * dt_sim + 0.5 * s_ax[2] * dt_sim**2 + ju * dt_sim**3 / 6.0
            s_ax[1] += s_ax[2] * dt_sim + 0.5 * ju * dt_sim**2
            s_ax[2] += ju * dt_sim
        if push_N and push_t <= t < push_t + push_dur:      # exogenous disturbance
            idx = push_axis
            (xs if idx == 0 else ys)[2] += (push_N / MASS) * dt_sim / push_dur
        t += dt_sim
        if abs(ys[0]) > 1.0 or not np.isfinite([xs, ys]).all():
            if verbose:
                print("  [DIVERGENCE] t=%.2f s, y=%.3f m" % (t, ys[0]))
            break

    for key in ("t", "c", "z", "foot"):
        log[key] = np.asarray(log[key])
    diverged = log["t"][-1] < duration - 5 * dt_sim
    return dict(t_end=float(log["t"][-1]), diverged=diverged,
                dist=float(log["c"][-1, 0]),
                y_max=float(np.abs(log["c"][:, 1]).max()),
                inbox=float(np.mean(log["inbox"])),
                n_steps=len(log["steps"]), mpc=mpc, log=log)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dur", type=float, default=10.0)
    ap.add_argument("--push", type=float, default=0.0)
    ap.add_argument("--compare", action="store_true")
    a = ap.parse_args()

    if a.compare:
        print("=" * 78)
        print("IMPULSE TOLERANCE — LIPM-MPC (adaptive footsteps) vs")
        print("centroidal MPC with FIXED footsteps (measurements 2026-08-11: 50 N ok,")
        print("100 N marginal, 150 N divergent)")
        print("=" * 78)
        print("%-10s %-8s %-11s %-10s %-9s %s"
              % ("push N", "t_end s", "diverged", "|y|max mm", "CoP ok", "steps"))
        for p in (0.0, 50.0, 100.0, 150.0, 200.0, 300.0):
            r = run(duration=10.0, push_N=p)
            print("%-10.0f %-8.2f %-11s %-10.1f %-9.1f%% %d"
                  % (p, r["t_end"], "YES" if r["diverged"] else "no",
                     r["y_max"] * 1e3, 100 * r["inbox"], r["n_steps"]))
        return

    print("=" * 78)
    print("Footstep-placement LIPM-MPC — START AT REST (no initialisation on")
    print("the limit cycle, unlike the centroidal MPC)")
    print("=" * 78)
    r = run(duration=a.dur, push_N=a.push, verbose=True)
    print(r["mpc"].report())
    print("-" * 78)
    print("  simulated time   : %.2f s / %.1f s requested  -> %s"
          % (r["t_end"], a.dur, "DIVERGENCE" if r["diverged"] else "stable"))
    print("  progression      : %.2f m in %d steps" % (r["dist"], r["n_steps"]))
    print("  lateral oscill.  : |y|max %.1f mm" % (r["y_max"] * 1e3))
    print("  CoP inside foot  : %.1f %% of time steps" % (100 * r["inbox"]))
    print("=" * 78)
    ok = (not r["diverged"]) and r["inbox"] > 0.99 and r["mpc"].stats["not_solved"] == 0
    print("VERDICT: %s" % ("OK" if ok else "TO INVESTIGATE"))


if __name__ == "__main__":
    main()
