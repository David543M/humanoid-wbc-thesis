"""
test_centroidal_mpc.py — validation HORS LIGNE de la couche MPC centroidale.

Le MPC pilote ici la dynamique centroidale NON LINEAIRE EXACTE :
    l_dot = sum f_i + m g
    k_dot = sum (p_i - c) x f_i          <-- vrai bras de levier, NON linearise
alors que le MPC optimise sur le modele LINEARISE (bras de levier evalue sur la
reference). L'erreur de linearisation est donc reellement exercee : c'est le
point de la validation.

Ce test ne valide PAS la marche : il valide la COUCHE DE PLANIFICATION sur son
propre modele. Aucun resultat de marche ne doit en etre deduit — la comparaison
avec S2 exige l'integration MuJoCo.

USAGE
    python test_centroidal_mpc.py                 # scenario nominal
    python test_centroidal_mpc.py --push 150      # impulsion laterale
    python test_centroidal_mpc.py --sweep         # dimensionnement horizon/temps
"""
import argparse
import numpy as np

from centroidal_mpc import CentroidalMPC, GaitSchedule, G

MASS = 95.0          # TALOS ~ 95 kg
ZC = 0.86


def in_support(cop_xy, pts, act, margin=1e-6):
    """CoP dans l'enveloppe des points actifs (test par boite englobante :
    suffisant ici, les appuis sont rectangulaires et alignes sur les axes)."""
    p = pts[act > 0.5]
    if p.size == 0:
        return True
    lo, hi = p[:, :2].min(axis=0) - margin, p[:, :2].max(axis=0) + margin
    return bool(np.all(cop_xy >= lo) and np.all(cop_xy <= hi))


def run(horizon=12, dt_mpc=0.04, dt_sim=0.002, duration=6.0,
        push_N=0.0, push_t=3.0, push_dur=0.1, push_axis=1, verbose=True):
    sched = GaitSchedule(step_len=0.08, t_step=0.50, ds_ratio=0.12, ly=0.085,
                         n_steps=70, t_settle=0.8)
    mpc = CentroidalMPC(MASS, sched, horizon=horizon, dt=dt_mpc, zc=ZC)

    # Demarrage SUR le cycle limite lateral (cf. centroidal_mpc.limit_cycle_state).
    # Sans cela la divergence est garantie, planificateur ou pas.
    x, t = sched.limit_cycle_state(MASS, ZC)
    duration = duration + t
    f = np.zeros((sched.n_contacts, 3))
    next_mpc = 0.0
    log = dict(t=[], c=[], cref=[], k=[], cop_ok=[], fric_ok=[], fz_min=[])

    n = int((duration - t) / dt_sim)
    act_prev = sched.contact_active(0.0)
    for _ in range(n):
        act_now = sched.contact_active(t)
        # re-resolution FORCEE au changement d'ensemble de contacts : sans cela le
        # bloqueur d'ordre zero maintient pendant jusqu'a dt_mpc des forces
        # calculees pour un pied qui vient de decoller (CoP hors polygone).
        switched = not np.array_equal(act_now, act_prev)
        if t >= next_mpc - 1e-12 or switched:
            out = mpc.solve(x, t)
            f = out["f_ref"]
            next_mpc = t + dt_mpc
        act_prev = act_now

        pts = sched.contact_points(t)
        act = act_now
        c, l, k = x[0:3], x[3:6], x[6:9]

        # --- dynamique centroidale EXACTE (bras de levier vrai) ---
        f_tot = f.sum(axis=0)
        ldot = f_tot + MASS * G
        if push_N and push_t <= t < push_t + push_dur:
            ldot = ldot + np.eye(3)[push_axis] * push_N
        kdot = np.zeros(3)
        for i in range(sched.n_contacts):
            kdot += np.cross(pts[i] - c, f[i])

        # --- diagnostics AVANT integration ---
        fz = f[:, 2]
        fric_ok = bool(np.all(np.abs(f[:, 0]) <= mpc.mu * fz + 1e-6) and
                       np.all(np.abs(f[:, 1]) <= mpc.mu * fz + 1e-6))
        s = fz.sum()
        cop = (pts[:, :2] * fz[:, None]).sum(axis=0) / s if s > 1e-6 else c[:2]
        log["t"].append(t); log["c"].append(c.copy())
        log["cref"].append(sched.com_reference(t, ZC)[0])
        log["k"].append(k.copy()); log["cop_ok"].append(in_support(cop, pts, act))
        log["fric_ok"].append(fric_ok); log["fz_min"].append(float(fz.min()))

        # --- integration semi-implicite ---
        x[3:6] = l + ldot * dt_sim
        x[6:9] = k + kdot * dt_sim
        x[0:3] = c + (x[3:6] / MASS) * dt_sim
        t += dt_sim
        if x[2] < 0.3 or not np.isfinite(x).all():
            if verbose:
                print("  [ABORT] divergence a t=%.2f s (z=%.3f)" % (t, x[2]))
            break

    for key in ("t", "c", "cref", "k", "fz_min"):
        log[key] = np.asarray(log[key])
    err = np.linalg.norm(log["c"][:, :2] - log["cref"][:, :2], axis=1)
    res = dict(
        t_end=float(log["t"][-1]), dist=float(log["c"][-1, 0] - log["c"][0, 0]),
        rmse_mm=float(np.sqrt(np.mean(err**2)) * 1e3),
        max_err_mm=float(err.max() * 1e3),
        z_dev_mm=float(np.abs(log["c"][:, 2] - ZC).max() * 1e3),
        k_max=float(np.abs(log["k"]).max()),
        cop_ok=float(np.mean(log["cop_ok"])), fric_ok=float(np.mean(log["fric_ok"])),
        fz_min=float(log["fz_min"].min()), mpc=mpc, log=log)
    return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--horizon", type=int, default=12)
    ap.add_argument("--dt", type=float, default=0.04)
    ap.add_argument("--dur", type=float, default=6.0)
    ap.add_argument("--push", type=float, default=0.0)
    ap.add_argument("--sweep", action="store_true")
    a = ap.parse_args()

    if a.sweep:
        print("=" * 78)
        print("DIMENSIONNEMENT — cible the thesis scope definition: MPC lineaire < 10 ms @ 50-100 Hz")
        print("=" * 78)
        print("%-6s %-7s %-8s %-9s %-10s %-9s %s"
              % ("N", "dt", "n_var", "mean ms", "p99 ms", "RMSE mm", "statuts"))
        for N, dtm in ((8, 0.05), (10, 0.05), (12, 0.04), (16, 0.04), (20, 0.03)):
            r = run(horizon=N, dt_mpc=dtm, duration=3.0, verbose=False)
            m = r["mpc"]; t = np.array(m.stats["t_ms"])
            print("%-6d %-7.3f %-8d %-9.2f %-10.2f %-9.1f %s"
                  % (N, dtm, N * m.nu, t.mean(), np.percentile(t, 99),
                     r["rmse_mm"], m.stats["statuses"]))
        return

    print("=" * 78)
    print("MPC CENTROIDAL — boucle fermee sur la dynamique NON LINEAIRE exacte")
    print("  horizon N=%d, dt_mpc=%.0f ms (%.1f s d'anticipation), duree %.1f s%s"
          % (a.horizon, a.dt * 1e3, a.horizon * a.dt, a.dur,
             "" if not a.push else "  | impulsion %.0f N laterale a t=3 s" % a.push))
    print("=" * 78)
    r = run(horizon=a.horizon, dt_mpc=a.dt, duration=a.dur, push_N=a.push)
    print(r["mpc"].report())
    print("-" * 78)
    print("  progression      : %.3f m en %.2f s" % (r["dist"], r["t_end"]))
    print("  suivi CoM (xy)   : RMSE %.1f mm | max %.1f mm" % (r["rmse_mm"], r["max_err_mm"]))
    print("  hauteur CoM      : deviation max %.1f mm  (le LIPM l'imposerait a 0)"
          % r["z_dev_mm"])
    print("  moment cinetique : |k|_max %.2f kg.m2/s  (le LIPM l'ignore)" % r["k_max"])
    print("  CoP dans l'appui : %.1f %% des pas de temps" % (100 * r["cop_ok"]))
    print("  cone de friction : %.1f %% | fz_min %.2f N (contrainte >= 0)"
          % (100 * r["fric_ok"], r["fz_min"]))
    print("=" * 78)
    ok = (r["cop_ok"] > 0.99 and r["fric_ok"] > 0.999 and r["fz_min"] > -1e-6
          and r["mpc"].stats["not_solved"] == 0)
    print("VERDICT : %s" % ("OK" if ok else "A INVESTIGUER"))


if __name__ == "__main__":
    main()
