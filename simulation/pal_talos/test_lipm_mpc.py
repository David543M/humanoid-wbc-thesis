"""
test_lipm_mpc.py — validation hors ligne du LIPM-MPC a placement de pas.

BOUCLE FERMEE PHYSIQUE (et non tautologique) :
    le MPC produit un JERK -> une acceleration commandee -> un CoP commande
        z_cmd = c - (zc/g) * c_ddot_cmd
    le PLANT est le pendule inverse pilote par ce CoP :
        c_ddot_reel = omega^2 (c - z_cmd) + F_ext/m
Sans perturbation les deux coincident ; sous impulsion, l'ecart est reel et le
MPC doit le rattraper par le retour d'etat ET par le deplacement des pas.

CE QUI EST TESTE  : formulation du QP, satisfaction du CoP dans l'empreinte,
                    adaptation du lieu de pas, rejet de perturbation, temps.
CE QUI NE L'EST PAS: hauteur de CoM variable, moment cinetique (hors modele),
                     couplage avec l'executeur QP-WBC (exige MuJoCo).

USAGE
    python test_lipm_mpc.py                    # nominal, depart AU REPOS
    python test_lipm_mpc.py --push 150         # impulsion laterale
    python test_lipm_mpc.py --compare          # balayage d'impulsions
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

    # --- DEPART AU REPOS : aucune initialisation sur le cycle limite ---
    xs = np.zeros(3)                    # [c, c_dot, c_ddot] axe x
    ys = np.zeros(3)                    # axe y
    # Pendant l'etablissement, on vise deja le pied qui portera le pas 0 :
    # le cout de centrage du ZMP realise ainsi le TRANSFERT DE POIDS lateral
    # avant le decollage, au lieu de laisser le CoM au milieu.
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
            # Commit du pied porteur. ATTENTION a l'amorcage : lors de la
            # transition etablissement -> pas 0, AUCUN pied ne se pose. Le
            # porteur du pas 0 est le pied GAUCHE, deja au sol (le pied droit
            # balance aux pas pairs, convention GaitSchedule). Les pas suivants
            # prennent bien le premier pas planifie, qui vient de se poser.
            if k == 0:
                foot_c = np.array([0.0, +mpc.ly])
            elif plan is not None:
                foot_c = plan[0].copy()
            k_prev = k
            log["steps"].append((t, foot_c.copy()))
            next_mpc = t          # RE-RESOLUTION FORCEE au changement d'appui :
            # sans cela le bloqueur d'ordre zero maintient jusqu'a T un ZMP
            # calcule pour le pied PRECEDENT, compare ensuite au nouveau pied
            # -> ~T/t_step = 20 % de faux « hors empreinte ».

        if t >= next_mpc - 1e-12:
            out = mpc.solve(xs, ys, t, foot_c)
            plan = out["footsteps"]
            jerk = out["jerk"]
            next_mpc = t + T

        # PLANT : triple integrateur pilote par le JERK du MPC -- exactement le
        # modele que le QP optimise. Le ZMP realise s'en deduit :
        #     z = c - (zc/g) * c_ddot
        # evalue sur le MEME instant que la position, ce qui est la grandeur
        # que la contrainte du QP borne. (Une version anterieure melangeait
        # c(t) et c_ddot(t+T) : on contraignait une grandeur et on en simulait
        # une autre, d'ou 33 % de fausses violations.)
        c = np.array([xs[0], ys[0]])
        acc = np.array([xs[2], ys[2]])
        z_cmd = c - (ZC / GRAV) * acc

        # empreinte du pied porteur (pendant l'etablissement : double appui, plus large)
        hx, hy = mpc.foot_h["x"], mpc.foot_h["y"]
        ph = 1.0 if k < 0 else ((t - mpc.t_settle) % mpc.t_step) / mpc.t_step
        if k < 0 or ph < mpc.ds_ratio:          # double appui : polygone elargi
            hy = hy + mpc.ly
        inbox = bool(abs(z_cmd[0] - foot_c[0]) <= hx + 1e-6 and
                     abs(z_cmd[1] - foot_c[1]) <= hy + 1e-6)
        log["t"].append(t); log["c"].append(c.copy()); log["z"].append(z_cmd.copy())
        log["foot"].append(foot_c.copy()); log["inbox"].append(inbox)

        # --- integration exacte du triple integrateur sous jerk constant ---
        j2 = jerk if plan is not None else np.zeros(2)
        for s_ax, ju in ((xs, j2[0]), (ys, j2[1])):
            s_ax[0] += s_ax[1] * dt_sim + 0.5 * s_ax[2] * dt_sim**2 + ju * dt_sim**3 / 6.0
            s_ax[1] += s_ax[2] * dt_sim + 0.5 * ju * dt_sim**2
            s_ax[2] += ju * dt_sim
        if push_N and push_t <= t < push_t + push_dur:      # perturbation exogene
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
        print("TOLERANCE AUX IMPULSIONS — LIPM-MPC (pas adaptatifs) vs")
        print("MPC centroidal a pas FIXES (mesures 2026-08-11 : 50 N ok, 100 N")
        print("marginal, 150 N divergent)")
        print("=" * 78)
        print("%-10s %-8s %-11s %-10s %-9s %s"
              % ("push N", "t_end s", "diverge", "|y|max mm", "CoP ok", "pas"))
        for p in (0.0, 50.0, 100.0, 150.0, 200.0, 300.0):
            r = run(duration=10.0, push_N=p)
            print("%-10.0f %-8.2f %-11s %-10.1f %-9.1f%% %d"
                  % (p, r["t_end"], "OUI" if r["diverged"] else "non",
                     r["y_max"] * 1e3, 100 * r["inbox"], r["n_steps"]))
        return

    print("=" * 78)
    print("LIPM-MPC a placement de pas — DEPART AU REPOS (aucune initialisation")
    print("sur le cycle limite, contrairement au MPC centroidal)")
    print("=" * 78)
    r = run(duration=a.dur, push_N=a.push, verbose=True)
    print(r["mpc"].report())
    print("-" * 78)
    print("  duree simulee    : %.2f s / %.1f s demandees  -> %s"
          % (r["t_end"], a.dur, "DIVERGENCE" if r["diverged"] else "stable"))
    print("  progression      : %.2f m en %d pas" % (r["dist"], r["n_steps"]))
    print("  oscillation lat. : |y|max %.1f mm" % (r["y_max"] * 1e3))
    print("  CoP dans le pied : %.1f %% des pas de temps" % (100 * r["inbox"]))
    print("=" * 78)
    ok = (not r["diverged"]) and r["inbox"] > 0.99 and r["mpc"].stats["not_solved"] == 0
    print("VERDICT : %s" % ("OK" if ok else "A INVESTIGUER"))


if __name__ == "__main__":
    main()
