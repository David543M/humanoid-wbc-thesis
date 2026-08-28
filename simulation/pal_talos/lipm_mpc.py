"""
lipm_mpc.py — MPC LIPM a horizon fuyant avec PLACEMENT DE PAS OPTIMISE.

Ecrit le 2026-08-11, apres centroidal_mpc.py. Formulation de Herdt et al.
(2010), « Online walking motion generation with automatic footstep placement ».

DEPENDANCES : numpy + proxsuite. Aucun MuJoCo, aucun import des fichiers geles.

===========================================================================
POURQUOI CE FICHIER EXISTE
===========================================================================
`centroidal_mpc.py` repond litteralement a « MPC centroidal » (moment lineaire
ET angulaire, hauteur de CoM libre) mais, a pas FIXES, sa seule autorite de
rejet est le CoP dans l'empreinte : il diverge au-dela de ~4-6 s et ne tolere
qu'une impulsion laterale de ~50 N (mesures 2026-08-11).

Le LIPM a une propriete que le modele centroidal n'a pas : le ZMP est une
fonction LINEAIRE de l'etat,
        z = c - (zc/g) * c_ddot
En prenant l'etat x = [c, c_dot, c_ddot] et le JERK comme commande, tout est
lineaire ET LES AXES x ET y SE DECOUPLENT. Le QP passe de 240 variables a
~19 PAR AXE. A ce prix, on peut ajouter les POSITIONS DE PAS comme variables
de decision -- exactement l'adaptation qui manque au MPC centroidal.

Le compromis est explicite : hauteur de CoM constante, moment cinetique nul.
Ce planificateur ne repond donc PAS au mot « centroidal » de la question de
recherche ; il repond a « horizon fuyant + contraintes explicites ».

===========================================================================
FORMULATION (par axe, horizon N, periode T)
===========================================================================
    x_{k+1} = A x_k + B u_k ,  u = jerk
    A = [[1, T, T^2/2], [0, 1, T], [0, 0, 1]] ,  B = [T^3/6, T^2/2, T]^T
    z_k = C x_k ,               C = [1, 0, -zc/g]

Forme condensee (matrices constantes, precalculees une fois) :
    Z = P_zs x_0 + P_zu U        (ZMP predit)
    V = P_vs x_0 + P_vu U        (vitesse predite)

Variables de decision : X = [ U (N) ; F (m) ]  ->  N + m ~ 19
    U = jerks, F = positions des m prochains pas sur cet axe.

Cout :
    a/2 ||U||^2                      regularisation du jerk
  + b/2 ||V - V_ref||^2              suivi de vitesse (la consigne de marche)
  + c/2 ||Z - Vc*f_c - Vf*F||^2      ZMP centre dans le pied porteur
  + d/2 ||F - F_nom||^2              regularite de la foulee

Contraintes DURES :
  (1) ZMP dans l'empreinte :  -h <= Z - Vc*f_c - Vf*F <= h
  (2) foulee sagittale :      |F_j - F_{j-1}| <= stride_max
  (3) ecartement lateral :    F_j - F_{j-1} dans [+min_sep, +max_sep] ou
                              [-max_sep, -min_sep] selon le pied -- c'est ce
                              qui interdit le croisement de jambes.

STATUT DU SOLVEUR lu a chaque appel (cf. audit ProxQP du 2026-08-11).

===========================================================================
RESULTATS DE VALIDATION HORS LIGNE  (2026-08-11, test_lipm_mpc.py)
===========================================================================
Protocole : depart AU REPOS, sans initialisation sur le cycle limite (le MPC
centroidal, lui, ne demarre pas sans elle). Reglage retenu : T=0.05 s, N=32.

ETABLI, ET REPRODUCTIBLE
  * 15 pas sur 8 s SANS DIVERGENCE, depart a l'arret ; oscillation laterale
    |y|max 62 mm (a comparer a l'ecartement ly = 85 mm) ;
  * 100 % des QP converges, statut verifie a chaque appel ;
  * temps de resolution DEUX AXES : 0.45 ms (0.17 ms a T=0.1 s), contre
    4.8 ms pour le MPC centroidal -- ~19x moins cher, pour ~35 variables par
    axe au lieu de 240 ;
  * TOLERANCE AUX IMPULSIONS laterales, excursion post-poussee :
        0 N -> 49 mm | 100 N -> 52 | 200 N -> 81 | 300 N -> 244 | 400 N -> 688
    contre le MPC centroidal A PAS FIXES : 50 N absorbee, 100 N marginale,
    150 N DIVERGENTE. L'adaptation du LIEU de pas vaut donc environ un
    facteur 4 sur la marge de perturbation. C'est LE resultat de ce fichier.

NON ETABLI -- ne pas presenter comme acquis
  * LA CONTRAINTE ZMP N'EST PAS PROPREMENT SATISFAITE. Le taux mesure de
    « CoP dans l'empreinte » varie de 65 % a 86 % selon la maniere dont on
    compte la phase de double appui. La definition de la mesure a change
    trois fois au cours de la mise au point, donc CE CHIFFRE N'EST PAS
    FIABLE et l'ecart (slack) est actif de facon non quantifiee. Un MPC dont
    la contrainte principale n'est pas verifiee ne peut pas etre declare
    fonctionnel.
  * derive laterale residuelle de quelques centimetres sur 8 s ;
  * vitesse 10-15 % au-dessus de la consigne ;
  * PLANT = LIPM nominal + perturbation : hors poussee, le modele optimise
    et le modele simule coincident. Validation strictement plus faible que
    celle du MPC centroidal, qui tourne contre la dynamique centroidale non
    lineaire exacte ;
  * aucun couplage avec l'executeur QP-WBC (exige MuJoCo, non teste).

HISTORIQUE DE MISE AU POINT -- cinq defauts trouves par la MESURE, aucun par
le raisonnement : engagement du pied porteur decale d'un pas ; polygone de
double appui trop etroit a l'amorcage ; consigne de vitesse active pendant
l'etablissement (le CoM partait devant le pied d'appui) ; contrainte ZMP dure
sans variable d'ecart, qui envoyait ProxQP a 10 000 iterations ; et surtout
un PLANT incoherent avec le QP -- on contraignait z(t+T) et on simulait
c(t) - (zc/g) c_ddot(t+T). Ce dernier point a lui seul expliquait l'essentiel
des violations sagittales.

CONCLUSION HONNETE. Ce fichier etablit une COMPARAISON -- l'adaptativite du
plan domine la richesse du modele dans le regime teste (marche plane, faible
vitesse). Il n'etablit pas un marcheur.
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
    """MPC LIPM a horizon fuyant, placement de pas en variables de decision."""

    def __init__(self, zc=0.86, T=0.1, N=16, t_step=0.5, t_settle=0.8,
                 step_len=0.08, ly=0.085,
                 foot_hx=0.085, foot_hy=0.045,
                 stride_max=0.30, min_sep=0.11, max_sep=0.42, ds_ratio=0.12,
                 w_jerk=1e-5, w_vel=1.0, w_zmp=1e-2, w_step=10.0, w_slack=1e4,
                 k_ypos=2.0, v_lat_max=0.15,
                 eps_abs=1e-6, max_iter=400):
        if not _HAVE_PROXQP:
            raise SystemExit("[ABORT] proxsuite requis : pip install proxsuite")
        self.zc, self.T, self.N = float(zc), float(T), int(N)
        self.t_step, self.t_settle = float(t_step), float(t_settle)
        self.step_len, self.ly = float(step_len), float(ly)
        self.foot_h = {"x": float(foot_hx), "y": float(foot_hy)}
        self.stride_max, self.min_sep, self.max_sep = stride_max, min_sep, max_sep
        self.ds_ratio = float(ds_ratio)
        self.w = dict(jerk=w_jerk, vel=w_vel, zmp=w_zmp, step=w_step, slack=w_slack)
        self.eps_abs = eps_abs; self.max_iter = int(max_iter)
        self.k_ypos, self.v_lat_max = float(k_ypos), float(v_lat_max)
        self.m = int(np.ceil(self.N * self.T / self.t_step)) + 1   # pas futurs vus
        self._build_prediction()
        self._qp = {}
        self.stats = dict(calls=0, solved=0, not_solved=0, statuses={}, t_ms=[])

    # ------------------------------------------------------------------
    def _build_prediction(self):
        """Matrices de prediction condensees (constantes)."""
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
        """Qui porte a chaque echantillon de l'horizon ?

        Retourne (Vc, Vf, k0, side0) :
          Vc (N,)   1 si l'echantillon est porte par le pied ACTUEL (connu),
          Vf (N,m)  one-hot du pas FUTUR porteur (variable de decision),
          k0        indice du pas courant, side0 le pied porteur courant
                    (+1 = gauche, -1 = droit).
        """
        N, T = self.N, self.T
        Vc = np.zeros(N); Vf = np.zeros((N, self.m))
        k0 = self.step_index(t)
        for k in range(N):
            j = self.step_index(t + (k + 1) * T)
            if j <= k0:
                Vc[k] = 1.0                            # pied actuel, position connue
            else:
                Vf[k, min(j - k0 - 1, self.m - 1)] = 1.0   # pas futur, variable
        # convention GaitSchedule : le pied DROIT balance aux pas pairs
        # -> appui GAUCHE (+1) aux pas pairs, DROIT (-1) aux impairs
        side0 = +1.0 if (max(k0, 0) % 2 == 0) else -1.0
        return Vc, Vf, k0, side0

    def step_index(self, t):
        """-1 pendant l'etablissement initial, puis 0, 1, 2, ..."""
        if t < self.t_settle:
            return -1
        return int((t - self.t_settle) // self.t_step)

    # ------------------------------------------------------------------
    def _solve_axis(self, axis, x0, Vc, Vf, f_c, F_nom, v_ref, sides, h_vec):
        """Un QP pour un axe. X = [U (N) ; F (m)].

        h_vec (N,) : demi-empreinte AUTORISEE au ZMP a chaque echantillon. Elle
        n'est pas constante : pendant le double appui initial le polygone est
        celui des DEUX pieds, nettement plus large que celui d'un pied seul.
        Imposer l'empreinte d'un pied unique des le depart force un demarrage
        violent et fait diverger le premier pas.
        """
        N, m = self.N, self.m
        n = N + m + N                        # U | F | S (ecarts sur le ZMP)
        Pzs, Pzu, Pvs, Pvu = self.P_zs, self.P_zu, self.P_vs, self.P_vu
        Z = np.zeros((N, N))

        z0 = Pzs @ x0                       # partie ZMP independante de U
        v0 = Pvs @ x0
        # --- residus lineaires ---
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

        # --- contraintes ---
        rows, lo, up = [], [], []
        # (1) ZMP dans l'empreinte, RELACHEE par un ecart s_k >= 0 fortement penalise.
        #     Un MPC a contraintes ZMP DURES et duree de pas figee devient
        #     infaisable des qu'une perturbation depasse l'autorite du CoP : le
        #     solveur part alors a la limite d'iterations. L'ecart rend le QP
        #     toujours faisable et rend la violation MESURABLE au lieu de fatale.
        for k in range(N):
            a = Azmp[k].copy(); a[N + m + k] = -1.0        # ... - s_k <= h
            rows.append(a); lo.append(-1e20); up.append(h_vec[k] - bzmp[k])
            b = -Azmp[k].copy(); b[N + m + k] = -1.0       # -(...) - s_k <= h
            rows.append(b); lo.append(-1e20); up.append(h_vec[k] + bzmp[k])
            c = np.zeros(n); c[N + m + k] = 1.0            # s_k >= 0
            rows.append(c); lo.append(0.0); up.append(1e20)
        # (2) foulee sagittale / (3) ecartement lateral, entre pas consecutifs.
        # La ligne exprime  F_j - F_{j-1}  (ou F_0 - f_c pour j = 0) ; le terme
        # constant f_c passe donc au second membre pour j = 0 uniquement.
        for j in range(m):
            r = np.zeros(n); r[N + j] = 1.0
            offset = f_c if j == 0 else 0.0
            if j > 0:
                r[N + j - 1] = -1.0
            if axis == "x":
                d_lo, d_hi = -self.stride_max, +self.stride_max
            else:
                s = sides[j]                     # pied qui se pose : +1 gauche, -1 droit
                d_lo, d_hi = (s * self.min_sep, s * self.max_sep)
                if s < 0:                        # ordonner les bornes
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
        """xs, ys : etats [c, c_dot, c_ddot]. foot_c : (fx, fy) du pied porteur.

        Retourne dict(jerk, footsteps, zmp_pred, status, solve_ms).
        """
        Vc, Vf, k0, side0 = self._selectors(t)
        # plan nominal des m prochains pas
        Fx_nom, Fy_nom, sides = [], [], []
        for j in range(self.m):
            Fx_nom.append(foot_c[0] + (j + 1) * self.step_len)
            s = -side0 if (j % 2 == 0) else side0      # alternance stricte
            sides.append(s)
            Fy_nom.append(s * self.ly)
        Fx_nom = np.array(Fx_nom); Fy_nom = np.array(Fy_nom)

        # demi-empreinte par echantillon : polygone des DEUX pieds tant que
        # l'echantillon tombe dans le double appui initial
        # Echantillons en DOUBLE APPUI : etablissement initial, ET debut de
        # chaque pas (recouvrement ds_ratio, comme le marcheur S2). Sans cette
        # phase, le ZMP devrait sauter d'un pied a l'autre en passant par le
        # milieu, qui n'appartient a aucune des deux empreintes : violation
        # laterale structurelle a chaque transition (mesure : 19 % des ticks).
        def _is_ds(tt):
            if tt < self.t_settle:
                return 1.0
            ph = ((tt - self.t_settle) % self.t_step) / self.t_step
            return 1.0 if ph < self.ds_ratio else 0.0
        in_ds = np.array([_is_ds(t + (k + 1) * self.T) for k in range(self.N)])
        hx_vec = self.foot_h["x"] * np.ones(self.N)
        hy_vec = self.foot_h["y"] + in_ds * self.ly

        # Consigne de vitesse NULLE pendant l'etablissement : commander la
        # vitesse de croisiere des t=0 fait partir le CoM en avant du pied
        # d'appui avant le premier pas ; le ZMP sature alors a l'avant de
        # l'empreinte et le marcheur tombe vers l'avant en foulees maximales.
        v_ref_x = (self.step_len / self.t_step) * (1.0 - in_ds)
        Xx, stx, msx = self._solve_axis("x", xs, Vc, Vf, foot_c[0], Fx_nom,
                                        v_ref_x, sides, hx_vec)
        # Boucle externe de POSITION laterale. Un MPC de Herdt suit une VITESSE ;
        # une consigne laterale nulle est satisfaite par n'importe quel decalage
        # constant, et la marche derive alors en diagonale (mesure : -0.08 a
        # -0.23 m sur 8 s, insensible a w_step). On referme donc la position par
        # un terme proportionnel, saturé pour ne jamais dominer le cycle lateral.
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
        return ("LIPM-MPC : %d appels | %d resolus | %d NON resolus\n"
                "           solve (2 axes) mean %.3f ms | p99 %.3f ms | max %.3f ms\n"
                "           statuts %s"
                % (s["calls"], s["solved"], s["not_solved"],
                   np.nanmean(t), np.nanpercentile(t, 99), np.nanmax(t),
                   s["statuses"]))
