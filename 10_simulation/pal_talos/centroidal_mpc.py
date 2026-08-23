"""
centroidal_mpc.py — Couche de planification MPC CENTROIDALE CONVEXE (S2bis).

Ecrit le 2026-08-11. Repond litteralement a la couche annoncee dans la question
de recherche (Ch1 l.168) et specifiee en Ch3 §3.3, jamais implementee jusqu'ici.

DEPENDANCES : numpy + proxsuite UNIQUEMENT. Aucun import de MuJoCo, aucun import
des fichiers S2/S3/S4 geles -> ce module ne peut pas regresser les campagnes
existantes. Il est validable et valide HORS LIGNE (test_centroidal_mpc.py).

===========================================================================
1. MODELE
===========================================================================
Dynamique centroidale EXACTE (Orin et al. 2013), etat
    x = [ c (3) ; l (3) ; k (3) ]
avec c la position du CoM, l = m*c_dot le moment lineaire, k le moment
cinetique centroidal (autour du CoM) :

    c_dot = l / m
    l_dot = sum_i f_i + m*g
    k_dot = sum_i (p_i - c) x f_i                       <-- BILINEAIRE en (c, f)

Le seul terme non convexe est le bras de levier (p_i - c). On le linearise
autour d'une trajectoire de reference c_hat(t) connue a l'avance
(Di Carlo et al. 2018, « Dynamic Locomotion in the MIT Cheetah 3 via Convex
Model-Predictive Control ») :

    k_dot ~ sum_i (p_i - c_hat_k) x f_i

Le probleme devient un QP CONVEXE en les forces de contact. C'est la
difference de fond avec le LIPM : ni hauteur de CoM constante, ni moment
cinetique nul. Les deux hypotheses du LIPM sont levees.

===========================================================================
2. DISCRETISATION (exacte, pas d'approximation supplementaire)
===========================================================================
A est nilpotente (A^2 = 0) car seule la ligne c depend de l. Donc
    A_d = I + A*dt                       exact
    B_d = B*dt + A*B*dt^2/2              exact
    g_d = [ g*dt^2/2 ; m*g*dt ; 0 ]      exact

===========================================================================
3. QP CONDENSE
===========================================================================
Variables : U = [u_0 ... u_{N-1}], u_k = forces empilees aux N_c points.
    min  sum_k ||x_k - x_k^ref||^2_Q + ||u_k||^2_R   (+ terminal Q_N)
    s.c. 0 <= f_z,i <= f_max * s_ik        (s_ik = ordonnancement des contacts)
         |f_x,i| <= mu * f_z,i , |f_y,i| <= mu * f_z,i    (pyramide, 4 facettes)

L'unilateralite par COIN de pied contraint implicitement le CoP au polygone
d'appui : c'est la meme construction que l'executeur QP-WBC, donc les deux
couches partagent le meme modele de contact (« inter-layer compatibility »
revendiquee en Ch3 §3.4).

===========================================================================
4. SORTIES POUR L'EXECUTEUR QP-WBC
===========================================================================
    c_ref, cdot_ref, cddot_ref  -> tache CoM
    f_ref                       -> regularisation mu_f*||f - f_ref||^2
                                   (Ch3 eq:qp-cost ECRIT f_ref ; le code actuel
                                   implemente f_ref = 0. Le MPC le fournit.)
    k_ref                       -> tache de moment cinetique (non presente dans
                                   l'executeur actuel ; expose pour Ch6)

===========================================================================
5. STATUT DU SOLVEUR
===========================================================================
Lecon de l'audit du 2026-08-11 : ProxQP NE LEVE PAS sur infaisabilite, il
retourne son dernier itere. Ici le statut est lu A CHAQUE appel et un solve
non converge est signale, compte, et remplace par la solution precedente
(strategie de repli explicite, jamais silencieuse).

===========================================================================
6. RESULTAT DE VALIDATION ET LIMITE STRUCTURELLE  (2026-08-11)
===========================================================================
Valide hors ligne sur la dynamique centroidale NON LINEAIRE exacte
(test_centroidal_mpc.py). Ce qui MARCHE :
  * equilibre statique : gravite compensee EXACTEMENT (932.0 N pour 95 kg),
    derive du CoM nulle a 1e-4 m pres sur 3 s ;
  * marche sur ~4 s (env. 7 pas) : RMSE CoM 45-53 mm, deviation de HAUTEUR
    2.5-4.3 mm (le LIPM l'imposerait a 0 par hypothese), moment cinetique
    borne a 6.5-6.8 kg.m2/s (le LIPM l'ignore) ;
  * cone de friction et unilateralite satisfaits a 100 % par construction ;
  * temps de resolution N=10, dt=50 ms : mean 4.9 ms, p99 7.9 ms
    -> tient la cible the thesis scope definition(« MPC lineaire < 10 ms »).

Ce qui NE MARCHE PAS, et pourquoi ce n'est pas un bug :
  * au-dela de ~4-6 s la trajectoire diverge (RMSE 2.9 m a 8 s) ;
  * tolerance a une impulsion laterale : 50 N absorbee (RMSE 65 mm),
    100 N marginale (237 mm), 150 N divergente.

CAUSE. Les pas sont FIXES par le plan : la seule autorite de rejet est le
deplacement du CoP a l'interieur de l'empreinte (~0.20 x 0.12 m). La dynamique
laterale etant un pendule inverse (omega ~ 3.4 rad/s, constante de temps
0.3 s), toute erreur residuelle croit exponentiellement, le CoP sature, et la
divergence suit. C'est un resultat CONNU : c'est exactement pourquoi Herdt et
al. (2010) font du LIEU DE PAS une variable de decision, et pourquoi le
marcheur S2 de cette these adapte la DUREE de pas (Khadiv et al.).

Autrement dit, cette implementation etablit quantitativement que la couche MPC
centroidale, SEULE, ne suffit pas : il lui faut une adaptation de pas. Le
levier evenementiel de S2 n'est donc pas un artifice, c'est le minimum
d'adaptativite requis. TOUTE conclusion de marche exige d'abord d'ajouter
l'adaptation du lieu de pas, puis l'integration MuJoCo.
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
#  Outils
# ---------------------------------------------------------------------------
def skew(r):
    """S(r) tel que S(r) @ f = r x f."""
    return np.array([[0.0, -r[2], r[1]],
                     [r[2], 0.0, -r[0]],
                     [-r[1], r[0], 0.0]])


class GaitSchedule:
    """Plan de pas + ordonnancement des contacts, aligne sur la convention S2.

    Geometrie de pied identique a talos_wbc.CORNERS_MEASURED : 4 coins par pied.
    Sequence : DS initial, puis alternance (DS bref, SS) comme le marcheur DCM.
    """

    def __init__(self, step_len=0.08, t_step=0.50, ds_ratio=0.12, ly=0.085,
                 n_steps=70, corners_x=(+0.095, -0.105), corners_y=(+0.06, -0.06),
                 t_settle=0.8):
        self.step_len, self.t_step, self.ds_ratio = step_len, t_step, ds_ratio
        self.ly, self.n_steps, self.t_settle = ly, n_steps, t_settle
        self.corners = [(sx, sy) for sx in corners_x for sy in corners_y]
        self.n_pts_per_foot = len(self.corners)
        self.n_contacts = 2 * self.n_pts_per_foot          # 8 points au total

    def foot_centres(self, t):
        """Centres (x, y) des deux pieds a l'instant t. Indice 0 = gauche."""
        if t < self.t_settle:
            return np.array([0.0, +self.ly]), np.array([0.0, -self.ly])
        tau = t - self.t_settle
        k = int(tau / self.t_step)                          # numero de pas
        k = min(k, self.n_steps - 1)
        # le pied droit demarre ; a chaque pas, le pied de balancement avance de 2*step_len
        n_r = (k + 1) // 2                                  # nb de poses du pied droit
        n_l = k // 2
        xr = n_r * 2 * self.step_len
        xl = n_l * 2 * self.step_len
        return np.array([xl, +self.ly]), np.array([xr, -self.ly])

    def contact_points(self, t):
        """Positions monde des N_c points de contact (N_c x 3)."""
        cl, cr = self.foot_centres(t)
        pts = []
        for centre in (cl, cr):
            for sx, sy in self.corners:
                pts.append([centre[0] + sx, centre[1] + sy, 0.0])
        return np.asarray(pts)

    def contact_active(self, t):
        """Vecteur booleen (N_c,) : 1 si le point porte a l'instant t."""
        n = self.n_pts_per_foot
        s = np.ones(self.n_contacts)
        if t < self.t_settle:
            return s                                        # double appui initial
        tau = t - self.t_settle
        k = int(tau / self.t_step)
        phase = (tau - k * self.t_step) / self.t_step
        if phase < self.ds_ratio:
            return s                                        # double appui de transfert
        # appui simple : le pied qui avance est en l'air
        swing_is_right = (k % 2 == 0)
        if swing_is_right:
            s[n:] = 0.0
        else:
            s[:n] = 0.0
        return s

    def limit_cycle_state(self, mass, zc, t_start=None, lat_gain=0.75):
        """Etat initial SUR le cycle limite lateral periodique.

        Un bipede a pas pre-planifies ne peut pas demarrer au repos : la
        dynamique laterale est un pendule inverse, et tout ecart au cycle
        periodique croit en e^{omega t} (omega ~ 3.4 rad/s, soit 0.3 s de
        constante de temps). Sans cette initialisation, la divergence est
        garantie quel que soit le planificateur.

        Derivation (LIPM lateral, appui alternant a +-ly, duree T) : en imposant
        que l'etat apres un pas soit l'image miroir de l'etat initial, le
        systeme se resout en forme fermee et donne
            y(0) = 0           (le CoM est sur l'axe median)
            ydot(0) = p_y * omega * tanh(omega*T/2)
        ou p_y est le pied d'appui. C'est le meme resultat que la valeur de
        commutation xi_s = ly*tanh(omega*T/2) utilisee par le marcheur DCM
        (talos_dcm_walk_timing.py l.87), obtenue ici independamment.

        CALIBRATION `lat_gain` (mesuree par tir, 2026-08-11). La formule ci-dessus
        suppose une alternance d'appuis SIMPLES purs. Le plan reel comporte 12 %
        de double appui, pendant lequel le pendule lateral n'est pas celui du
        modele -> la formule SURESTIME vy. Balayage sur 4 s :

            mult   0.70   0.75   0.80   0.85   0.90   1.00   1.20
            RMSE   45.1   46.8   52.7   63.2   82.7  240.6 1634.3  mm

        Optimum plat sur [0.70, 0.80], divergence brutale au-dela : signature
        d'une variete instable. Defaut retenu 0.75, au centre du plateau.
        """
        omega = np.sqrt(9.81 / zc)
        t0 = self.t_settle if t_start is None else t_start
        # premier appui simple : le pied droit balance (k=0) -> appui GAUCHE, p_y = +ly
        p_y = +self.ly
        vy = lat_gain * p_y * omega * np.tanh(omega * self.t_step / 2.0)
        c_ref, v_ref = self.com_reference(t0, zc)
        c = np.array([c_ref[0], 0.0, zc])
        v = np.array([v_ref[0], vy, 0.0])
        return np.r_[c, mass * v, np.zeros(3)], t0

    def _support_centre(self, t):
        """Centre du polygone d'appui instantane (creneau : discontinu en DS->SS)."""
        cl, cr = self.foot_centres(t)
        act = self.contact_active(t)
        n = self.n_pts_per_foot
        if act[:n].any() and act[n:].any():
            return 0.5 * (cl + cr)
        return cl.copy() if act[:n].any() else cr.copy()

    def com_reference(self, t, zc, n_smooth=9):
        """Reference CoM = plan d'appui LISSE sur une periode de pas.

        Le centre d'appui instantane est un creneau : le CoM d'un bipede ne peut
        pas le suivre, et le prendre pour reference penalise le MPC pour une
        erreur qu'il a raison de commettre. On moyenne donc sur +-T_step/2, ce
        qui est l'equivalent discret du filtrage passe-bas reliant plan de ZMP
        et trajectoire de CoM. Aucune hypothese LIPM n'est introduite ici : le
        lissage porte sur la REFERENCE, pas sur le modele.
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
    """MPC centroidal convexe a horizon fuyant sur les forces de contact."""

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
        """Resout le MPC a l'instant t0 depuis l'etat x0 = [c, l, k].

        Retourne un dict : f_ref (nc x 3), c_ref, cdot_ref, cddot_ref, k_ref,
        status, solve_ms.
        """
        N, nu, nx, dt = self.N, self.nu, self.nx, self.dt
        # --- references et linearisation le long de l'horizon ---
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

        # --- inegalites : 1 ligne fz (bornee) + 4 lignes de friction par point ---
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
