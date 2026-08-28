import io
p='lipm_mpc.py'
s=io.open(p,encoding='utf8').read()
R=[]

R.append(('''"""
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
"""''', '''"""
lipm_mpc.py — receding-horizon LIPM MPC with OPTIMIZED FOOTSTEP PLACEMENT.

Written 2026-08-11, after centroidal_mpc.py. Formulation from Herdt et al.
(2010), "Online walking motion generation with automatic footstep placement".

DEPENDENCIES: numpy + proxsuite. No MuJoCo, no import of the frozen files.

===========================================================================
WHY THIS FILE EXISTS
===========================================================================
`centroidal_mpc.py` literally answers "centroidal MPC" (linear AND angular
momentum, free CoM height) but, with FIXED footsteps, its only rejection
authority is the CoP within the support polygon: it diverges beyond ~4-6 s and
tolerates only a ~50 N lateral impulse (measured 2026-08-11).

The LIPM has a property the centroidal model lacks: the ZMP is a
LINEAR function of the state,
        z = c - (zc/g) * c_ddot
Taking the state x = [c, c_dot, c_ddot] and JERK as the input, everything is
linear AND THE x AND y AXES DECOUPLE. The QP drops from 240 variables to
~19 PER AXIS. At that price, the STEP POSITIONS can be added as decision
variables -- exactly the adaptation the centroidal MPC lacks.

The trade-off is explicit: constant CoM height, zero angular momentum.
This planner therefore does NOT answer the "centroidal" word in the research
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
    a/2 ||U||^2                      jerk regularization
  + b/2 ||V - V_ref||^2              velocity tracking (the walking command)
  + c/2 ||Z - Vc*f_c - Vf*F||^2      ZMP centered in the stance foot
  + d/2 ||F - F_nom||^2              stride regularity

HARD constraints:
  (1) ZMP in the support polygon:  -h <= Z - Vc*f_c - Vf*F <= h
  (2) sagittal stride:             |F_j - F_{j-1}| <= stride_max
  (3) lateral spacing:             F_j - F_{j-1} in [+min_sep, +max_sep] or
                              [-max_sep, -min_sep] depending on the foot -- this
                              is what forbids leg crossing.

SOLVER STATUS read on every call (cf. ProxQP audit of 2026-08-11).

===========================================================================
OFFLINE VALIDATION RESULTS  (2026-08-11, test_lipm_mpc.py)
===========================================================================
Protocol: start AT REST, without initializing on the limit cycle (the
centroidal MPC, by contrast, does not start without it). Setting used: T=0.05 s, N=32.

ESTABLISHED, AND REPRODUCIBLE
  * 15 steps over 8 s WITHOUT DIVERGENCE, standing start; lateral oscillation
    |y|max 62 mm (compare to the foot spacing ly = 85 mm);
  * 100% of QPs converged, status checked on every call;
  * TWO-AXIS solve time: 0.45 ms (0.17 ms at T=0.1 s), vs
    4.8 ms for the centroidal MPC -- ~19x cheaper, for ~35 variables per
    axis instead of 240;
  * LATERAL IMPULSE TOLERANCE, post-push excursion:
        0 N -> 49 mm | 100 N -> 52 | 200 N -> 81 | 300 N -> 244 | 400 N -> 688
    vs the FIXED-FOOTSTEP centroidal MPC: 50 N absorbed, 100 N marginal,
    150 N DIVERGENT. Adapting the step LOCATION is therefore worth roughly a
    factor of 4 in disturbance margin. This is THE result of this file.

NOT ESTABLISHED -- do not present as settled
  * THE ZMP CONSTRAINT IS NOT PROPERLY SATISFIED. The measured rate of
    "CoP within the support polygon" varies from 65% to 86% depending on how
    the double-support phase is counted. The definition of the metric changed
    three times during development, so THIS NUMBER IS NOT
    RELIABLE and the slack is active in an unquantified way. An MPC whose
    main constraint is not verified cannot be declared
    functional.
  * residual lateral drift of a few centimeters over 8 s;
  * velocity 10-15% above the command;
  * PLANT = nominal LIPM + disturbance: outside of pushes, the optimized model
    and the simulated model coincide. Strictly weaker validation than
    the centroidal MPC's, which runs against the exact nonlinear
    centroidal dynamics;
  * no coupling with the QP-WBC executor (requires MuJoCo, untested).

DEVELOPMENT HISTORY -- five defects found by MEASUREMENT, none by
reasoning: stance-foot engagement offset by one step; double-support
polygon too narrow at startup; velocity command active during
settling (the CoM would start ahead of the stance foot); hard ZMP constraint
with no slack variable, which sent ProxQP to 10,000 iterations; and above all
a PLANT inconsistent with the QP -- z(t+T) was constrained while
c(t) - (zc/g) c_ddot(t+T) was simulated. This last point alone explained most
of the sagittal violations.

HONEST CONCLUSION. This file establishes a COMPARISON -- plan adaptivity
dominates model richness in the regime tested (flat-ground walking, low
speed). It does not establish a walker.
"""'''))

R.append(('            raise SystemExit("[ABORT] proxsuite requis : pip install proxsuite")',
           '            raise SystemExit("[ABORT] proxsuite required: pip install proxsuite")'))
R.append(('''class LIPMWalkMPC:
    """MPC LIPM a horizon fuyant, placement de pas en variables de decision."""''',
'''class LIPMWalkMPC:
    """Receding-horizon LIPM MPC, footstep placement as decision variables."""'''))
R.append(("        self.m = int(np.ceil(self.N * self.T / self.t_step)) + 1   # pas futurs vus",
           "        self.m = int(np.ceil(self.N * self.T / self.t_step)) + 1   # future steps seen"))
R.append(('''    def _build_prediction(self):
        """Matrices de prediction condensees (constantes)."""''',
'''    def _build_prediction(self):
        """Condensed prediction matrices (constant)."""'''))
R.append(('''    def _selectors(self, t):
        """Qui porte a chaque echantillon de l'horizon ?

        Retourne (Vc, Vf, k0, side0) :
          Vc (N,)   1 si l'echantillon est porte par le pied ACTUEL (connu),
          Vf (N,m)  one-hot du pas FUTUR porteur (variable de decision),
          k0        indice du pas courant, side0 le pied porteur courant
                    (+1 = gauche, -1 = droit).
        """''',
'''    def _selectors(self, t):
        """Which foot bears load at each sample of the horizon?

        Returns (Vc, Vf, k0, side0):
          Vc (N,)   1 if the sample is borne by the CURRENT foot (known),
          Vf (N,m)  one-hot of the FUTURE bearing step (decision variable),
          k0        index of the current step, side0 the current stance foot
                    (+1 = left, -1 = right).
        """'''))
R.append(("                Vc[k] = 1.0                            # pied actuel, position connue",
           "                Vc[k] = 1.0                            # current foot, known position"))
R.append(("                Vf[k, min(j - k0 - 1, self.m - 1)] = 1.0   # pas futur, variable",
           "                Vf[k, min(j - k0 - 1, self.m - 1)] = 1.0   # future step, variable"))
R.append(('''        # convention GaitSchedule : le pied DROIT balance aux pas pairs
        # -> appui GAUCHE (+1) aux pas pairs, DROIT (-1) aux impairs''',
'''        # GaitSchedule convention: the RIGHT foot swings on even steps
        # -> LEFT stance (+1) on even steps, RIGHT (-1) on odd'''))
R.append(('''    def step_index(self, t):
        """-1 pendant l'etablissement initial, puis 0, 1, 2, ..."""''',
'''    def step_index(self, t):
        """-1 during the initial settling, then 0, 1, 2, ..."""'''))
R.append(('''    def _solve_axis(self, axis, x0, Vc, Vf, f_c, F_nom, v_ref, sides, h_vec):
        """Un QP pour un axe. X = [U (N) ; F (m)].

        h_vec (N,) : demi-empreinte AUTORISEE au ZMP a chaque echantillon. Elle
        n'est pas constante : pendant le double appui initial le polygone est
        celui des DEUX pieds, nettement plus large que celui d'un pied seul.
        Imposer l'empreinte d'un pied unique des le depart force un demarrage
        violent et fait diverger le premier pas.
        """''',
'''    def _solve_axis(self, axis, x0, Vc, Vf, f_c, F_nom, v_ref, sides, h_vec):
        """One QP per axis. X = [U (N) ; F (m)].

        h_vec (N,): half-width of the ZMP support ALLOWED at each sample. It
        is not constant: during the initial double support the polygon is
        that of BOTH feet, much wider than that of a single foot.
        Imposing a single-foot polygon from the start forces a
        violent startup and makes the first step diverge.
        """'''))
R.append(("        n = N + m + N                        # U | F | S (ecarts sur le ZMP)",
           "        n = N + m + N                        # U | F | S (slacks on the ZMP)"))
R.append(("        z0 = Pzs @ x0                       # partie ZMP independante de U",
           "        z0 = Pzs @ x0                       # ZMP part independent of U"))
R.append(("        # --- residus lineaires ---",
           "        # --- linear residuals ---"))
R.append(('''        # --- contraintes ---
        rows, lo, up = [], [], []
        # (1) ZMP dans l'empreinte, RELACHEE par un ecart s_k >= 0 fortement penalise.
        #     Un MPC a contraintes ZMP DURES et duree de pas figee devient
        #     infaisable des qu'une perturbation depasse l'autorite du CoP : le
        #     solveur part alors a la limite d'iterations. L'ecart rend le QP
        #     toujours faisable et rend la violation MESURABLE au lieu de fatale.''',
'''        # --- constraints ---
        rows, lo, up = [], [], []
        # (1) ZMP in the support polygon, RELAXED by a heavily penalized slack s_k >= 0.
        #     An MPC with HARD ZMP constraints and a fixed step duration becomes
        #     infeasible as soon as a disturbance exceeds the CoP's authority: the
        #     solver then hits the iteration limit. The slack keeps the QP
        #     always feasible and makes the violation MEASURABLE instead of fatal.'''))
R.append(('''        # (2) foulee sagittale / (3) ecartement lateral, entre pas consecutifs.
        # La ligne exprime  F_j - F_{j-1}  (ou F_0 - f_c pour j = 0) ; le terme
        # constant f_c passe donc au second membre pour j = 0 uniquement.''',
'''        # (2) sagittal stride / (3) lateral spacing, between consecutive steps.
        # The row expresses  F_j - F_{j-1}  (or F_0 - f_c for j = 0); the
        # constant term f_c therefore moves to the RHS only for j = 0.'''))
R.append(("                s = sides[j]                     # pied qui se pose : +1 gauche, -1 droit",
           "                s = sides[j]                     # foot that lands: +1 left, -1 right"))
R.append(("                if s < 0:                        # ordonner les bornes",
           "                if s < 0:                        # order the bounds"))
R.append(('''    def solve(self, xs, ys, t, foot_c):
        """xs, ys : etats [c, c_dot, c_ddot]. foot_c : (fx, fy) du pied porteur.

        Retourne dict(jerk, footsteps, zmp_pred, status, solve_ms).
        """''',
'''    def solve(self, xs, ys, t, foot_c):
        """xs, ys: states [c, c_dot, c_ddot]. foot_c: (fx, fy) of the stance foot.

        Returns dict(jerk, footsteps, zmp_pred, status, solve_ms).
        """'''))
R.append(("        # plan nominal des m prochains pas",
           "        # nominal plan for the next m steps"))
R.append(("            s = -side0 if (j % 2 == 0) else side0      # alternance stricte",
           "            s = -side0 if (j % 2 == 0) else side0      # strict alternation"))
R.append(('''        # demi-empreinte par echantillon : polygone des DEUX pieds tant que
        # l'echantillon tombe dans le double appui initial
        # Echantillons en DOUBLE APPUI : etablissement initial, ET debut de
        # chaque pas (recouvrement ds_ratio, comme le marcheur S2). Sans cette
        # phase, le ZMP devrait sauter d'un pied a l'autre en passant par le
        # milieu, qui n'appartient a aucune des deux empreintes : violation
        # laterale structurelle a chaque transition (mesure : 19 % des ticks).''',
'''        # per-sample half-width: polygon of BOTH feet as long as
        # the sample falls within the initial double support
        # Samples in DOUBLE SUPPORT: initial settling, AND the start of
        # each step (ds_ratio overlap, like the S2 walker). Without this
        # phase, the ZMP would have to jump from one foot to the other through the
        # middle, which belongs to neither support polygon: structural
        # lateral violation at every transition (measured: 19% of ticks).'''))
R.append(('''        # Consigne de vitesse NULLE pendant l'etablissement : commander la
        # vitesse de croisiere des t=0 fait partir le CoM en avant du pied
        # d'appui avant le premier pas ; le ZMP sature alors a l'avant de
        # l'empreinte et le marcheur tombe vers l'avant en foulees maximales.''',
'''        # ZERO velocity command during settling: commanding the
        # cruise velocity from t=0 sends the CoM ahead of the stance
        # foot before the first step; the ZMP then saturates at the front of
        # the support polygon and the walker falls forward at maximum stride.'''))
R.append(('''        # Boucle externe de POSITION laterale. Un MPC de Herdt suit une VITESSE ;
        # une consigne laterale nulle est satisfaite par n'importe quel decalage
        # constant, et la marche derive alors en diagonale (mesure : -0.08 a
        # -0.23 m sur 8 s, insensible a w_step). On referme donc la position par
        # un terme proportionnel, saturé pour ne jamais dominer le cycle lateral.''',
'''        # Outer LATERAL POSITION loop. A Herdt MPC tracks a VELOCITY;
        # a zero lateral command is satisfied by any constant offset,
        # and the walk then drifts diagonally (measured: -0.08 to
        # -0.23 m over 8 s, insensitive to w_step). Position is therefore closed
        # with a proportional term, saturated so it never dominates the lateral cycle.'''))
R.append(('''        return ("LIPM-MPC : %d appels | %d resolus | %d NON resolus\n"
                "           solve (2 axes) mean %.3f ms | p99 %.3f ms | max %.3f ms\n"
                "           statuts %s"
                % (s["calls"], s["solved"], s["not_solved"],
                   np.nanmean(t), np.nanpercentile(t, 99), np.nanmax(t),
                   s["statuses"]))''',
'''        return ("LIPM-MPC : %d calls | %d solved | %d NOT solved\n"
                "           solve (2 axes) mean %.3f ms | p99 %.3f ms | max %.3f ms\n"
                "           statuses %s"
                % (s["calls"], s["solved"], s["not_solved"],
                   np.nanmean(t), np.nanpercentile(t, 99), np.nanmax(t),
                   s["statuses"]))'''))

fails=[]
for old,new in R:
    n = s.count(old)
    if n != 1:
        fails.append((n, old[:80]))
    else:
        s = s.replace(old, new)
if fails:
    print("FAILURES:")
    for n,o in fails:
        print(n, repr(o))
else:
    io.open(p,'w',encoding='utf8',newline='\n').write(s)
    print('ok', len(R))
