import io
p='talos_dcm_walk_timing.py'
s=io.open(p,encoding='utf8').read()
R=[]

R.append(('''"""
TALOS — S2 : marche 3 m avec TIMING DE PAS ADAPTATIF (event-based touchdown).

Complement direct de WBC_planning_diagnosis.md §4 : le closed-loop --walk-cl
(feedback DCM + capture-point + cpswing) double la survie mais chute vers le
6e pas car la decision reste A HORAIRE FIXE. Ce fichier rend le cycle de
marche EVENEMENTIEL (Khadiv et al., step timing adaptation, T-RO 2020) :

  E1. DEMARRAGE : la marche ne commence pas a une date fixe mais quand le
      DCM mesure atteint la condition d'entree du cycle limite lateral
      (d_lat <= ly - xi_s) pendant le transfert de poids.
  E2. TOUCHDOWN : pendant l'appui simple le DCM lateral diverge en
      d(tau) = d0*e^{omega*tau} ; le poser est déclenché quand d atteint
      la valeur de commutation d* = ly + xi_s :
          T_touchdown = tau + ln(d*/d)/omega   (ne peut que RACCOURCIR le pas)
  E3. GARDE-PIED : le switch de contact n'a lieu que si le pied de
      balancement est effectivement pres du sol (< 1.5 cm) — jamais de
      no-slip dur sur un pied en l'air.

Le QP-WBC (talos_wbc.py, valide S1) et talos_dcm_walk.py restent INCHANGES —
tout est surcharge dans une sous-classe. Les lois de juin (--feedback,
--footfb, --cpswing, --footori) sont actives par defaut, debrayables --no-*.

A placer dans pal_talos/ :
    python talos_dcm_walk_timing.py                 # S2 : 40 pas x 0.08 = 3.2 m
    python talos_dcm_walk_timing.py --steps 10      # debug court
    python talos_dcm_walk_timing.py --no-timing     # ablation (= walk-cl-like)
    python talos_dcm_walk_timing.py --viewer        # viewer MuJoCo temps reel
    python talos_dcm_walk_timing.py --seed 3 --save s2.npz
"""''', '''"""
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
"""'''))

R.append(('''T_END     = 2.5    # equilibre final apres le dernier pas (s)
XFER_RATE = 0.10   # vitesse de transfert du poids (m/s) en phase initiale
XFER_TMIN = 0.40   # duree minimale du transfert (s)
XFER_TMAX = 4.0    # timeout du transfert (s) — demarre quand meme + warning
FOOT_TOL  = 0.010  # tolerance hauteur pied pour autoriser le touchdown (m)
Z_LAND    = 0.60   # vitesse de descente de la REFERENCE en mode landing (m/s)
Z_DESC    = 0.25   # vitesse de descente PHYSIQUE observee du pied (m/s) — pour l'anticipation
MIN_SEP   = 0.11   # separation laterale min entre centres des pieds (m) — anti-croisement
MAX_SEP   = 0.42   # separation laterale max (m) — anti-grand-ecart (large : autorite de recuperation)
SAG_MAX   = 0.13   # limite de capture sagittale : DCM avant - appui (m) -> poser immediat
# --- soft landing (--softland) : anti-vacillement de cheville au poser ---
DEBOUNCE_N = 2     # ticks de contact consecutifs requis avant bascule d'appui (4 ms @ 500 Hz)
ZLAND_K    = 15.0  # ease-in : v_descente ref = K*h (1/s) — decelere SEULEMENT sous ZLAND_MIN/K
                   # (~1.2 cm) : retard de toucher ~+45 ms vs baseline. v2-v3 (K=8, decel des 2.2 cm
                   # + lag PD du pied ~2 cm) : toucher ~0.15 s en retard -> overshoot DCM (FELL)
ZLAND_MIN  = 0.18  # vitesse plancher d'impact (m/s) — v1=0.10 REJETE (toucher bien trop tardif)
ZDESC_SOFT = 0.18  # vitesse physique de descente observee avec ease-in (pour l'anticipation t_lat)
LAND_WIN   = 0.10  # fenetre des metriques de poser (s) — loggees AVEC ou SANS --softland (A/B)''',
'''T_END     = 2.5    # final settling balance after the last step (s)
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
LAND_WIN   = 0.10  # landing-metrics window (s) — logged WITH or WITHOUT --softland (A/B)'''))

R.append(('''def apply_squat(m, d, drop):
    """Flechit les genoux dans la CONFIGURATION initiale, sans passer par le
    WBC : hanche -a, genou +2a, cheville -a (pieds a plat), base abaissee
    pour garder les semelles au sol. Le controleur construit ensuite sa
    posture home, zc et omega sur cette pose flechie — sortie de la
    singularite jambe tendue des le premier tick."""''',
'''def apply_squat(m, d, drop):
    """Bends the knees in the INITIAL CONFIGURATION, bypassing the
    WBC: hip -a, knee +2a, ankle -a (feet flat), base lowered
    to keep the soles on the ground. The controller then builds its
    home posture, zc and omega on this bent pose — clear of the
    straight-leg singularity from the first tick."""'''))
R.append(("    L_LEG = 0.76                        # cuisse+tibia TALOS (approx.; base corrigee numeriquement)",
           "    L_LEG = 0.76                        # TALOS thigh+shin (approx.; base numerically corrected)"))
R.append(("    d.qpos[2] -= (z1 - z0)              # ramener les semelles au sol",
           "    d.qpos[2] -= (z1 - z0)              # bring the soles back to the ground"))
R.append(('''    print("[squat] genou=%.1f deg  base_z=%.3f m  CoM_z=%.3f m"
          % (np.degrees(2 * alpha), d.qpos[2],
             float(d.subtree_com[m.body("base_link").id][2])))''',
'''    print("[squat] knee=%.1f deg  base_z=%.3f m  CoM_z=%.3f m"
          % (np.degrees(2 * alpha), d.qpos[2],
             float(d.subtree_com[m.body("base_link").id][2])))'''))
R.append(('''class DCMWalkT(B.DCMWalk):
    """Machine a pas evenementielle + timing adaptatif au-dessus de DCMWalk."""''',
'''class DCMWalkT(B.DCMWalk):
    """Event-driven footstep state machine + adaptive timing on top of DCMWalk."""'''))
R.append(('''        # NB : si apply_squat() a ete appele avant, zc et omega (mesures par
        # la classe de base sur la pose flechie) sont deja coherents.''',
'''        # NB: if apply_squat() was called beforehand, zc and omega (measured by
        # the base class on the bent pose) are already consistent.'''))
R.append(('''        # --- LEVIER L1 (--sagfb) : ajustement sagittal du LIEU de pas, couple a
        # la duree adaptative Tk. OFF PAR DEFAUT -> S2 (config gelee, batch N=50)
        # et S5 (230 essais) restent BIT-IDENTIQUES.
        # Motivation : audit_timing_vs_reactive_planners.md -> la loi de juin a
        # bien la loi de duree (Khadiv2020) mais AUCUN ajustement sagittal du
        # lieu ni couplage lieu<->duree ; S5 mesure I50 sagittal 20-30 N.s vs
        # lateral 45-50+ = signature d'un canal sagittal sans mecanisme actif.
        # b_sag = offset DCM sagittal du point fixe du cycle nominal :
        #   b * e^{omega*T} = STEP_LEN + b   =>   b = L / (e^{omega*T} - 1)
        # En regime nominal u_x = xi_pred_x - b_sag == plan : le levier est
        # NO-OP tant que le DCM sagittal ne devie pas (faible risque de regression).''',
'''        # --- LEVER L1 (--sagfb): sagittal adjustment of the step LOCATION, coupled
        # to the adaptive duration Tk. OFF BY DEFAULT -> S2 (frozen config, batch N=50)
        # and S5 (230 trials) remain BIT-IDENTICAL.
        # Motivation: audit_timing_vs_reactive_planners.md -> the June law does
        # have the duration law (Khadiv2020) but NO sagittal adjustment of the
        # location and no location<->duration coupling; S5 measures sagittal I50
        # 20-30 N.s vs lateral 45-50+ = signature of a sagittal channel with no active mechanism.
        # b_sag = sagittal DCM offset of the nominal cycle's fixed point:
        #   b * e^{omega*T} = STEP_LEN + b   =>   b = L / (e^{omega*T} - 1)
        # At nominal regime u_x = xi_pred_x - b_sag == plan: the lever is
        # a NO-OP as long as the sagittal DCM does not deviate (low regression risk).'''))
R.append(("        self.t0 = None                 # debut du pas courant (None = transfert)",
           "        self.t0 = None                 # start of the current step (None = transfer)"))
R.append(("        self.Tk = t_nom                # duree (adaptative) du pas courant",
           "        self.Tk = t_nom                # (adaptive) duration of the current step"))
R.append(("        self.ended = None              # instant d'entree en phase END",
           "        self.ended = None              # time of entry into the END phase"))
R.append(("        # cible finale : dernier appui + pied de fermeture cote a cote",
           "        # final target: last stance + closing foot side by side"))
R.append(("        # mecanismes soft-landing cote timing (opt-in ; sl_imp/sl_ramp sont dans la base)",
           "        # timing-side soft-landing mechanisms (opt-in; sl_imp/sl_ramp are in the base class)"))
R.append(("        self.sl_ease = False           # descente deceleree avant impact",
           "        self.sl_ease = False           # decelerated descent before impact"))
R.append(("        self.sl_debounce = False       # contact confirme sur DEBOUNCE_N ticks avant bascule",
           "        self.sl_debounce = False       # contact confirmed over DEBOUNCE_N ticks before switching"))
R.append(("        # etat des metriques de poser + debounce contact",
           "        # state for landing metrics + contact debounce"))
R.append(('''    def _foot_contact(self, fb):
        """Contact du corps fb avec le SOL (geoms du worldbody, body 0) — reste
        robuste au pied incline (pointe qui touche, cheville haute).
        v4 2026-07-08 : avant, N'IMPORTE QUEL contact du corps comptait (pied-pied,
        pied-jambe) -> poses fantomes a dz=+0.03..0.046 observees run70h."""''',
'''    def _foot_contact(self, fb):
        """Contact of body fb with the GROUND (worldbody geoms, body 0) — stays
        robust to a tilted foot (toe touching, ankle raised).
        v4 2026-07-08: previously, ANY contact of the body counted (foot-foot,
        foot-leg) -> phantom touchdowns at dz=+0.03..0.046 observed in run70h."""'''))
R.append(('''    def _grf_z(self, fb):
        """Somme des forces normales de contact pied-SOL (N) sur le corps fb."""''',
'''    def _grf_z(self, fb):
        """Sum of foot-GROUND contact normal forces (N) on body fb."""'''))
R.append(("                tot += abs(float(f6[0]))            # composante normale (repere contact)",
           "                tot += abs(float(f6[0]))            # normal component (contact frame)"))
R.append(('''    def _land_metrics_tick(self):
        """Fenetre LAND_WIN apres chaque poser : pic de vitesse angulaire du pied
        (vacillement), pic de GRF (impact), transitions make/break (rebond).
        Loggee avec ET sans --softland -> comparaison A/B directe pour Ch5/Ch6."""''',
'''    def _land_metrics_tick(self):
        """LAND_WIN window after each touchdown: peak angular velocity of the foot
        (wobble), peak GRF (impact), make/break transitions (bounce).
        Logged WITH and WITHOUT --softland -> direct A/B comparison for Ch5/Ch6."""'''))
R.append(('''    def _dlat(self, xi_y, k):
        """Distance laterale DCM->appui k, positive vers l'interieur du robot.
        Le cote de l'appui vient de support[k] (G/D), PAS du signe de sa
        position monde — qui s'inverse des que le robot derive lateralement."""''',
'''    def _dlat(self, xi_y, k):
        """Lateral distance DCM->stance k, positive toward the inside of the robot.
        The stance side comes from support[k] (L/R), NOT from the sign of its
        world position — which flips as soon as the robot drifts laterally."""'''))
R.append(("    # ---------- machine a pas ----------",
           "    # ---------- footstep state machine ----------"))
R.append(("        self._land_metrics_tick()                    # metriques de poser (toutes phases)",
           "        self._land_metrics_tick()                    # landing metrics (all phases)"))
R.append(("        # ---- E1 : transfert de poids evenementiel (remplace SETTLE fixe) ----",
           "        # ---- E1: event-driven weight transfer (replaces fixed SETTLE) ----"))
R.append(('''                if self.verbose:
                    tag = "TIMEOUT — depart force" if self.t > XFER_TMAX else "condition d'entree atteinte"
                    print("[gait] depart t=%.2f s (%s : dlat=%.3f, entree=%.3f)"
                          % (self.t, tag, d0, entry))''',
'''                if self.verbose:
                    tag = "TIMEOUT — forced start" if self.t > XFER_TMAX else "entry condition reached"
                    print("[gait] start t=%.2f s (%s: dlat=%.3f, entry=%.3f)"
                          % (self.t, tag, d0, entry))'''))
R.append(('''        # ---- E2/E3 : evenement de poser = CONTACT reel ----
        # Des que le pied touche pendant la fenetre d'atterrissage, on bascule
        # IMMEDIATEMENT le support : sinon le pied au sol reste hors du set de
        # contact et se fait trainer par les taches de swing encore actives.''',
'''        # ---- E2/E3: touchdown event = actual CONTACT ----
        # As soon as the foot touches during the landing window, we switch
        # the stance IMMEDIATELY: otherwise the foot on the ground stays outside
        # the contact set and gets dragged around by the still-active swing tasks.'''))
R.append(('''            # debounce (--softland) : un micro-rebond ne doit pas faire osciller
            # le set de contact DS/SS entre deux ticks''',
'''            # debounce (--softland): a micro-bounce must not make the
            # DS/SS contact set oscillate between two ticks'''))
R.append(("                # depart de la rampe de contact (--softland) + fenetre de metriques",
           "                # start of the contact ramp (--softland) + metrics window"))
R.append(("                self._flush_land_metrics()           # fenetre precedente non close (pas < 100 ms)",
           "                self._flush_land_metrics()           # previous window not closed (step < 100 ms)"))
R.append(('''                    print("[step %2d] pose %s  T=%.2fs  in=%.3f out=%.3f (d*=%.3f)  pied_dz=%+.3f"
                          % (k, "G" if self.swing == self.left else "D", tau,
                             getattr(self, "d_in", -1.0), dl,
                             self.ly + self.xi_s, foot_z - self.fz))''',
'''                    print("[step %2d] touchdown %s  T=%.2fs  in=%.3f out=%.3f (d*=%.3f)  foot_dz=%+.3f"
                          % (k, "G" if self.swing == self.left else "D", tau,
                             getattr(self, "d_in", -1.0), dl,
                             self.ly + self.xi_s, foot_z - self.fz))'''))
R.append(('''                if k + 1 < self.N:
                    # aligner TOUT LE PLAN RESTANT sur la realite : l'appui
                    # suivant devient le pied reellement pose, et les pas
                    # futurs (+ la geometrie nominale du plan) sont translates
                    # d'autant — plus aucune cible dans un repere fantome''',
'''                if k + 1 < self.N:
                    # align THE ENTIRE REMAINING PLAN to reality: the next
                    # stance becomes the foot that actually touched down, and
                    # future steps (+ the plan's nominal geometry) are shifted
                    # by the same amount — no more targets in a phantom frame'''))
R.append(('''                    # pas futurs : x translate sur la realite, mais le y se
                    # RE-CENTRE vers la ligne de depart (<= 2 cm/pas) -> marche
                    # en ligne droite malgre les derives des recuperations''',
'''                    # future steps: x shifts onto reality, but y RE-CENTERS
                    # toward the starting line (<= 2 cm/step) -> walks
                    # in a straight line despite recovery drift'''))
R.append(('''                    self.ended = self.t
                    # cible finale = milieu des pieds REELS (pas du plan)''',
'''                    self.ended = self.t
                    # final target = midpoint of the ACTUAL feet (not the plan)'''))
R.append(("        # ---- phase finale : equilibre statique entre les deux pieds ----",
           "        # ---- final phase: static balance between the two feet ----"))
R.append(("        # ---- debut de pas : decision de placement (capture point, base) ----",
           "        # ---- start of step: placement decision (capture point, base) ----"))
R.append(('''            # re-planification par pas : la reference laterale repart de
            # l'entree DCM MESUREE (coherence regle de timing <-> reference)''',
'''            # per-step replanning: the lateral reference restarts from
            # the MEASURED DCM entry (consistency between timing rule <-> reference)'''))
R.append(('''            # mode RECUPERATION : entree degeneree (DCM du mauvais cote ou
            # saturee) -> le pas suivant est purement LATERAL, on arrete
            # d'avancer tant que le cycle n'est pas rattrape''',
'''            # RECOVERY mode: degenerate entry (DCM on the wrong side or
            # saturated) -> the next step is purely LATERAL, forward
            # progress stops until the cycle is recovered'''))
R.append(('                print("[recov] k=%d entree degeneree (d0=%+.3f) -> pas lateral pur" % (k, d0))',
           '                print("[recov] k=%d degenerate entry (d0=%+.3f) -> pure lateral step" % (k, d0))'))
R.append(('''        # ---- E2 : timing adaptatif (canal lateral, en appui simple) ----
        # Suivre l'estimation COURANTE du temps de commutation : un DCM qui
        # fuit avance le poser ; un DCM sage peut allonger jusqu'a T_MAX.''',
'''        # ---- E2: adaptive timing (lateral channel, single support) ----
        # Track the CURRENT estimate of the switching time: a DCM that
        # runs away advances touchdown; a well-behaved DCM can extend up to T_MAX.'''))
R.append(('''            # les DEUX canaux sont predictifs : on pose quand le premier des
            # deux DCM (lateral ou sagittal) atteindra sa valeur de commutation''',
'''            # BOTH channels are predictive: touchdown happens when the first of the
            # two DCM components (lateral or sagittal) reaches its switching value'''))
R.append(('''                # DCM du mauvais cote de l'appui -> poser IMMEDIATEMENT
                # (et descendre le pied tout de suite, sans repasser par la cloche)''',
'''                # DCM on the wrong side of the stance -> touch down IMMEDIATELY
                # (and lower the foot right away, without going back through the arc)'''))
R.append(("        # ---- reference DCM : x = recursion (temps virtuel), y = cycle limite ----",
           "        # ---- DCM reference: x = recursion (virtual time), y = limit cycle ----"))
R.append(("        tv = s * self.T_nom                              # temps virtuel sagittal",
           "        tv = s * self.T_nom                              # sagittal virtual time"))
R.append(("        # ---- feedback DCM -> reference CoM STABLE (forme validee en juin) ----",
           "        # ---- DCM feedback -> STABLE CoM reference (form validated in June) ----"))
R.append(("        # ---- pied de balancement (duree variable geree par s = tau/Tk) ----",
           "        # ---- swing foot (variable duration handled by s = tau/Tk) ----"))
R.append(('''        if getattr(self, "_recover", False):
            goal[0] = self.zmp[k][0]          # recuperation : pas lateral pur,
                                              # aucune progression sagittale''',
'''        if getattr(self, "_recover", False):
            goal[0] = self.zmp[k][0]          # recovery: purely lateral step,
                                              # no sagittal progress'''))
R.append(('''        # phase : fin de pas en DS SEULEMENT si le pied est reellement pose —
        # jamais de contrainte no-slip sur un pied en l'air''',
'''        # phase: step ends in DS ONLY if the foot is actually down —
        # never a no-slip constraint on a foot in the air'''))
R.append(('''        if self.use_cp_swing and self.phase == "SS":
            # poser le pied AU-DELA du DCM mesure, decale de off_lat :
            # reproduit l'entree du cycle au pas suivant (Khadiv). off_lat >
            # (ly - xi_s) nominal pour compenser la poussee du double appui
            # (ZMP entre les pieds -> le DCM derive vers le nouvel appui).''',
'''        if self.use_cp_swing and self.phase == "SS":
            # place the foot BEYOND the measured DCM, offset by off_lat:
            # reproduces the cycle entry at the next step (Khadiv). off_lat >
            # nominal (ly - xi_s) to compensate for the double-support push
            # (ZMP between the feet -> the DCM drifts toward the new stance).'''))
R.append(('''            # viser le DCM PREDIT au moment du poser : pendant la descente du
            # pied (~0.15 s) le DCM continue de diverger (~x1.6) — viser le
            # DCM courant fait atterrir le pied "en retard", offset mange''',
'''            # target the PREDICTED DCM at touchdown time: during the foot's
            # descent (~0.15 s) the DCM keeps diverging (~x1.6) — targeting the
            # current DCM lands the foot "late", eating into the offset'''))
R.append(('                off = max(off, 0.11)          # recuperation : poser plus loin au-dela du DCM',
           '                off = max(off, 0.11)          # recovery: land further beyond the DCM'))
R.append(('''            # bornes RELATIVES AU PIED D'APPUI reel : jamais moins de min_sep
            # entre les pieds (anti-croisement), jamais plus de MAX_SEP''',
'''            # bounds RELATIVE TO THE ACTUAL STANCE FOOT: never less than min_sep
            # between the feet (anti-crossing), never more than MAX_SEP'''))
R.append(('''            # ---- L1 : ajustement sagittal du lieu de pas (capture point) ----
            # u_x = xi_pred_x(t_go) - b_sag, avec le MEME t_go que le canal
            # lateral -> le LIEU est couple a la DUREE adaptative Tk (c'est le
            # couplage lieu<->duree absent de la loi de juin).
            # Garde-fous : (1) inactif en mode recuperation (le pas lateral pur
            # reste prioritaire) ; (2) borne a +-sag_dev autour du plan = une
            # CORRECTION, pas une replanification ; (3) jamais en arriere de
            # l'appui courant -> progression sagittale monotone.''',
'''            # ---- L1: sagittal adjustment of the step location (capture point) ----
            # u_x = xi_pred_x(t_go) - b_sag, with the SAME t_go as the lateral
            # channel -> the LOCATION is coupled to the adaptive DURATION Tk (this
            # is the location<->duration coupling absent from the June law).
            # Guardrails: (1) inactive in recovery mode (the pure lateral step
            # stays priority); (2) bounded to +-sag_dev around the plan = a
            # CORRECTION, not a replan; (3) never behind the current
            # stance -> monotonic sagittal progress.'''))
R.append(('''        if (tau >= self.Tk - t_lat or getattr(self, "_force_land", False)) and not landed:
            # LANDING demande : descente active, cible FIGEE (plus de poursuite
            # du DCM en fin de vol), swing task actif jusqu'au contact''',
'''        if (tau >= self.Tk - t_lat or getattr(self, "_force_land", False)) and not landed:
            # LANDING requested: active descent, FROZEN target (no more DCM
            # tracking near the end of flight), swing task active until contact'''))
R.append(('''            if self.sl_ease:
                # ease-in : la reference decelere pres du sol -> impact ~ZLAND_MIN
                # au lieu de ~Z_DESC (l'energie d'impact chute d'un facteur ~(0.25/0.10)^2)''',
'''            if self.sl_ease:
                # ease-in: the reference decelerates near the ground -> impact ~ZLAND_MIN
                # instead of ~Z_DESC (impact energy drops by a factor ~(0.25/0.10)^2)'''))
R.append(("        # ---- debug : etat du timing toutes les ~0.1 s ----",
           "        # ---- debug: timing state every ~0.1 s ----"))
R.append(("        # ---- metriques ----",
           "        # ---- metrics ----"))
R.append(("    # parametres de gait via les globals du module de base (convention etablie)",
           "    # gait parameters via the base module's globals (established convention)"))
R.append(("    B.STEP_H = 0.04     # cloche plus basse : moins de distance a descendre au poser",
           "    B.STEP_H = 0.04     # lower swing arc: less distance to descend at touchdown"))
R.append(('''    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0   # chevilles fermes (900/90 + wfootst 1500
                                          # teste 2026-07-06 : FELL — vole les couples
                                          # de cheville necessaires au ZMP)''',
'''    W.KP_FOOT, W.KD_FOOT = 650.0, 65.0   # stiff ankles (900/90 + wfootst 1500
                                          # tested 2026-07-06: FELL — steals the ankle
                                          # torque needed for the ZMP)'''))
R.append(('''    B.KP_SW, B.KD_SW = 420.0, 42.0       # swing plus rapide : cibles laterales
                                          # larges atteignables en < 0.3 s''',
'''    B.KP_SW, B.KD_SW = 420.0, 42.0       # faster swing: large lateral
                                          # targets reachable in < 0.3 s'''))
R.append(("    if a.floortc > 0:                     # sol plus mou : timeconst du solref des geoms du worldbody",
           "    if a.floortc > 0:                     # softer ground: solref timeconst of the worldbody geoms"))
R.append(('        print("[floor] solref timeconst=%.3f s applique a %d geom(s) du sol" % (a.floortc, nfl))',
           '        print("[floor] solref timeconst=%.3f s applied to %d ground geom(s)" % (a.floortc, nfl))'))
R.append(("        apply_squat(m, d_, a.zcdrop)     # flexion AVANT construction du WBC",
           "        apply_squat(m, d_, a.zcdrop)     # bend the knees BEFORE building the WBC"))
R.append(("        c_.sag_fb = a.sagfb                  # LEVIER L1 (defaut False = config gelee)",
           "        c_.sag_fb = a.sagfb                  # LEVER L1 (default False = frozen config)"))
R.append(("        c_.b_sag *= a.bsagk                  # calibration du point fixe sur le gait REEL",
           "        c_.b_sag *= a.bsagk                  # calibration of the fixed point on the ACTUAL gait"))
R.append(("        if a.sagsub: c_.foot_lat_only = True # L1 remplace (au lieu de doubler) le footfb sagittal",
           "        if a.sagsub: c_.foot_lat_only = True # L1 replaces (instead of doubling up) the sagittal footfb"))
R.append(("    # ---------- viewer temps reel (meme pattern que view_talos_dcm.py) ----------",
           "    # ---------- real-time viewer (same pattern as view_talos_dcm.py) ----------"))
R.append(('''                if d.qpos[2] < 0.5:                      # chute -> relance auto
                    print("[viewer] chute a t=%.2f s (pas k=%d) — relance" % (c.t, c.k))''',
'''                if d.qpos[2] < 0.5:                      # fall -> auto restart
                    print("[viewer] fall at t=%.2f s (step k=%d) — restarting" % (c.t, c.k))'''))
R.append(("    # ---------- bilan S2 ----------",
           "    # ---------- S2 summary ----------"))
R.append(('''    print("  levier L1    : sagfb=%s%s" %
          (a.sagfb, ("  (b_sag=%.4f m [k=%.2f], dev max %.3f m)" % (c.b_sag, a.bsagk, a.sagdev)) if a.sagfb else ""))''',
'''    print("  lever L1     : sagfb=%s%s" %
          (a.sagfb, ("  (b_sag=%.4f m [k=%.2f], dev max %.3f m)" % (c.b_sag, a.bsagk, a.sagdev)) if a.sagfb else ""))'''))
R.append(('''    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, pas k=%d)" % (fell_at, c.k)))''',
'''    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, step k=%d)" % (fell_at, c.k)))'''))
R.append(('''    print("  distance     : %.2f m (cible %.1f) -> %s | derive laterale %.2f m" %
          (dist, a.dist, "SUCCESS" if success else "FAIL", float(com[1] - y0m)))''',
'''    print("  distance     : %.2f m (target %.1f) -> %s | lateral drift %.2f m" %
          (dist, a.dist, "SUCCESS" if success else "FAIL", float(com[1] - y0m)))'''))
R.append(('    print("  pas atteint  : k=%d / %d | pas poses=%d" % (c.k, a.steps - 1, len(ts)))',
           '    print("  steps reached: k=%d / %d | steps taken=%d" % (c.k, a.steps - 1, len(ts)))'))
R.append(('''        print("  duree de pas : mean %.2f s | min %.2f | max %.2f (adaptatif)" %
              (ts.mean(), ts.min(), ts.max()))''',
'''        print("  step duration: mean %.2f s | min %.2f | max %.2f (adaptive)" %
              (ts.mean(), ts.min(), ts.max()))'''))
R.append(('''        print("  poser 100 ms : |w_pied| pic %.2f rad/s (mean %.2f) | GRF pic %.0f N (mean %.0f)"
              " | make/break mean %.1f" %
              (lw.max(), lw.mean(), lg.max(), lg.mean(), lmb.mean()))''',
'''        print("  touchdown 100 ms: |w_foot| peak %.2f rad/s (mean %.2f) | GRF peak %.0f N (mean %.0f)"
              " | make/break mean %.1f" %
              (lw.max(), lw.mean(), lg.max(), lg.mean(), lmb.mean()))'''))

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
