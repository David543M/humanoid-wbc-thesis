import io
p='talos_s3_stairs.py'
s=io.open(p,encoding='utf8').read()
R=[]

R.append(('''"""
TALOS — S3 : MONTEE D'ESCALIER (3 marches) par approche LIPM PAR PLATEAUX.

Voir s3_design.md. Le marcheur S2 evenementiel (talos_dcm_walk_timing.DCMWalkT)
suppose un sol plat : hauteur de pied `self.fz` scalaire et hauteur de CoM
`self.zc` figee (omega=sqrt(g/zc) calcule une seule fois). Ce fichier leve
l'hypothese plane SANS toucher au noyau (talos_wbc, talos_dcm_walk,
talos_dcm_walk_timing INCHANGES) via une sous-classe :

  1. `self.fz` devient une PROPRIETE renvoyant la hauteur de la marche CIBLE du
     pied de balancement (stair_height(x_cible)). Toute la logique de swing, de
     garde au sol et de detection de poser de DCMWalkT.update() l'utilise telle
     quelle -> cloche de swing et poser corrects sur chaque marche, sans
     reimplementer update().
  2. `self.zc` (reference verticale de CoM, lue dans DCMWalk.control() ligne 219)
     est ELEVEE en rampe vers la hauteur de la marche D'APPUI -> le CoM gagne
     reellement de l'altitude au lieu de s'accroupir. omega reste fixe (plateau
     quasi-statique, cf. s3_design.md par.3 ; omega_k = raffinement future work).
  3. Metrique NOUVELLE `foot clearance` : gap minimal semelle<->surface sous le
     pied pendant le vol (negatif = collision avec le nez de marche).

Le plan de pas DCM reste 2D (xy) : l'escalier n'ajoute que la composante z.

Profil d'escalier = doit rester synchronise avec scene_stairs.xml.

Usage (depuis pal_talos/, conda base avec mujoco) :
    python talos_s3_stairs.py                    # bring-up deterministe
    python talos_s3_stairs.py --viewer           # viewer MuJoCo (relance si chute)
    python talos_s3_stairs.py --seed 0 --save s3_run.npz
    python talos_s3_stairs.py --hriser 0.05      # balayage de conception
"""''', '''"""
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
"""'''))

R.append(("# --- profil d'escalier : (front_x, top_z) ---", "# --- stair profile: (front_x, top_z) ---"))
R.append(('''# Le profil de REFERENCE (make_stair_height) et la SCENE physique
# (build_stairs_xml) sont derives des MEMES parametres (x0, tread, h_riser, n),
# ce qui garantit leur coherence quel que soit le balayage.''',
'''# The REFERENCE profile (make_stair_height) and the physical SCENE
# (build_stairs_xml) are derived from the SAME parameters (x0, tread, h_riser, n),
# which guarantees their consistency regardless of the sweep.'''))
R.append(("EPS = 1e-6              # tolerance de bord (evite les artefacts flottants aux nez de marche)",
           "EPS = 1e-6              # edge tolerance (avoids floating-point artifacts at step noses)"))
R.append(("PLATFORM = 0.40        # profondeur du palier d'arrivee (m)",
           "PLATFORM = 0.40        # depth of the landing platform (m)"))
R.append(('''        # au-dela du palier d'arrivee, on retombe au sol (securite : le robot ne
        # doit pas depasser top_x, sinon la marche disparait sous le pied)''',
'''        # beyond the landing platform, drop back to ground level (safety: the robot
        # must not exceed top_x, otherwise the step disappears from under the foot)'''))
R.append(('''    """Genere la scene MuJoCo (n marches BOX) a partir des memes parametres que
    le profil de reference -> physique et reference toujours synchrones."""''',
'''    """Generates the MuJoCo scene (n BOX steps) from the same parameters as
    the reference profile -> physics and reference always stay in sync."""'''))
R.append(('''class DCMWalkS3(T.DCMWalkT):
    """Montee d'escalier par plateaux au-dessus du marcheur evenementiel S2."""''',
'''class DCMWalkS3(T.DCMWalkT):
    """Stair climbing via plateaus, layered on top of the event-driven S2 walker."""'''))
R.append(('''    _stairs = None          # defaut classe : desactive (self.fz == sol plat) tant
                            # que la config escalier n'est pas posee en fin d'__init__''',
'''    _stairs = None          # class default: disabled (self.fz == flat ground) until
                            # the stair config is set at the end of __init__'''))
R.append(('''        super().__init__(m, d, **kw)        # DCMWalk.__init__ fait self.fz = p0[...]
                                            #  -> passe par le setter -> _fz0
        self._stairs = None                 # plat pendant la reconstruction du plan
        self.zc0 = self.zc                  # hauteur de CoM au sol (reference)
        # offset cheville->semelle (pour la metrique de clearance)''',
'''        super().__init__(m, d, **kw)        # DCMWalk.__init__ does self.fz = p0[...]
                                            #  -> goes through the setter -> _fz0
        self._stairs = None                 # flat while rebuilding the plan
        self.zc0 = self.zc                  # CoM height on the ground (reference)
        # ankle->sole offset (for the clearance metric)'''))
R.append(('''        self.ZC_RATE = zc_rate              # vitesse de rampe verticale du CoM (m/s)
        # garde au sol ADAPTATIVE : plat = valeur S2 gelee (0.04, ne PAS casser le
        # cycle limite valide) ; franchissement de contremarche = garde majoree.
        # NB : self.fz = hauteur de la marche CIBLE releve deja toute la cloche au
        # niveau superieur -> STEP_H n'a qu'a couvrir le retard PD au-dessus du nez,
        # PAS la hauteur de contremarche (erreur du 1er jet : 0.14 global -> chute plat).
        self.FLAT_CLEAR = 0.04
        self.CROSS_CLEAR = 0.10             # surchargeable via --steph
        self.COM_RATE = 0.9                 # vitesse de co-pilotage CoM en montee (m/s)
        self.T_SWING_ARC = 0.45             # duree de l'arc up-over-down en appui simple (s)''',
'''        self.ZC_RATE = zc_rate              # vertical CoM ramp rate (m/s)
        # ADAPTIVE ground clearance: flat = frozen S2 value (0.04, must NOT break the
        # valid limit cycle); riser crossing = increased clearance.
        # NB: self.fz = height of the TARGET step already lifts the entire swing arc to
        # the upper level -> STEP_H only needs to cover the PD lag above the nose,
        # NOT the riser height (1st-draft bug: 0.14 flat everywhere -> fall on flat ground).
        self.FLAT_CLEAR = 0.04
        self.CROSS_CLEAR = 0.10             # overridable via --steph
        self.COM_RATE = 0.9                 # CoM co-drive rate while climbing (m/s)
        self.T_SWING_ARC = 0.45             # duration of the up-over-down arc in single support (s)'''))
R.append(('''        # STAIR-MODE : reconstruire le plan de pas (approche petits-pas + montee
        # step-together = 1 pied monte sur le giron, l'autre le rejoint). Les
        # appuis d'escalier sont FIGES par la geometrie (pas de replanification de
        # derive comme sur le plat) -> voir re-pin dans update().''',
'''        # STAIR-MODE: rebuild the footstep plan (small-step approach + step-together
        # climb = one foot steps onto the tread, the other joins it). The
        # stair footholds are PINNED by geometry (no drift replanning
        # like on flat ground) -> see re-pin in update().'''))
R.append(("        self._stairs = stair_height         # ACTIVE la logique escalier",
           "        self._stairs = stair_height         # ACTIVATES the stair logic"))
R.append(('''    def _rebuild_xi(self):
        """Recursion arriere DCM : xi_ini a l'entree de chaque appui (identique a
        DCMWalk.__init__, recalculee sur le plan d'escalier courant)."""''',
'''    def _rebuild_xi(self):
        """DCM backward recursion: xi_ini at the start of each stance (identical to
        DCMWalk.__init__, recomputed on the current stair plan)."""'''))
R.append(('''    def _build_stair_plan(self):
        """Plan de pas stair-mode : approche plate puis montee step-together.
        Remplace le plan uniforme 0.08 m de la classe de base."""''',
'''    def _build_stair_plan(self):
        """Stair-mode footstep plan: flat approach then step-together climb.
        Replaces the base class's uniform 0.08 m plan."""'''))
R.append(("        side = self.right                              # appui initial = pied droit",
           "        side = self.right                              # initial stance = right foot"))
R.append(("        ctr0 = risers[0][0] + 0.5 * tread              # centre du giron 1 (cible cheville)",
           "        ctr0 = risers[0][0] + 0.5 * tread              # center of tread 1 (ankle target)"))
R.append(('''        # anti-collision : la POINTE de la semelle (cheville + TOE_EXT) du dernier pied
        # d'approche ne doit PAS depasser le nez de la 1ere contremarche, sinon le pied
        # se pose dans la marche (bug « va trop loin »). On bride la cheville en consequence.''',
'''        # anti-collision: the TOE of the sole (ankle + TOE_EXT) of the last approach
        # foot must NOT go past the nose of the 1st riser, otherwise the foot
        # lands inside the step (the "overshoots" bug). The ankle is clamped accordingly.'''))
R.append(("        x_app_max = risers[0][0] - TOE_EXT - MARGIN     # cheville max d'un pied d'approche",
           "        x_app_max = risers[0][0] - TOE_EXT - MARGIN     # max ankle x of an approach foot"))
R.append(("        # APPROCHE : petits pas plats jusqu'a etre a portee de montee du giron 1",
           "        # APPROACH: small flat steps until within climbing range of tread 1"))
R.append(('''            if x_cur >= x_app_max - 1e-9:              # bord anti-collision atteint -> monter''',
'''            if x_cur >= x_app_max - 1e-9:              # anti-collision edge reached -> climb'''))
R.append(("        # MONTEE step-together : mount (monte sur le giron) puis join (l'autre pied rejoint)",
           "        # CLIMB step-together: mount (step onto the tread) then join (the other foot follows)"))
R.append(("        self._plan0 = [p.copy() for p in zmp]          # plan fige (re-pin)",
           "        self._plan0 = [p.copy() for p in zmp]          # pinned plan (re-pin)"))
R.append(("        # journaliser le plan (footholds vises)",
           "        # log the plan (targeted footholds)"))
R.append(('''        print("[plan] %d footholds : approche %d petits-pas + montee %d (step-together)"
              % (self.N, self._n_approach, self.N - self._n_approach))''',
'''        print("[plan] %d footholds: approach %d small-steps + climb %d (step-together)"
              % (self.N, self._n_approach, self.N - self._n_approach))'''))
R.append(('''    def _stairs_probe(self, x):
        """Hauteur d'escalier au point x (independant de l'activation self._stairs)."""''',
'''    def _stairs_probe(self, x):
        """Stair height at point x (independent of self._stairs activation)."""'''))
R.append(("    # ---- self.fz : hauteur de la marche CIBLE du pied de balancement ----",
           "    # ---- self.fz: height of the TARGET step of the swing foot ----"))
R.append(('''            return self._fz0                # sol plat (avant activation / fallback)''',
'''            return self._fz0                # flat ground (before activation / fallback)'''))
R.append(("        self._fz0 = float(v)                # hauteur de reference du sol plat",
           "        self._fz0 = float(v)                # reference height of the flat ground"))
R.append(("    # ---- reference verticale de CoM : suit la marche d'APPUI en rampe ----",
           "    # ---- vertical CoM reference: ramps to follow the STANCE step ----"))
R.append(('''        # garde au sol adaptative : fixee AVANT que le gait de base ne lise B.STEP_H.
        # crossing = le pied de balancement change de niveau (fz_land > fz_takeoff).
        # NB self.swing n'existe qu'apres le 1er super().update() -> garde hasattr
        # (avant le 1er pas, pas de swing : STEP_H=0.04 par defaut, correct).''',
'''        # adaptive ground clearance: set BEFORE the base gait reads B.STEP_H.
        # crossing = the swing foot changes level (fz_land > fz_takeoff).
        # NB self.swing only exists after the 1st super().update() -> hasattr guard
        # (before the 1st step, no swing: STEP_H=0.04 default, correct).'''))
R.append(("            if k != self._k_prev:           # nouveau pas : memoriser la hauteur de decollage",
           "            if k != self._k_prev:           # new step: remember the takeoff height"))
R.append(("        super().update(dt)                  # gait S2 evenementiel inchange",
           "        super().update(dt)                  # unchanged event-driven S2 gait"))
R.append(('''        # RE-PIN : la base translate les appuis futurs de l'erreur de poser (derive
        # plane). En escalier les appuis sont FIGES par la geometrie -> on restaure
        # le plan fige pour les pas a venir (l'appui courant garde le poser reel).''',
'''        # RE-PIN: the base class shifts future footholds by the touchdown error (flat-
        # ground drift). On stairs the footholds are PINNED by geometry -> restore
        # the pinned plan for upcoming steps (the current stance keeps its actual touchdown).'''))
R.append(('''        # STAIR-MODE (montee) : co-piloter le CoM en QUASI-STATIQUE. Le DCM plan seul
        # laisse le CoM traîner (~270 mm) derriere les grands footholds -> la jambe ne
        # peut pas atteindre la marche -> percute. On avance donc explicitement la
        # reference CoM (xy) sur l'appui, en la menant vers le milieu appui->prochain
        # foothold avec la phase du pas. Remplace la reference DCM sur les pas de montee.''',
'''        # STAIR-MODE (climb): co-drive the CoM QUASI-STATICALLY. The DCM plan alone
        # leaves the CoM trailing (~270 mm) behind the large footholds -> the leg
        # cannot reach the step -> it hits it. So we explicitly advance the
        # CoM (xy) reference over the stance, driving it toward the midpoint of stance->next
        # foothold with the step phase. Replaces the DCM reference on climbing steps.'''))
R.append(('''        # STAIR-MODE : surcharger la trajectoire du pied sur les pas de MONTEE
        # (remplace la cloche plate + le mode recovery lateral-pur, inadaptes).''',
'''        # STAIR-MODE: override the foot trajectory on CLIMB steps
        # (replaces the flat swing arc + the lateral-only recovery mode, both unsuitable).'''))
R.append(('''    def _climb_com_codrive(self, dt):
        """Reference CoM quasi-statique pendant la montee : suit l'appui et avance
        vers le milieu (appui, prochain foothold) selon la phase -> le corps monte
        AVEC les pieds au lieu de traîner. Ralentir (--tstep) = plus quasi-statique."""''',
'''    def _climb_com_codrive(self, dt):
        """Quasi-static CoM reference during the climb: follows the stance foot and moves
        toward the midpoint of (stance, next foothold) according to the phase -> the body climbs
        WITH the feet instead of trailing. Slowing down (--tstep) = more quasi-static."""'''))
R.append(('''        # stair-mode actif des que la CIBLE du pas (zmp[k+1]) est un foothold
        # d'escalier -> k+1 >= _n_approach (sinon le 1er mount reste au gait plat
        # = overshoot + percute la marche : bug d'indice observe « va trop loin »).''',
'''        # stair-mode active as soon as the step TARGET (zmp[k+1]) is a stair
        # foothold -> k+1 >= _n_approach (otherwise the 1st mount stays on the flat gait
        # = overshoot + hits the step: the observed "overshoots" indexing bug).'''))
R.append(('''        # QUASI-STATIQUE STRICT : la reference CoM SUIT LE PIED D'APPUI (x ET y).
        # Le trace #7 a montre que viser le MILIEU appui->prochain foothold poussait
        # le CoM DEVANT le seul pied au sol pendant l'appui simple (CoM 0.44 vs appui
        # 0.19) -> bascule avant (pitch +65, DCM_x->1.8, chute). En restant au-dessus
        # de l'appui, le CoM n'avance QUE lorsque ce pied (plus avance) devient l'appui
        # a l'etape suivante -> montee en « CoM suit l'appui », sans se jeter en avant.''',
'''        # STRICT QUASI-STATIC: the CoM reference FOLLOWS THE STANCE FOOT (x AND y).
        # Trace #7 showed that targeting the MIDPOINT stance->next foothold pushed
        # the CoM AHEAD of the single foot on the ground during single support (CoM 0.44 vs stance
        # 0.19) -> forward tip-over (pitch +65, DCM_x->1.8, fall). By staying above
        # the stance foot, the CoM only advances once that (more advanced) foot becomes the stance
        # at the next step -> climbing with "CoM follows the stance", without pitching forward.'''))
R.append(('''    def _climb_swing_override(self):
        """Trajectoire de swing dediee au franchissement : leve d'abord (pic a
        s~0.45, sans saut au depart), avance ensuite (retardee), descend au CENTRE
        du giron fige. Phase indexee sur T_nom (PAS sur le timing adaptatif qui
        ecourtait le pas -> pied court -> percute la contremarche). Bypasse aussi
        le mode recovery de la base (lateral-pur = fatal en escalier)."""''',
'''    def _climb_swing_override(self):
        """Dedicated swing trajectory for step crossing: lifts first (peak at
        s~0.45, no jump at the start), advances next (delayed), descends onto the CENTER
        of the pinned tread. Phase indexed on T_nom (NOT on the adaptive timing which
        shortened the step -> short foot -> hits the riser). Also bypasses
        the base class's recovery mode (lateral-only = fatal on stairs)."""'''))
R.append(('''        # actif des que la CIBLE (zmp[k+1]) est un foothold d'escalier ; le 1er mount
        # est le pas k=_n_approach-1 (bug d'indice corrige : « 3e pas va trop loin »).''',
'''        # active as soon as the TARGET (zmp[k+1]) is a stair foothold; the 1st mount
        # is step k=_n_approach-1 (indexing bug fixed: "3rd step overshoots").'''))
R.append(('''        # NB : le GATING (maintien du pied au sol en DS) a ete RETIRE (stair-mode #9) :
        # le timing adaptatif du gait plat raccourcit le pas a T_MIN quand le DCM derive,
        # terminant le pas PENDANT le maintien -> mount jamais execute (0/3). Incompatibilite
        # architecturale documentee (Ch5 §5.6 / Ch6). Config conservee = « CoM suit l'appui ».''',
'''        # NB: GATING (holding the foot on the ground during DS) was REMOVED (stair-mode #9):
        # the flat gait's adaptive timing shortens the step to T_MIN when the DCM drifts,
        # ending the step WHILE holding -> mount never executes (0/3). Architectural
        # incompatibility documented (Ch5 §5.6 / Ch6). Config kept = "CoM follows the stance".'''))
R.append(('''            return                                       # double appui : gere par la base''',
'''            return                                       # double support: handled by the base class'''))
R.append(('''        # profil UP-OVER-DOWN : monter VERTICAL au-dessus du decollage (phase 1),
        # avancer A HAUTEUR DE PIC (phase 2, la semelle est deja au-dessus du nez),
        # descendre sur le giron cible (phase 3). Le mouvement horizontal ne se fait
        # QU'A hauteur de pic -> la semelle ne frotte plus la contremarche (bug frottement).
        # NB self.swing_pos est la cheville ; la semelle est ~_ankle plus bas, la garde
        # CROSS_CLEAR (peak) couvre cet offset + le retard PD.''',
'''        # UP-OVER-DOWN profile: rise VERTICALLY above the takeoff point (phase 1),
        # advance AT PEAK HEIGHT (phase 2, the sole is already above the nose),
        # descend onto the target tread (phase 3). Horizontal motion happens
        # ONLY at peak height -> the sole no longer scrapes the riser (scraping bug).
        # NB self.swing_pos is the ankle; the sole is ~_ankle lower, the CROSS_CLEAR
        # clearance (peak) covers this offset + the PD lag.'''))
R.append(("        if s < LIFT:                                     # 1) monter vertical",
           "        if s < LIFT:                                     # 1) rise vertically"))
R.append(("        elif s < FWD:                                    # 2) avancer a hauteur de pic",
           "        elif s < FWD:                                    # 2) advance at peak height"))
R.append(("            xfrac = u * u * (3.0 - 2.0 * u)              # smootherstep horizontal",
           "            xfrac = u * u * (3.0 - 2.0 * u)              # horizontal smootherstep"))
R.append(("        else:                                            # 3) descendre sur le giron",
           "        else:                                            # 3) descend onto the tread"))
R.append(('''    def _clearance_tick(self):
        """Gap minimal semelle<->surface directement sous le pied pendant le vol."""''',
'''    def _clearance_tick(self):
        """Minimum gap sole<->surface directly under the foot during flight."""'''))
R.append(('''def _base_pitch_deg(d):
    """Tangage du bassin (deg) : + = penche vers l'AVANT (se jette en avant)."""''',
'''def _base_pitch_deg(d):
    """Pelvis pitch (deg): + = leaning FORWARD (pitching forward)."""'''))
R.append(('''def trace_row(m, d, c):
    """Instantane detaille de l'etat (1 ligne de trace)."""''',
'''def trace_row(m, d, c):
    """Detailed state snapshot (1 trace row)."""'''))
R.append(("    # --- geometrie escalier (doit rester synchro avec scene_stairs.xml) ---",
           "    # --- stair geometry (must stay in sync with scene_stairs.xml) ---"))
R.append(("    # --- gait : defauts = config S2 gelee ; tstep majore pour grands pas ---",
           "    # --- gait: defaults = frozen S2 config; tstep increased for large steps ---"))
R.append(("    top_target = risers[-1][1]              # hauteur du sommet (= n*h_riser)",
           "    top_target = risers[-1][1]              # top height (= n*h_riser)"))
R.append(("    # generer la scene physique depuis les MEMES parametres (coherence garantie)",
           "    # generate the physical scene from the SAME parameters (consistency guaranteed)"))
R.append(('''    print("[scene] scene_stairs.xml genere : x0=%.2f tread=%.2f h=%.2f n=%d (top_x=%.2f)"
          % (a.x0, a.tread, a.hriser, len(risers), top_x))''',
'''    print("[scene] scene_stairs.xml generated: x0=%.2f tread=%.2f h=%.2f n=%d (top_x=%.2f)"
          % (a.x0, a.tread, a.hriser, len(risers), top_x))'''))
R.append(("    # parametres de gait via les globals du module de base (convention etablie)",
           "    # gait parameters via the base module's globals (established convention)"))
R.append(("    B.STEP_H = 0.04                          # garde plat = valeur S2 gelee ; adaptee par pas dans DCMWalkS3.update()",
           "    B.STEP_H = 0.04                          # flat clearance = frozen S2 value; adapted per step in DCMWalkS3.update()"))
R.append(("    T.SAG_MAX = a.sagmax                     # capture sagittale elargie (grands pas de montee)",
           "    T.SAG_MAX = a.sagmax                     # widened sagittal capture (large climbing steps)"))
R.append(('    W.MODEL = "scene_stairs.xml"             # <-- escalier au lieu du sol plat',
           '    W.MODEL = "scene_stairs.xml"             # <-- stairs instead of flat ground'))
R.append(('                    print("[viewer] chute t=%.2f s (pas k=%d) — relance" % (c.t, c.k))',
           '                    print("[viewer] fall t=%.2f s (step k=%d) — restarting" % (c.t, c.k))'))
R.append(("    t_cap = 6.0 + c.N * a.tmax + T.T_END     # duree basee sur la longueur du plan d'escalier",
           "    t_cap = 6.0 + c.N * a.tmax + T.T_END     # duration based on the length of the stair plan"))
R.append(('                _trace_print(trace_row(m, d, c)); print("[trc] ===== CHUTE (base_z<0.6) =====")',
           '                _trace_print(trace_row(m, d, c)); print("[trc] ===== FALL (base_z<0.6) =====")'))
R.append(('        print("[ok] %s (%d lignes)" % (a.tracefile, len(trace_rows)))',
           '        print("[ok] %s (%d lines)" % (a.tracefile, len(trace_rows)))'))
R.append(("    # ---------- bilan S3 ----------", "    # ---------- S3 summary ----------"))
R.append(('''    print("S3 STAIRS  steps=%d len=%.2f Tnom=%.2f [%.2f,%.2f] | escalier x0=%.2f tread=%.2f h=%.2f steph=%.2f seed=%s"
          % (a.steps, a.steplen, a.tstep, a.tmin, a.tmax, a.x0, a.tread, a.hriser, a.steph, a.seed))''',
'''    print("S3 STAIRS  steps=%d len=%.2f Tnom=%.2f [%.2f,%.2f] | stairs x0=%.2f tread=%.2f h=%.2f steph=%.2f seed=%s"
          % (a.steps, a.steplen, a.tstep, a.tmin, a.tmax, a.x0, a.tread, a.hriser, a.steph, a.seed))'''))
R.append(('''    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, pas k=%d)" % (fell_at, c.k)))''',
'''    print("  outcome      : %s%s" % ("UPRIGHT" if upright else "FELL",
          "" if fell_at is None else "  (t=%.2f s, step k=%d)" % (fell_at, c.k)))'''))
R.append(('''    print("  montee       : %d/%d marches (max appui %.2f m) | gain CoM z %.3f m -> %s"
          % (n_climbed, len(risers), c._max_climb, climb, "SUCCESS" if success else "FAIL"))''',
'''    print("  climb        : %d/%d steps (max stance %.2f m) | CoM z gain %.3f m -> %s"
          % (n_climbed, len(risers), c._max_climb, climb, "SUCCESS" if success else "FAIL"))'''))
R.append(('    print("  avancee x    : %.2f m | pas poses=%d / %d" % (dist, len(ts), a.steps - 1))',
           '    print("  x progress   : %.2f m | steps taken=%d / %d" % (dist, len(ts), a.steps - 1))'))
R.append(('''    print("  CLEARANCE    : min %.3f m | mean %.3f m | franchissements=%d%s"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clearance"]),
             "  ⚠ COLLISION" if (len(c.log["clearance"]) and np.nanmin(cl) < 0) else ""))''',
'''    print("  CLEARANCE    : min %.3f m | mean %.3f m | crossings=%d%s"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clearance"]),
             "  ⚠ COLLISION" if (len(c.log["clearance"]) and np.nanmin(cl) < 0) else ""))'''))
R.append(('''    if len(c.log["land_grf"]):
        print("  poser GRF    : pic %.0f N | mean %.0f N" % (np.nanmax(lg), np.nanmean(lg)))''',
'''    if len(c.log["land_grf"]):
        print("  landing GRF  : peak %.0f N | mean %.0f N" % (np.nanmax(lg), np.nanmean(lg)))'''))

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
