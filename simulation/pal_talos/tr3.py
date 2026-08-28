import io
p='talos_s3_stairs_qs.py'
s=io.open(p,encoding='utf8').read()
R=[]

R.append(('''"""
TALOS — S3 : MONTEE D'ESCALIER par SEQUENCEUR QUASI-STATIQUE.

Motivation (voir Ch5 §5.6) : le marcheur plat
DCM (talos_dcm_walk_timing) repose sur un TIMING EVENEMENTIEL (touchdown
adaptatif, capture-point, recovery) qui s'est revele FONDAMENTALEMENT
INCOMPATIBLE avec les gros transferts de poids deliberes d'un escalier :
le timing raccourcit le pas a T_MIN des que le DCM derive, terminant le
mount avant que le corps ait bascule sur la marche.

Ce fichier ABANDONNE ce timing. Il garde UNIQUEMENT l'executeur QP-WBC
valide (DCMWalk.control() : dynamique corps-rigide dure, cone de friction,
limites de couple, taches CoM / orientation / posture / pied ; PROXQP) et
pilote une MACHINE A ETATS EXPLICITE quasi-statique :

    SETTLE                     : les 2 pieds au sol, CoM centre, posture etablie
    pour chaque pas de plan :
      TRANSFER (double appui)  : les 2 pieds PORTENT (contact dur dans le QP),
                                 le CoM bascule (x,y) sur le pied d'APPUI +
                                 la hauteur de CoM monte a la marche d'appui,
                                 jusqu'a convergence (pas de course contre une
                                 horloge : on attend le transfert).
      SWING   (appui simple)   : SEUL le pied d'appui porte ; le pied libre
                                 suit un arc UP-OVER-DOWN vers le centre du
                                 giron fige ; le CoM reste sur l'appui.
      -> pose (contact) -> pas suivant.
    DONE                       : equilibre final sur le palier.

Le point cle : en DS, control() met les DEUX pieds dans l'ensemble de
contact (contrainte no-slip dure sur chacun) -> le transfert de poids
lateral+vertical se fait a deux pieds, ce qui manquait au marcheur plat.
Aucune horloge de pas ne peut interrompre le transfert : la transition
DS->SS n'a lieu QUE lorsque le CoM a effectivement bascule.

Usage (depuis pal_talos/, conda base) :
    python talos_s3_stairs_qs.py                     # bring-up
    python talos_s3_stairs_qs.py --viewer
    python talos_s3_stairs_qs.py --trace --save s3qs.npz
"""''', '''"""
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
"""'''))

R.append(('''class StairQS(B.DCMWalk):
    """Sequenceur quasi-statique de montee d'escalier au-dessus de l'executeur QP."""''',
'''class StairQS(B.DCMWalk):
    """Quasi-static stair-climbing sequencer layered on top of the QP executor."""'''))
R.append(("        B.N_STEPS = 2; B.STEP_LEN = 0.1        # plan plat de la base : ignore",
           "        B.N_STEPS = 2; B.STEP_LEN = 0.1        # base class's flat plan: ignored"))
R.append(("        self.fz0 = float(self.fz)              # hauteur cheville pied au sol (~0.104)",
           "        self.fz0 = float(self.fz)              # ankle height of the grounded foot (~0.104)"))
R.append(("        # --- drapeaux executeur (control() les lit) ---",
           "        # --- executor flags (read by control()) ---"))
R.append(('''        # w_foot_stance RELEVE 500->1400 (POLISH #6a) : le pied d'APPUI doit rester
        # TOTALEMENT plat/pose pendant le swing. A 500 (vs swing 2500) la reaction du
        # pied qui se leve fait BASCULER la semelle d'appui sur une arete (rocking, le
        # pied "quitte le sol un peu"). On raidit la tache pied-porteur (position+ori du
        # sole a plat) sans toucher a la trajectoire de swing ni au CoM (2500). Bonus :
        # sur le palier haut, un pied d'appui plus ferme donne plus d'appui a la base
        # contre le tangage de la jonction finale (aide la recuperation au sommet).''',
'''        # w_foot_stance RAISED 500->1400 (POLISH #6a): the STANCE foot must stay
        # FULLY flat/planted during the swing. At 500 (vs swing 2500) the reaction of
        # the lifting foot makes the stance sole TIP onto an edge (rocking, the
        # foot "leaves the ground a little"). We stiffen the load-bearing foot task
        # (flat sole position+orientation) without touching the swing trajectory or the CoM (2500). Bonus:
        # on the top landing, a stiffer stance foot gives the base more support
        # against the pitch of the final junction (helps recovery at the top).'''))
R.append(("        self.w_foot_swing = 2500.0; self.w_foot_stance = 1400.0  # swing < CoM (equilibre prioritaire)",
           "        self.w_foot_swing = 2500.0; self.w_foot_stance = 1400.0  # swing < CoM (balance takes priority)"))
R.append(("        # --- reglages sequenceur ---",
           "        # --- sequencer settings ---"))
R.append(("        self.VTOL = 0.03                       # vitesse CoM max pour finir le transfert (m/s)",
           "        self.VTOL = 0.03                       # max CoM speed to finish the transfer (m/s)"))
R.append(('''        # SEEK d'atterrissage : si l'arc finit sans contact, on presse le pied vers le
        # giron (jusqu'a SEEK_MAX sous la cible) au lieu de basculer en DONE pied en l'air.''',
'''        # landing SEEK: if the arc finishes without contact, the foot is pressed toward
        # the tread (up to SEEK_MAX below the target) instead of switching to DONE with the foot airborne.'''))
R.append(('''        # biais AVANT de la cible CoM : la cheville n'est pas au centre de la semelle
        # (talon −0.125 / pointe +0.075) et la montee tend a laisser le CoM EN RETRAIT
        # (bascule arriere en haut). On vise legerement vers la pointe.''',
'''        # FORWARD bias of the CoM target: the ankle is not at the center of the sole
        # (heel -0.125 / toe +0.075) and climbing tends to leave the CoM TRAILING
        # (tips backward at the top). We aim slightly toward the toe.'''))
R.append(("        # --- etat physique des pieds (positions plantees courantes) ---",
           "        # --- physical foot state (current planted positions) ---"))
R.append(("        # --- etat machine ---",
           "        # --- state machine ---"))
R.append(("    # ---------- geometrie escalier ----------",
           "    # ---------- stair geometry ----------"))
R.append(('''    def _build_moves(self):
        """Sequence de deplacements de pieds : approche a petits pas (pied bride
        avant la 1ere contremarche) puis montee step-together (centre du giron)."""''',
'''    def _build_moves(self):
        """Sequence of foot moves: small-step approach (foot clamped
        before the 1st riser) then step-together climb (center of the tread)."""'''))
R.append(("        foots = []                              # (side, x) ; index 0 = appui initial (pas un move)",
           "        foots = []                              # (side, x); index 0 = initial stance (not a move)"))
R.append(("        self._n_approach = len(foots)           # nb de footholds d'approche (incl. initial)",
           "        self._n_approach = len(foots)           # number of approach footholds (incl. initial)"))
R.append(('        print("[plan-QS] %d moves (appui initial D@x=%.3f) :" % (len(self.moves), rx))',
           '        print("[plan-QS] %d moves (initial stance R@x=%.3f):" % (len(self.moves), rx))'))
R.append(('''    def _arc(self, frm, tgt, s):
        """Trajectoire cheville UP-OVER-DOWN : monte vertical au-dessus du depart,
        avance a hauteur de pic (semelle au-dessus du nez), descend sur le giron."""''',
'''    def _arc(self, frm, tgt, s):
        """UP-OVER-DOWN ankle trajectory: rises vertically above the start,
        advances at peak height (sole above the nose), descends onto the tread."""'''))
R.append(("    # ---------- machine a etats ----------",
           "    # ---------- state machine ----------"))
R.append(("            st_live = np.asarray(self.d.xpos[self.stance][:2], float)   # pied VIVANT",
           "            st_live = np.asarray(self.d.xpos[self.stance][:2], float)   # LIVE foot"))
R.append(("            tgt = np.array([st_live[0], st_live[1]])                    # CoM sur l'appui vivant",
           "            tgt = np.array([st_live[0], st_live[1]])                    # CoM over the live stance"))
R.append(("            settled = self._com_speed < self.VTOL      # CoM ARRETE sur l'appui (anti-coast)",
           "            settled = self._com_speed < self.VTOL      # CoM STOPPED over the stance (anti-coast)"))
R.append(("            # APPUI SIMPLE : CoM strictement au-dessus du pied porteur (pas de biais avant).",
           "            # SINGLE SUPPORT: CoM strictly above the load-bearing foot (no forward bias)."))
R.append(('''            # SETTLE FINAL — recuperation confinee a DONE (moves 0-7 = montee 3/3 propre,
            # intouches). Le dernier pied de swing (m08) peut ne PAS s'etre pose : sur le
            # palier haut le bassin tangue et la cible mondiale est ratee (GRF=0, pied en
            # l'air) -> l'ancienne version basculait en arriere sur UN seul pied.
            # Deux temps :
            #   (1) pied libre PAS pose -> on le PRESSE vers le giron (seek z) ET on tient
            #       le CoM ferme sur le pied D'APPUI (deja sur le palier) + biais avant :
            #       ca DE-tangue la base et fait descendre le pied jusqu'au contact ;
            #   (2) deux pieds au sol -> CoM au centre du polygone sommet.''',
'''            # FINAL SETTLE — recovery confined to DONE (moves 0-7 = clean 3/3 climb,
            # untouched). The last swing foot (m08) may NOT have landed: on the
            # top landing the pelvis pitches and the world-frame target is missed (GRF=0, foot
            # airborne) -> the old version tipped backward onto a SINGLE foot.
            # Two stages:
            #   (1) free foot NOT down -> PRESS it toward the tread (seek z) AND hold
            #       the CoM firmly over the STANCE foot (already on the landing) + forward
            #       bias: this UN-pitches the base and brings the foot down to contact;
            #   (2) both feet on the ground -> CoM at the center of the top support polygon.'''))
R.append(("        for fb in self.feet:                    # suivi GRF (pic)",
           "        for fb in self.feet:                    # GRF tracking (peak)"))
R.append(('''    print("[scene] scene_stairs.xml genere : x0=%.2f tread=%.2f h=%.2f n=%d"
          % (a.x0, a.tread, a.hriser, len(risers)))''',
'''    print("[scene] scene_stairs.xml generated: x0=%.2f tread=%.2f h=%.2f n=%d"
          % (a.x0, a.tread, a.hriser, len(risers)))'''))
R.append(('''    # CoM = PRIORITE en appui simple. Le trace #1 a montre que les taches du pied de
    # swing (W_SWING/w_foot_swing = 6000) ECRASENT la tache CoM (400) -> le CoM derive
    # hors du pied porteur (demi-semelle ±0.06 m) et le pendule inverse (omega~3.3/s)
    # diverge. On donne au CoM une autorite comparable au swing + gains raidis/amortis,
    # et on abaisse le poids du swing pour qu'il ne vole pas l'equilibre.''',
'''    # CoM = PRIORITY in single support. Trace #1 showed that the swing-foot
    # tasks (W_SWING/w_foot_swing = 6000) OVERWHELM the CoM task (400) -> the CoM drifts
    # off the load-bearing foot (half-sole ±0.06 m) and the inverted pendulum (omega~3.3/s)
    # diverges. The CoM is given authority comparable to the swing + stiffened/damped gains,
    # and the swing weight is lowered so it doesn't steal the balance.'''))
R.append(('''    # BASSIN A PLAT : W_ORI par defaut (40) est ridicule vs CoM/swing (2500) -> le
    # bassin part en tangage pendant les grands swings de montee, ce qui PROJETTE le
    # pied de swing au-dela de sa cible (collision, cf trace stair-3). On raidit fort.
    # 600 = valeur de la montee 3/3 PROPRE (QS #5). Monter a 1100 n'a PAS aide et a
    # coincide avec la regression laterale (QS #5b) -> on garde 600. Le pitch -19 deg
    # transitoire du swing-jonction m08 est rattrape par le SEEK (le pied SE POSE) +
    # recuperation DONE a DEUX pieds, pas par un raidissement orientation global.''',
'''    # FLAT PELVIS: the default W_ORI (40) is negligible vs CoM/swing (2500) -> the
    # pelvis pitches during the large climbing swings, which OVERSHOOTS the
    # swing foot past its target (collision, cf trace stair-3). We stiffen it heavily.
    # 600 = the value of the CLEAN 3/3 climb (QS #5). Raising to 1100 did NOT help and
    # coincided with the lateral regression (QS #5b) -> 600 is kept. The transient
    # -19 deg pitch of the swing-junction m08 is caught by SEEK (the foot LANDS) +
    # two-foot DONE recovery, not by a global orientation stiffening.'''))
R.append(('                    print("[viewer] chute t=%.2f s (%s) — relance" % (c.t, c.state))',
           '                    print("[viewer] fall t=%.2f s (%s) — restarting" % (c.t, c.state))'))
R.append(('            if a.trace: print("[trc] ===== CHUTE ====="); trc(c)',
           '            if a.trace: print("[trc] ===== FALL ====="); trc(c)'))
R.append(('''    print("S3-QS  escalier x0=%.2f tread=%.2f h=%.2f | moves=%d rate=%.2f tswing=%.2f seed=%s"
          % (a.x0, a.tread, a.hriser, len(c.moves), a.rate, a.tswing, a.seed))''',
'''    print("S3-QS  stairs x0=%.2f tread=%.2f h=%.2f | moves=%d rate=%.2f tswing=%.2f seed=%s"
          % (a.x0, a.tread, a.hriser, len(c.moves), a.rate, a.tswing, a.seed))'''))
R.append(('''    print("  outcome    : %s%s | etat final=%s"
          % ("UPRIGHT" if upright else "FELL",
             "" if fell_at is None else "  (t=%.2f s)" % fell_at, c.state))''',
'''    print("  outcome    : %s%s | final state=%s"
          % ("UPRIGHT" if upright else "FELL",
             "" if fell_at is None else "  (t=%.2f s)" % fell_at, c.state))'''))
R.append(('''    print("  montee     : %d/%d marches (max appui %.2f m) | gain CoM z %.3f m -> %s"
          % (n_climbed, len(risers), c._maxclimb, climb, "SUCCESS" if success else "FAIL"))''',
'''    print("  climb      : %d/%d steps (max stance %.2f m) | CoM z gain %.3f m -> %s"
          % (n_climbed, len(risers), c._maxclimb, climb, "SUCCESS" if success else "FAIL"))'''))
R.append(('    print("  avancee x  : %.2f m | move atteint mi=%d/%d" % (dist, c.mi, len(c.moves) - 1))',
           '    print("  x progress : %.2f m | move reached mi=%d/%d" % (dist, c.mi, len(c.moves) - 1))'))
R.append(('''    print("  clearance  : min %.3f m | mean %.3f m | franchissements=%d"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clear"])))''',
'''    print("  clearance  : min %.3f m | mean %.3f m | crossings=%d"
          % (np.nanmin(cl), np.nanmean(cl), len(c.log["clear"])))'''))
R.append(('''        # NB : ajout 2026-08-06 de cles de REPORTING pour s3_batch.py. Bloc post-boucle,
        # apres la fin de la simulation -> strictement inerte vis-a-vis de la dynamique
        # et du controleur (aucune ligne de control()/update() touchee). La config gelee
        # du succes 3/3 (QS #5c + w_foot_stance=1400) reste intacte.''',
'''        # NB: 2026-08-06 addition of REPORTING keys for s3_batch.py. Post-loop block,
        # after the simulation ends -> strictly inert with respect to the dynamics
        # and the controller (no line of control()/update() touched). The frozen config
        # of the 3/3 success (QS #5c + w_foot_stance=1400) remains intact.'''))

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
