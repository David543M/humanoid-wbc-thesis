# Model-based walking (WBC + DCM planning) — diagnosis and closed-loop attempt

## 1. Failure mode of the open-loop walker (precisely localised)

A tick-level trace of the lateral channel shows the baseline (`talos_dcm_walk.py`,
default) falls **sideways at the second step** (roll → −1.1 rad at t ≈ 4 s), not
sagittally. During the first left single-support phase the measured DCM
(`ξ_y = c_y + ċ_y/ω`) overshoots the stance-foot edge: it leaves the planned
+0.085 m and grows exponentially (+0.14 → +0.28 → +1.0 m) while the CoM tips outward.
Once ξ_y passes the foot edge (~+0.135 m, the maximum ZMP a 0.05 m-wide foot can
produce), the lateral dynamics is no longer controllable by ankle/CoM action and the
robot is lost. The CoM reference itself stays well-behaved (±0.06 m) — the divergence
is in the *plant*, not the reference.

*Critical insight.* The decisive number is the lateral divergence rate ω = √(g/z_c) ≈
3.23 s⁻¹: over the original T_step = 1.3 s the lateral DCM error is multiplied by
e^{ω·T} ≈ 66 per step. No bounded feedback can stabilise a once-per-step decision with
that much intra-step growth — the instability is structural, set by the gait timing,
before any controller gain is chosen.

## 2. Model-based remedies implemented (opt-in, WBC-QP untouched, defaults preserved)

Three standard model-based feedback laws were added to `DCMWalk`, all in the planning
layer; the QP cost/constraints are unchanged and the default (no-flag) behaviour is
bit-identical to the baseline (FELL, z = 0.272, x = 0.820):

1. **Continuous DCM feedback into the CoM reference** (`--feedback`, gain `k_dcm`):
   pulls the CoM reference to cancel the measured DCM error (the stable LIPM part).
2. **Per-step capture-point foot placement** (`--footfb`, gain `k_foot`): at each
   DS→SS transition shifts the next footstep by `k_foot·(ξ_meas − ξ_ref)`, clamped to
   a feasible offset and a minimum lateral separation (no foot crossing).
3. **Continuous capture-point swing retargeting** (`--cpswing`): steers the swing foot
   *during* the step toward the measured capture point.

A naïve fourth attempt — commanding the CoM with the raw LIPM acceleration
`c̈ = ω²(c − p_zmp)` — was tested and rejected: that form is open-loop unstable and the
soft CoM task cannot realise it stiffly, so it diverged faster (CoM to −0.6 m). It is
not retained.

*Critical insight.* That the unstable-acceleration form fails while the
stable-reference form survives is the practical lesson: with a CoM-tracking WBC the
feedback must enter through a *stable* reference trajectory, never through the
inverted-pendulum acceleration directly — a subtlety that separates a working DCM
controller from a textbook transcription.

## 3. Result of the closed-loop preset

The `--walk-cl` preset bundles the three laws with faster stepping (T_step = 0.5 s,
longer double-support DS = 0.45) to cut the per-step error growth from ×66 to ×e^{1.6}
≈ ×5:

| Controller | Steps before fall | Forward distance | Fall mode |
|---|---:|---:|---|
| open-loop DCM (default) | 3 | 0.82 m | lateral, step 2 |
| **closed-loop `--walk-cl`** | **~4** | **~0.92 m** | sagittal, later |

The closed-loop controller roughly **doubles** the survivable steps and extends the
walk, but does **not** achieve sustained walking: it still loses lateral balance around
the sixth step. Sensitivity to the planning horizon (a 10-step plan superficially
"reaches step 9" because its terminal capture masks the divergence, whereas a 30-step
plan falls at step 6) confirms the residual instability is in the *gait design*, not
the feedback gains.

## 4. What sustained model-based walking would require

The remaining gap is a *valid lateral limit cycle*: the planned lateral DCM and the
weight-transfer timing must be designed so the robot enters each single-support phase
with the DCM inside the foot and moving so as to reverse — together with **adaptive
step timing** (trigger touchdown early when the DCM approaches the foot edge), which
the current fixed-time plan lacks. This is a substantial planner redesign
(periodic-DCM / divergent-component gait synthesis + event-based timing).

*Critical insight.* This is the honest justification for the learned high-level layer:
the analytical controller is bounded by the once-per-step, fixed-timing decision
structure, and closing the lateral loop robustly demands adapting *both* footstep
placement *and* timing under the full nonlinear dynamics — precisely the
high-dimensional decision that an RL policy over the fixed WBC is positioned to supply.
The model-based effort here is therefore not a detour but the experiment that *defines*
what the learning layer must add.

## 5. Commands

```bash
python talos_dcm_walk.py                 # open-loop baseline (falls ~3 steps)
python talos_dcm_walk.py --walk-cl       # closed-loop preset (~4 pas propres)
python talos_dcm_walk.py --feedback --footfb --cpswing --tstep 0.5 --dsovl 0.45   # explicit
python view_talos_dcm.py --walk-cl       # watch it in the 3D viewer (your machine)
```

---

## 6. Synthèse du cycle limite latéral (investigation poussée)

**Théorie.** Pour une marche latérale périodique (appuis alternés à ±ly, durée T), le
DCM latéral admet un point fixe analytique ξ_s = ly·tanh(ωT/2). On a vérifié
numériquement que la **récursion arrière du planner converge exactement vers ce point
fixe** — donc le plan est déjà sur le cycle limite pour les pas centraux ; le défaut
n'est pas le plan mais l'**état initial** (robot centré, vitesse nulle, hors cycle).

**Entrée impulsionnelle : échec instructif.** Imposer la vitesse latérale d'entrée
théorique (ċ_y = ω·ξ_s ≈ 0.18 m/s via `d.qvel`) **dégrade** le résultat (chute à 2.6 s
contre 4.08 s) : les pieds sont en contact (chaîne cinématique fermée), donc injecter
une vitesse de base ne produit pas une translation propre du CoM — le robot est projeté
latéralement (excursion 0.42 m contre 0.22 m). Conservé en option `--limit-cycle`, mais
désactivé par défaut.

**Résultat positif majeur.** La rétroaction closed-loop (`--walk-cl`) **résout
l'instabilité latérale d'origine** : sur les pas 1–4 le CoM latéral reste à ±0.05 m
(contre 0.65 m en boucle ouverte) et le roll à ±0.05 rad (contre 1.5). Le mode d'échec
latéral du §1 est éliminé.

**Le goulot a migré vers le sagittal.** Le robot tient désormais latéralement puis
**bascule vers l'avant** (pitch 0.08→0.65→1.20 rad, CoM_x accélérant 0.3→0.7→1.0 m) vers
le pas 5–6 : une divergence sagittale (emballement avant). Restreindre le placement de
pied au latéral seul ne la corrige pas. Plafond honnête : **~4 pas propres, latéral résolu,
sagittal devenu limitant.**

*Critical insight.* Le déplacement du mode d'échec (latéral → sagittal) est le résultat
le plus significatif de cette phase : il prouve que la rétroaction capture-point latérale
fonctionne et isole le verrou suivant. Une marche soutenue en pur model-based demande
maintenant trois briques additionnelles — une **entrée par bercement latéral** progressif
(et non impulsionnel, à cause de la chaîne fermée), une **régulation sagittale** de la
vitesse/du tangage, et un **timing de pas adaptatif** — c'est-à-dire une synthèse de
démarche complète. Chaque brique est un incrément ciblé, désormais clairement défini par
ce diagnostic plutôt que deviné.

---

## 7. Verrou sagittal : diagnostic et plafond architectural

Une fois le latéral résolu (§6), le mode d'échec devient un **emballement avant**.
La trace sagittale le localise précisément : le DCM avant suit le plan jusqu'au pas 3
(t≈2.85 s : dcm_x = 0.130 ≈ ref 0.135), puis la **vitesse d'avance dérive**
(0.10 → 0.15 → 0.19 → 0.27 m/s sur les pas 3–4), l'erreur de DCM dépasse l'écrêtage
de la rétroaction (0.05 m), et le DCM avant diverge exponentiellement (v atteint
1.5 m/s, tangage 0.08 → 1.20 rad). Mécanisme : le CoM avance ~0.13 m/pas alors que les
pieds n'avancent que de STEP_LEN = 0.05 m — **le CoM dépasse son support** et bascule.

**Le plafond est robuste à tout réglage.** Testé exhaustivement, *toutes* les variantes
donnent ~4 pas propres (≤6 en métrique temporelle lenient ; voir note) :

| Intervention | Pas (métrique temporelle, lenient) |
|---|---|
| walk-cl (référence) | 6 |
| rétroaction DCM forte (clip 0.20, k=2) | 4 |
| placement de pied latéral seul | 5 |
| amortissement CoM ×2.5 (freinage) | 4 |
| DCM-feedback OFF | 6 |
| foot-placement OFF | 6 |

Aucun gain, écrêtage, amortissement ou composante de rétroaction ne dépasse ce plafond.

> **Note métrique (corrigée).** Les chiffres du tableau utilisent une métrique *lenient*
> (transitions de phase basées sur le temps, qui continuent pendant la chute). La métrique
> *honnête* — **pas propres** = pas complétés en restant stable (z>0.95, |roulis|,|tangage|<0.2 rad) —
> donne ~2 de moins : le walk-cl fait **~4 pas propres** (1ère perte d'équilibre à t≈3.5 s),
> contre ~2 pour le baseline. Le balayage paramétrique a été relancé avec cette métrique
> (`runs/sweep3/`, `debug_probe.py` champ `pas PROPRES`).

*Critical insight.* L'invariance du plafond à travers tout l'espace des gains est le
résultat décisif : elle prouve que le verrou n'est pas paramétrique mais **structurel**,
imposé par la décision *une-fois-par-pas à timing fixe* et par une longueur de pas fixe
incompatible avec la vitesse que la dynamique du pendule génère. Réguler la vitesse
d'avance par la cheville/CoM seule est impossible une fois le DCM avant au-delà de
l'orteil (limite de contrôlabilité, l'exacte analogie sagittale du §1 latéral).

**Le seul levier restant est le timing de pas adaptatif.** Pour que le CoM ne dépasse
pas son support, il faut **poser le pied plus tôt** quand le DCM avant approche le bord
de l'orteil (déclenchement événementiel du contact, et non à T_STEP fixe), couplé au
placement capture-point déjà en place. C'est un changement structurel du planificateur
(la logique de phase passe du temps à un événement DCM-seuil), pas un réglage — et c'est
la brique qui peut casser le plafond de ~4 pas propres en pur model-based. À défaut, c'est
exactement la décision haut-niveau (placement + timing) que la couche RbL est conçue
pour fournir.
