# Audit — Loi de timing S2 vs implémentation officielle Khadiv2020
*2026-07-15 | Référence : machines-in-motion/reactive_planners (`src/dcm_vrp_planner.cpp`, BSD-3, NYU/MPI — groupe Righetti/Khadiv) vs `talos_dcm_walk_timing.py` (DCMWalkT)*

## 1. Formulation de référence (DcmVrpPlanner)

QP résolu à **chaque cycle de contrôle**, 9 variables : `u_x, u_y` (lieu du pas
suivant, relatif à l'appui), `τ = e^{ωT}` (durée exponentiée → contraintes
linéaires), `b_x, b_y` (offset DCM au prochain touchdown), 4 slacks.

- **Coût** : tracking quadratique de valeurs nominales (`l_nom, w_nom, τ_nom,
  bx_nom, by_nom`) dérivées de la vitesse désirée `v_des`.
- **Égalités (2)** : récursion DCM — `u + b = (ξ_mes − p_appui)·e^{−ωt}·τ`
  (cohérence lieu/durée/offset, canaux x et y **couplés par τ**).
- **Inégalités (10)** : boîtes sur longueur/largeur de pas (asymétriques selon
  le côté d'appui), `τ ∈ [τ_min, τ_max]` avec `τ_min` relevé dynamiquement à
  `e^{ω·max(t_min, t_écoulé + t_min_restant)}`, bornes de **viabilité** sur
  `b` (`bx = l/(τ−1)`, `by_in/by_out` asymétriques — l'asymétrie côté
  appui/swing est structurelle).
- **Fallback** : QP infaisable → valeurs nominales.

## 2. Notre implémentation (DCMWalkT, lignes 296–317 + 356–380)

**Analytique, pas de QP.** Découplage timing/placement :

- **Timing** : loi de divergence fermée `d(τ) = d0·e^{ωτ}` →
  `t_rem = ln(d*/d)/ω` par canal ; latéral : `d* = ly + ξ_s` ; sagittal :
  plafond `SAG_MAX` ; `T_k = clip(τ + min(t_lat_rem, t_sag_rem), T_MIN, T_MAX)`.
  `d ≤ 0` (DCM du mauvais côté) → poser immédiat (`_force_land`).
- **Placement** : capture point analytique `g_y = ξ_prédit(t_go) + sgn·off_lat`
  avec bornes géométriques `[min_sep, MAX_SEP]` relatives au pied d'appui réel ;
  **sagittal : foothold du plan figé** (pas d'ajustement `u_x`), mode
  récupération = pas latéral pur, progression sagittale gelée.
- **Re-planification par pas** : `d0` mesuré à chaque entrée de pas.

## 3. Correspondance

| Élément | Référence | Nôtre | Verdict |
|---|---|---|---|
| Loi de divergence DCM (LIP, e^{ωt}) | oui (via τ) | oui (fermée) | ✅ identique |
| Timing raccourci si DCM fuit | τ_min dynamique | T_k = τ + t_rem, plancher T_MIN | ✅ équivalent |
| Timing allongé si DCM sage | jusqu'à τ_max | jusqu'à T_MAX | ✅ équivalent |
| Valeur de commutation latérale | by_nom (l_p, v_des) | d* = ly + ξ_s | ✅ même rôle, paramétrisation simplifiée |
| Asymétrie côté appui | bornes by_in/by_out | min_sep anti-croisement (géométrique) | ⚠️ partielle |
| Ajustement lieu sagittal u_x | oui (QP joint) | **non** (plan figé + recovery latéral pur) | ❌ absent |
| Couplage lieu↔durée (τ dans l'égalité) | oui | non (découplé) | ❌ absent |
| Bornes de viabilité sur b | oui | non (bornes géométriques) | ❌ absent |
| Optimisation jointe multi-objectifs | QP 9 var + slacks | greedy min(2 canaux) | ❌ simplification |
| Recovery explicite entrée dégénérée | non (fallback nominal) | oui ([recov] latéral pur) | ➕ ajout de notre cru |

## 4. Conséquences pour la thèse

1. **La formulation « in the lineage of Khadiv et al. » (Ch5 §5.5) est
   correcte et doit rester** — ne jamais écrire « implémente Khadiv2020 » :
   nous implémentons la *loi de divergence temporelle* de cette lignée, pas le
   QP joint lieu+durée.
2. **L'audit CONFIRME la lecture architecturale de §5.8** : l'absence
   d'ajustement sagittal (`u_x` figé, recovery latéral pur) chez nous — alors
   que la référence l'optimise — est exactement le mécanisme manquant que la
   campagne S5 mesure (I₅₀ sagittal 20–30 N·s vs latéral 45–50+). La
   prescription « sagittal analogue of the lateral timing law » de la
   criticalinsight §5.8 correspond, en termes de la référence, à réintroduire
   `u_x` et le couplage par τ.
3. **Validité de l'ablation `--no-timing`** : `use_timing=False` gèle
   uniquement la mise à jour de T_k (T_k = T_nom) ; `use_cp_swing` (placement
   capture point) reste actif → l'ablation isole proprement la contribution
   du *timing* seul, séparée du *placement* — plus propre que dans la
   référence où les deux sont dans le même QP et non séparables.
4. **Candidat future work chiffrable** : remplacer le greedy 2-canaux par le
   QP 9-variables de la référence (quadprog déjà dans le pipeline) = extension
   à effort borné, directement comparable.

## 5. Limites de l'audit

Comparaison sur `dcm_vrp_planner.cpp` (master, consulté 2026-07-15) ;
le wrapper `DcmReactiveStepper` (gestion phases/end-effectors) n'a pas été
audité — hors périmètre : notre machine à pas est différente par conception.
La référence tourne sur Bolt/Solo (pas TALOS) ; aucune comparaison numérique
directe n'est donc possible, seulement structurelle.
