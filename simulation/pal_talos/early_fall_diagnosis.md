# S2 — Diagnostic des chutes précoces (passe robustesse, option C)

*Session 2026-07-08. Données : batch N=50 (`s2_batch/s2_seed*.npz`) + reconstruction
des perturbations initiales. Complément de `RESULTS.md` / `WBC_planning_diagnosis.md`.*

## Question
Le batch N=50 donne 70 % de succès ; parmi les 15 chutes, 7 sont **précoces**
(t < 16 s, < 2 m) : seeds 3, 15, 30, 32, 38, 48, 49. Peut-on remonter le taux
par un durcissement ciblé, ou la fragilité est-elle structurelle ?

## 1. La condition initiale ne prédit PAS la chute précoce
`--seed` injecte un bruit unique et reproductible sur les vitesses articulaires
(`np.random.default_rng(seed).normal(0, 0.01, nv=50)` sur `d.qvel`). En corrélant
ce kick avec l'issue des 50 seeds :

- magnitude totale du kick **identique** entre groupes : succès 0.069, chutes
  précoces 0.070, chutes tardives 0.070 ; `corr(|kick|, t_chute) = −0.01`.
- même seed 15 — seul faller **reproductible** (0.66 m à 9.4 s en N=20 *et* N=50)
  — a un kick base modeste (0.021), rien d'extrême.

**Conclusion :** l'effondrement précoce n'est pas « un gros choc initial ». C'est
cohérent avec le résultat P1e (l'issue est en partie fixée par une bifurcation
discrète en aval, pas par l'entrée).

## 2. Toutes les chutes = divergence terminale du DCM
La signature commune est nette : l'erreur DCM `|ξ − ξ_ref|` explose à
**870–1060 mm** dans les derniers instants, contre **1–3 mm** pour les succès
(qui se referment proprement). La chute EST une divergence du composant divergent.

## 3. Deux mécanismes distincts (durées de pas)
En regardant les 8 derniers pas avant la chute :

| Mécanisme | Seeds | Signature |
|---|---|---|
| **Chattering** | 3, 30, 32, 48 | 5–6 pas/8 sous 0.20 s, pas d'urgence à **0.075–0.09 s** (≪ T_MIN=0.24) → trébuchement en pas ultra-courts |
| **Divergence propre** | 15 (reproductible) | 0 pas court : les 8 derniers pas sont **normaux** (0.26–0.35 s) et le DCM diverge quand même |

Les pas ultra-courts ne sont pas une cause mais un **symptôme** : le corps bascule,
le pied d'appui frappe le sol très tôt (touchdown événementiel), le plan se recale
sur un mauvais contact → spirale. Seed 15 montre qu'on peut chuter **sans** chattering.

## 4. L'enveloppe de capture n'est pas le facteur limitant
Mesure de l'excursion DCM maximale que les runs **survivent** :

- succès : excursion `ξ` max survécue = **286 ± 66 mm** (jusqu'à **404 mm**) puis retour.
- fallers, juste avant l'emballement (fenêtre 80–90 % de l'épisode) : seulement
  **72–147 mm** — *bien à l'intérieur* de la bande survivable — puis explosion à ~1000 mm.

**Les fallers divergent depuis un état plus calme que ceux que les succès rattrapent
couramment.** La chute n'est donc pas une excursion « trop grande » franchissant une
limite lisse : c'est un **emballement soudain déclenché par un événement**, parti d'un
état ordinaire.

## 5. Verdict pour la passe de robustesse
Les trois leviers de tuning classiques sont **écartés par les données** :
- ce n'est pas la condition initiale (§1, corrélation nulle) ;
- ce n'est pas l'enveloppe de capture / la taille de pas (§4, l'enveloppe est déjà
  suffisante — 286 mm survécus) ;
- le chattering est un symptôme aval, pas la racine (§3, seed 15).

La racine est la **bifurcation discrète** de P1e : un événement rare (bascule
d'ensemble actif du QP / inclusion de contact au ras du seuil) déclenche un
emballement DCM soudain depuis un état sain. Un réglage de gains/offsets/timing ne
peut pas supprimer un déclencheur discret ; il ne fera que déplacer les seeds qui
tombent (déjà observé N=20 → N=50). Le remède de fond reste le **co-design** identifié
précédemment : durée de double-appui explicite dans la récursion DCM (le plan suppose
un basculement d'appui instantané), pour retirer la marge-mince qui rend le système
sensible à l'événement. → *future work*, pas un patch de tuning.

**Mitigation testable (si l'on veut tenter malgré tout) :** un détecteur d'emballement
précoce sur la *vitesse* de croissance de `ξ` (pas son niveau) déclenchant un arrêt
stabilisé (stopper la progression sagittale, élargir l'appui) AVANT le point de
non-retour. Gain incertain — l'emballement est rapide (§4) — mais falsifiable en un
run batch. À défaut, accepter le plafond ~70 % comme borne de cette architecture
flat-foot DCM/ZMP et le documenter.

## Mini insight critique
Le protocole d'évaluation a fait exactement son travail : il a transformé trois
intuitions de tuning (« mauvais seed », « pas trop petits », « enveloppe trop
étroite ») en trois hypothèses *réfutées* par la mesure, et convergé avec l'analyse
de reproductibilité (P1e) vers une cause unique et structurelle. La leçon n'est pas
que le contrôleur est mauvais — son enveloppe de capture (≈ 290 mm) est saine — mais
que **piloter un cycle limite à marge mince près des frontières de contraintes rend
l'issue décidée par un événement discret**, hors de portée du réglage. C'est un
argument de conception (Ch6), pas un bug à corriger.
