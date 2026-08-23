# S5 — Perturbation Robustness : Design Document (impulsions pendant la marche)

*Kickoff : 2026-07-10 | Statut : conception + première implémentation | Portée : poussées impulsionnelles sur le bassin pendant la marche S2*
*Satellite de the thesis (S5), §9.2 (métriques — flag « perturbation recovery » OUVERT), §13 (flow)*
*Pipeline cible : `pal_talos/` (MuJoCo natif + ProxQP), au-dessus de `talos_dcm_walk_timing.py` (S2) INCHANGÉ*

> **But du document.** Spécifier le dernier scénario du protocole S1–S5 : marche plane sous
> impulsions externes aléatoires, métrique principale *recovery rate* (the thesis methodology chapter).
> Décisions David (2026-07-10) : **(1) marcheur = S2 dynamique** (`DCMWalkT`, config gelée) —
> l'adaptation de timing (Khadiv2020) est précisément le mécanisme de récupération à tester ;
> **(2) calibration = balayage de magnitude** — la courbe recovery-vs-impulsion EST la
> calibration, ce qui ferme le flag Ch3 §3.6 (« seuil recovery : calibré conjointement avec la
> distribution d'impulsion S5 ») par la mesure plutôt que par un chiffre postulé.

---

## 1. Objectif et positionnement

S5 complète le protocole : S1 a validé l'équilibre statique sous poussées, S2–S4 la locomotion
et la loco-manipulation *sans* perturbation externe. S5 mesure la **capacité de rejet dynamique**
du couple {planificateur DCM + timing adaptatif + QP-WBC} : une impulsion sur le bassin déplace
le DCM mesuré, et la récupération passe par les mécanismes déjà en place — re-planification du
foothold (capture point, `--cpswing`), raccourcissement du pas (timing adaptatif), mode `[recov]`
(pas latéral pur). S5 ne teste donc **aucun mécanisme nouveau** : il quantifie ceux de S2 sous
stress contrôlé.

Le choix du marcheur dynamique (vs séquenceur QS) découle de cette lecture : le QS n'a aucun
mécanisme réactif (machine à états à convergence de position), sa non-récupération à toute
poussée significative est prédictible et peu informative ; le marcheur S2, lui, incorpore
exactement la loi de timing dont la littérature (Khadiv2020) revendique la valeur en récupération
de poussée — S5 en est le test direct dans notre framework.

> **Insight critique.** S5 a une double fonction que les autres scénarios n'ont pas : (i) mesurer
> la robustesse, (ii) **produire la calibration qui manque à sa propre métrique**. Table 3.1 laisse
> le seuil de recovery ouvert faute de distribution d'impulsion défendable ; postuler maintenant
> « 150 N × 0.1 s » serait circulaire (critique reviewer « métriques arbitraires », §18). Le
> balayage inverse la logique : on mesure recovery(I) et on **rapporte la marge I₅₀** (impulsion à
> 50 % de récupération) comme résultat, au lieu de juger contre un seuil inventé. C'est la même
> discipline que S2/S3 : la grandeur honnêtement mesurable d'abord, le verdict binaire seulement
> là où un ancrage existe.

---

## 2. Le verrou méthodologique : perturber un marcheur à 70 %

Le substrat S2 chute déjà dans ~30 % des essais **sans** poussée (batch N=50, Wilson
[0.562, 0.809]), avec non-déterminisme par seed (bifurcation discrète QP). Attribuer une chute
post-poussée à la poussée est donc non trivial. Trois précautions structurent le protocole :

1. **Conditionnement** : un essai n'est *éligible* que si le robot est debout et en marche à
   l'instant de la poussée. Les chutes pré-poussée sont exclues du dénominateur (elles relèvent
   de S2, déjà caractérisé).
2. **Bras de contrôle mag = 0** : chaque cellule du balayage est comparée à des essais identiques
   (mêmes seeds, même code, même fenêtre) sans force. Le taux de « récupération » du bras contrôle
   estime la survie de base sur la fenêtre d'observation — la robustesse est lue en **Δ vs
   contrôle**, pas en absolu.
3. **Unité reproductible = taux agrégé** (héritage S2) : le sort d'un seed n'est pas reproductible,
   le taux par cellule l'est. Wilson CI par cellule, N ≥ 10–20 selon la largeur visée.

> **Insight critique.** La définition de « récupération » est le point le plus attaquable de tout
> S5 et doit être fixée AVANT la campagne (pré-enregistrement de facto) : *recovered = debout
> (base_z > 0.8 m) T_REC = 5 s après la fin de l'impulsion, parmi les essais éligibles*. Les
> grandeurs continues (temps de retour du DCM en bande, excursion max du DCM, nombre de pas
> `[recov]`) sont **rapportées** mais ne participent pas au verdict — les inclure dans le critère
> multiplierait les seuils non ancrés. Un reviewer peut contester 5 s ou 0.8 m ; il ne pourra pas
> contester que le critère était fixé avant les données.

---

## 3. Approche retenue : impulsion `xfrc_applied` sur le bassin, balayage magnitude × direction

**Décision : force externe constante F appliquée au corps `base_link` pendant DUR = 0.1 s
(impulsion I = F·DUR), déclenchée en régime établi à une phase aléatoire du pas, balayée en
magnitude et direction.** Le marcheur, le QP et la planification sont **bit-pour-bit S2**
(config gelée `--steps 70 --offlat 0.08 --dsovl 0.12 --tmin 0.24`) ; la perturbation est
injectée dans la boucle de simulation, pas dans le contrôleur — le WBC ne « sait » rien.

Mécanique (`talos_s5_perturb.py`, aucun fichier S2 modifié) :

1. **Injection** : `d.xfrc_applied[base, :3] = F · û` pendant `[t_push, t_push + DUR]`, remise à
   zéro ensuite. Force au CoM du corps base (pas de couple appliqué) — patron déjà validé en S4
   (`--payload-unknown`).
2. **Déclenchement** : la poussée part quand le pas courant `k` atteint `K_PUSH = 12` (cycle
   limite établi, loin du départ et de la fermeture), à une **phase aléatoire** du pas tirée de
   `default_rng(seed + 1000)` — découplée du bruit initial `qvel` (seed) pour que le bras contrôle
   partage exactement l'état initial. La phase aléatoire échantillonne SS et DS sans stratifier
   (la stratification appui-simple/double-appui est une ablation, pas le premier jet).
3. **Directions** : {+x (dos), −x (face), +y (gauche), −y (droite)} en repère monde. Le canal
   latéral est le mode d'échec dominant de S2 (divergence DCM latérale) — l'asymétrie
   sagittal/latéral attendue est un résultat en soi.
4. **Magnitudes** : F ∈ {50, 100, 150, 200, 250} N × 0.1 s → I ∈ {5, 10, 15, 20, 25} N·s
   (≈ 0.05–0.26 × m·v avec m ≈ 95 kg : vitesse induite ~0.05–0.26 m/s). Bornes choisies pour
   encadrer l'échelle de capturabilité du modèle (ξ_s ≈ 28 mm, enveloppe mesurée S2 : les succès
   survivent à des excursions DCM de ~286 ± 66 mm) ; à ajuster au bring-up si 0 % ou 100 % partout.
5. **Instrumentation** : tick de poussée, éligibilité, debout à t_push + T_REC, excursion DCM max
   post-poussée, temps de retour en bande (bande = max(2 × médiane pré-poussée, 50 mm) tenue
   0.5 s), nombre de pas `[recov]` post-poussée, + toutes les métriques S2 (com_err, ctrl_ms,
   t_steps, GRF de poser).

> **Insight critique.** Appliquer la force au niveau de la boucle de simulation plutôt que par un
> hook contrôleur est ce qui rend S5 propre : le contrôleur traverse la poussée avec la même
> information qu'un robot réel (rien), et la chaîne de récupération observée — divergence DCM
> mesurée → raccourcissement du pas → re-placement capture-point → éventuel pas `[recov]` — est
> entièrement imputable aux mécanismes S2 documentés. Le point faible assumé : une force au CoM du
> bassin est le *proxy standard* mais pas la seule perturbation réaliste (couple, poussée à
> l'épaule, sol glissant) ; généraliser la conclusion au-delà de ce proxy serait une sur-lecture,
> à dire en Ch6.

---

## 4. Paramètres de conception

Statut *design commitment* (non ancrés littérature sauf mention) :

| Paramètre | Valeur | Justification / à vérifier |
|-----------|--------|----------------------------|
| Durée d'impulsion DUR | 0.1 s | Convention dominante des push-recovery papers (impulsion courte vs T_pas = 0.5 s) |
| Magnitudes F | {50, 100, 150, 200, 250} N | Encadrement a priori de l'échelle de capturabilité ; raffiner au bring-up |
| Directions | ±x, ±y monde | 4 cellules ; diagonales = future work |
| K_PUSH | pas k = 12 | Régime établi (S2 : cycle stable dès k ≈ 5), marge avant fermeture (k = 69) |
| Phase | U[0, 1) × Tk, rng(seed+1000) | Échantillonne SS et DS ; découplée du bruit initial |
| T_REC | 5.0 s | ~10 pas nominaux ; vérifier qu'aucune chute « lente » ne dépasse la fenêtre |
| Bande de retour DCM | max(2 × méd. pré-push, 50 mm), tenue 0.5 s | Grandeur rapportée, PAS critère de verdict |
| N par cellule | ≥ 10 (balayage), ≥ 20 (cellules près de I₅₀) | Wilson exploitable là où ça compte |

> **Insight critique.** Le paramètre le plus lourd de conséquences est K_PUSH fixe : pousser
> toujours au même pas standardise la comparaison entre cellules mais n'échantillonne qu'un seul
> point du transitoire de marche. L'alternative (instant uniforme sur tout l'essai) confondrait
> effet de la magnitude et effet de la position dans le plan de marche. Le choix « K fixe + phase
> aléatoire » est le compromis : variance intra-cellule contrôlée, phase du cycle couverte. Si les
> résultats suggèrent une sensibilité forte à la phase (DS vs SS), la stratification devient la
> première ablation à exécuter.

---

## 5. Métriques S5

| Métrique | Définition opérationnelle | Rôle |
|----------|---------------------------|------|
| **Recovery rate** | debout à t_push + T_REC \| éligible ; Wilson 95 % par cellule (mag × dir) | **Verdict** (vs bras contrôle) |
| Marge I₅₀ | impulsion à 50 % de récupération, par direction (interpolation sur le balayage) | **Calibration** — ferme le flag Ch3 §3.6 |
| Excursion DCM max post-push | max ‖ξ_meas − ξ_ref‖ sur [t_push, t_push + T_REC] | Mécanisme |
| Temps de retour en bande | 1er instant où l'erreur DCM < bande tenue 0.5 s | Mécanisme |
| Pas `[recov]` post-push | compte des entrées dégénérées | Mécanisme |
| Transverses S2 | QP feasibility (> 99 %), ctrl (< 1 ms mean / 5 ms p99), CoM RMSE, GRF poser | Continuité protocole |

> **Insight critique.** La séparation verdict/mécanisme est ce qui évite à S5 le piège S4 (le gate
> EE < 5 cm jamais atteint avait transformé une métrique en faux critère) : UNE seule variable
> binaire décide (debout à T_REC), tout le reste explique. L'I₅₀ par direction est le livrable le
> plus durable du scénario : c'est un chiffre de marge en unités physiques (N·s), comparable entre
> contrôleurs et publiable indépendamment du taux de succès absolu du marcheur porteur.

---

## 6. Protocole de campagne

1. **Bring-up déterministe** : seed 0, une poussée +y 100 N, `--viewer` puis headless — vérifier
   l'injection (trace force), l'éligibilité, la chaîne de récupération visible. Livrable :
   `s5_run.npz`.
2. **Sonde d'échelle** : seed 0, ±y, F ∈ {50 … 250} — vérifier que la plage encadre bien la
   transition 100 % → 0 % ; ajuster les magnitudes sinon.
3. **Balayage complet** (`s5_batch.py`) : 4 directions × 5 magnitudes × N ≥ 10 seeds + bras
   contrôle (mag 0, mêmes seeds) → recovery rate + Wilson par cellule, courbes recovery(I) par
   direction, I₅₀ interpolé.
4. **Densification** : N → 20 sur les 2 cellules encadrant chaque I₅₀.
5. **Rédaction Ch5 §5.8** (patron §5.5–§5.7) + **mise à jour Ch3 §3.6** : remplacer le flag ouvert
   par la calibration mesurée (renvoi explicite S5).

> **Insight critique.** L'étape 5 boucle une dette méthodologique ouverte depuis 2026-05-04 et
> transforme une faiblesse déclarée (seuil non ancré) en contribution (calibration par la mesure).
> Le risque de campagne est le coût : 4 × 5 × 10 + contrôle ≈ 210+ essais ~30–60 s — dans le budget
> d'une nuit sur la machine conda, ou déléguable à Vertex AI (protocole Session 16). Ne PAS réduire
> le bras contrôle pour économiser : sans lui, le taux de récupération d'un marcheur à 70 % de base
> est ininterprétable.

---

## 7. Points de contact code

`talos_s5_perturb.py` — réutilise `DCMWalkT` et `apply_squat` de `talos_dcm_walk_timing.py`
(import, **zéro modification des fichiers S2**) :

- setup gait identique à S2 (`B.N_STEPS = 70`, `B.STEP_H = 0.04`, gains W/B gelés — copie assumée
  et commentée, même discipline que S3/S4) ; défauts CLI = config gelée S2.
- boucle de simulation : détection `c.k >= K_PUSH` → tirage de phase → fenêtre de force
  `d.xfrc_applied[base]` → remise à zéro ; balises tick (`i_push0`, `i_push1`) pour découper les
  logs `com_err`/`xi_err` par régime pré/post.
- bilan : éligibilité, recovered (T_REC), excursion DCM max, t_retour bande, pas `[recov]`
  post-push, métriques S2 standard ; npz enrichi (métadonnées poussée incluses).
- `s5_batch.py` : patron `s2_batch.py` (sous-processus par essai, agrégation Wilson par cellule,
  report md + csv + npz, garde-fous cmd.exe), + interpolation I₅₀ par direction.

**Commandes de bring-up :**
```
python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --save s5_run.npz
python talos_s5_perturb.py --seed 0 --pushmag 100 --pushdir +y --viewer
python talos_s5_perturb.py --pushmag 100 --pushdir +y --viewer --manual
                                     # poussée sur ENTRÉE/ESPACE + flèche (exploration)
python talos_s5_perturb.py --seed 0 --pushmag 0                      # bras contrôle
python s5_batch.py                                                    # balayage complet
```

NB exploration : le viewer affiche la poussée par une **flèche orange** (patron S1,
affichage 0,6 s / physique 0,1 s) ; `--manual` déclenche à la demande (ENTRÉE/ESPACE,
re-déclenchable — le viewer passif MuJoCo ne transmet pas les modificateurs, donc pas
de Ctrl+combinaison). Les métriques/verdict restent calés sur la 1ère poussée ;
`--manual` est un outil qualitatif, jamais utilisé en campagne.

> **Insight critique.** S5 est le scénario au couplage code le plus faible de toute la campagne —
> aucune sous-classe, aucun override de `control()`/`update()`, une force dans la boucle. C'est un
> avantage direct de l'architecture : si S5 échoue, ce ne sera imputable ni à une couche nouvelle
> ni à une dette de copie, mais aux mécanismes S2 eux-mêmes sous stress — exactement la question
> posée. La seule dette est la duplication du bloc de setup gait de `main()` (globals B/W), à
> re-synchroniser si la config gelée S2 évolue — signalé en tête de fichier.

---

*Ce document est la spécification de conception S5. Il ne modifie aucun résultat validé ni aucun
fichier S1–S4. Toute implémentation doit s'y conformer ou signaler l'écart.*
