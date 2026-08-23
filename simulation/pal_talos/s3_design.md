# S3 — Stair Climbing : Design Document (montée 3 marches)

*Kickoff : 2026-07-09 | Statut : conception (aucun code S3 encore écrit) | Portée : ascension seule*
*Satellite de the thesis (S3), §9.2 (métriques), §13 (flow), §10 (définitions)*
*Pipeline cible : `pal_talos/` (MuJoCo natif + ProxQP), sous-classe de `talos_dcm_walk_timing.py`*

> **But du document.** Spécifier — avant toute ligne de code — ce que le scénario S3
> (montée de 3 marches, métriques *foot clearance* + *forces de contact*, the thesis)
> exige au-delà du marcheur plat S2, statuer sur l'approche de planification, et fixer
> le protocole de campagne. Ce document est la brique « problème → méthode » de S3 ;
> il précède l'implémentation et le futur §5.6 (Ch5).

---

## 1. Objectif et positionnement

S3 étend la validation du framework WBC hiérarchique (centroidal planning + QP-WBC) au
**franchissement d'obstacles verticaux discrets** : la montée de trois marches. Conformément
au the thesis, la métrique principale est le couple *(foot clearance, forces de
contact)*, avec report des métriques transverses (CoM tracking, task success, QP feasibility,
temps de solve) déjà instrumentées pour S1/S2.

Le positionnement dans la chaîne logique (§13) est direct : S1 valide l'équilibre statique,
S2 la locomotion plane ; S3 teste la **robustesse du même exécuteur QP-WBC face à une rupture
de l'hypothèse de terrain plat**. C'est la première fois dans la campagne que la géométrie du
sol cesse d'être un demi-plan z = fz constant. S3 n'introduit **aucune** nouvelle tâche de
manipulation (réservée à S4) ni perturbation stochastique (réservée à S5) : la seule variable
ajoutée est l'altitude du contact.

> **Insight critique.** La valeur de S3 pour la thèse n'est pas de « faire monter TALOS » — de
> nombreux travaux l'ont fait (Caron-Kheddar-Tempier 2019 sur HRP-4) — mais de mesurer, sous
> un protocole reproductible et borné en CI, **jusqu'où le pipeline S2 dégénère quand on lève
> l'hypothèse LIPM plane**. Un échec quantifié (ex. clearance insuffisante, pic de GRF hors
> borne) a autant de valeur probante qu'un succès, à condition d'être attribué à une cause
> identifiée. C'est la ligne éditoriale déjà adoptée pour le résultat négatif S2 (Ch6).

---

## 2. Le verrou technique : rupture de l'hypothèse LIPM à hauteur constante

Le planificateur actuel (`talos_dcm_walk.py`) repose sur le **Linear Inverted Pendulum Model**
à hauteur de CoM constante. Trois lignes en fixent l'hypothèse (vérifiées au niveau du code) :

```python
self.fz    = p0[self.left][2]          # hauteur des DEUX pieds = un seul scalaire (sol plat)
self.zc    = subtree_com[base][2]      # hauteur de CoM figée à l'init
self.omega = np.sqrt(9.81 / self.zc)   # pulsation LIPM constante, calculée UNE fois
```

et la trajectoire de swing pose systématiquement le pied à cette même altitude :

```python
swing_pos[2] = self.fz + STEP_H * np.sin(np.pi * s)   # cloche au-dessus d'un sol plat unique
goal = np.array([zmp[k+1][0], zmp[k+1][1], self.fz])  # cible d'appui à fz constant
```

Sur un escalier, cette hypothèse casse à trois niveaux :

1. **Altitude d'appui variable.** Chaque marche `k` a sa propre hauteur `fz_k = k · h_riser`.
   Un scalaire `self.fz` ne peut plus décrire ni la cible d'appui, ni la détection de poser,
   ni la garde au sol du pied de balancement.
2. **Hauteur de CoM non constante.** Monter fait croître `z_c` d'environ `h_riser` par marche.
   Or `omega = sqrt(g/z_c)` gouverne toute la récursion DCM. Le maintenir figé introduit une
   erreur de modèle systématique qui s'aggrave à chaque marche. C'est le cœur du problème :
   la dynamique du DCM plan (Englsberger 2015, forme 2D) n'est valable qu'à `z_c` constant.
3. **Composante verticale de la dynamique.** Le LIPM plan suppose une force verticale égale au
   poids (accélération verticale nulle du CoM). Une montée demande un travail vertical net
   (`m·g·h_riser` par marche) que le modèle 2D ne représente pas — il apparaît comme une
   perturbation non modélisée sur le canal sagittal.

Deux familles de réponses existent dans la littérature, qu'il faut distinguer :

| Approche | Référence (vérifiée) | Idée | Coût d'intégration dans `pal_talos/` |
|----------|----------------------|------|--------------------------------------|
| **3D-DCM (eCMP/VRP)** | Englsberger, Ott, Albu-Schäffer, *IEEE T-RO* 31(2):355-368, 2015 | Étend le DCM en 3D ; le *Virtual Repellent Point* encode direction **et** magnitude de la force totale → gère `z_c` variable | Élevé : réécriture de la récursion DCM (2D→3D), nouvelle référence verticale de CoM |
| **VHIP capturability** | Caron, Escande, Lanari, Mallein, *IEEE T-RO*, 2019 (*Capturability-based Pattern Generation for Walking with Variable Height* ; démo escalier HRP-4) | Générateur de motifs sur pendule à hauteur variable, capturabilité sur terrain accidenté ; résolu assez vite pour MPC temps-réel | Très élevé : nouveau générateur de motifs, schéma d'optimisation dédié |
| **LIPM par plateaux (quasi-statique)** | Extension pragmatique du pipeline existant (pas de nouvelle théorie) | Chaque marche = un plateau à `z_c` localement constant ; `omega` re-calculé par marche, transitions verticales absorbées en double-appui lent | Faible : surcharge de `self.fz` → `fz_k`, `omega` par palier, cloche de swing relative au max(départ, arrivée) |

> ⚠️ **Note d'intégrité des sources.** Le `Caron2019` **déjà présent** dans `references.bib`
> est le papier **ICRA** *Stair Climbing Stabilization of the HRP-4 … Whole-body Admittance
> Control* (stabilisation par admittance, PAS la génération de motifs). Le papier **T-RO 2019**
> *Capturability-based Pattern Generation with Variable Height* est **distinct** et devra
> recevoir un BibKey séparé (proposition : `Caron2019VHIP`) **avec vol/pp/DOI vérifiés avant
> insertion** — règle the thesis / §19 (jamais inventer). Englsberger2015 est également
> à ajouter (`Englsberger2015`, T-RO 31(2):355-368) après vérification DOI.

> **Insight critique.** Le choix d'approche est un arbitrage risque/temps, pas de pureté
> théorique. Le 3D-DCM et le VHIP sont les réponses « correctes » mais exigent de remplacer le
> noyau de planification qui a demandé des semaines à stabiliser pour S2 — et qui reste fragile
> (70 %). Les importer maintenant reviendrait à empiler deux sources d'instabilité non résolues.
> La stratégie défendable pour un **premier jet** est l'approche par plateaux : elle est
> honnête sur ses limites (elle *approxime* la dynamique verticale), elle réutilise un exécuteur
> QP validé, et elle transforme la question en une hypothèse testable — « jusqu'à quelle hauteur
> de marche l'approximation quasi-statique tient-elle ? » — plutôt qu'en un chantier ouvert.

---

## 3. Approche retenue pour le premier jet : LIPM par plateaux + double-appui de montée

**Décision (à valider) : approche par plateaux quasi-statiques**, avec bascule explicite vers
3D-DCM/VHIP documentée comme *future work* si l'approximation échoue au-delà d'un seuil de
hauteur. Justification : cohérence avec le risque the thesis Framework trop complexe,
temps insuffisant → design modulaire, réduire horizon/DOF » (§8.4), et avec la ligne S2 (itérer
sur un noyau maîtrisé plutôt que le remplacer).

Mécanique proposée (surcharge, `talos_dcm_walk.py` et `_timing` **inchangés** — sous-classe S3) :

1. **Plan de pas 3D.** Le plan `zmp[k]` (x,y) est augmenté d'une altitude `fz_k` par appui :
   pieds au sol pour les premiers appuis, puis `fz_k = n_k · h_riser` où `n_k` est le numéro de
   marche visé par l'appui `k`. Cadence de montée : un pied par marche (montée « step-over-step »
   plutôt que « step-together », plus proche de S2 et évitant le double appui prolongé sur une
   même marche).
2. **`omega` par palier.** Recalcul de `omega_k = sqrt(g / z_c,k)` avec `z_c,k` la hauteur de CoM
   nominale sur la marche `k` (≈ `z_c,0 + n_k · h_riser`). La montée du CoM est traitée comme une
   rampe de référence pendant le double appui, pas comme une dynamique DCM active.
3. **Cloche de swing sur-élevée.** La garde au sol devient relative au **point le plus haut**
   entre décollage et pose : `z_swing(s) = max(fz_takeoff, fz_land) + STEP_H · sin(π s)`, avec
   `STEP_H` augmenté (voir §4) pour dégager le nez de marche. C'est la modification qui adresse
   directement la métrique *foot clearance*.
4. **Détection de poser par marche.** `_foot_contact` teste déjà le contact avec le worldbody
   (body 0) ; si les marches sont des `geom` enfants du worldbody, la détection reste valide
   **sans modification**. La garde de hauteur `foot_z <= fz + FOOT_TOL` doit en revanche viser
   `fz_land,k` et non le scalaire global.
5. **Transfert vertical en double-appui.** Entre deux marches, allonger la fenêtre de double
   appui (`DS_OVL`) pour laisser la référence de CoM monter quasi-statiquement avant d'engager
   le swing suivant — c'est là que le travail vertical `m·g·h` est fourni, hors de la récursion
   DCM.

> **Insight critique.** L'approximation par plateaux déplace le problème plutôt qu'elle ne le
> résout : elle suppose que la transition verticale peut être rendue « lente devant la dynamique
> du DCM » via le double appui. Cette hypothèse est **fausse au-delà d'une certaine cadence** —
> si le robot doit monter vite, le CoM accélère verticalement et le canal sagittal voit une
> perturbation que le contrôleur plan ne compense pas. Le protocole (§6) doit donc **balayer la
> hauteur de marche et la cadence** pour cartographier la frontière de validité, ce qui fait de
> cette limite un résultat mesurable et non un angle mort.

---

## 4. Géométrie de la scène MuJoCo (`scene_stairs.xml`)

La scène plate actuelle (`scene_motor.xml`) n'a qu'un `geom` plan. S3 requiert une nouvelle
scène incluant `talos_motor.xml` et ajoutant 3 marches comme `geom` de type `box` enfants du
worldbody (donc `geom_bodyid == 0`, ce qui préserve la logique de détection de contact existante).

Paramètres de conception proposés (à figer après vérification cinématique sur le modèle TALOS
chargé — **valeurs de départ, statut : *design commitment*, non ancrées littérature**) :

| Paramètre | Valeur de départ | Justification / à vérifier |
|-----------|------------------|----------------------------|
| Hauteur de contremarche `h_riser` | **0.10 m** | Conservateur vs marches humaines (~0.17 m) ; ordre de grandeur des démos robot escalier. À balayer {0.05, 0.10, 0.15}. |
| Profondeur de giron `d_tread` | **0.30 m** | ≥ longueur de semelle TALOS (~0.20 m) + marge d'appui. Vérifier la semelle réelle dans `talos.xml`. |
| Largeur de marche | **1.0 m** | Large : élimine tout couplage latéral parasite pour ce premier jet. |
| Nombre de marches | **3** | Fixé par the thesis |
| Garde au sol cible `STEP_H` | **≥ 0.12 m** | Doit dépasser `h_riser` + marge de sécurité (≥ 2 cm) pour ne pas taper le nez de marche ; base plate = 0.04 m, très insuffisant. |

Structure XML pressentie (esquisse, à implémenter au jet suivant) : palier de départ plan,
puis 3 `box` empilés en escalier (chaque marche = un bloc dont la face supérieure est à
`n · h_riser`), palier d'arrivée plan pour la phase d'équilibre finale (réutilise la logique
`ended` de `_timing`).

> **Insight critique.** Le paramètre décisif est le rapport `STEP_H / h_riser`. Une garde trop
> basse fait taper le pied dans la contremarche (échec de clearance) ; une garde trop haute
> allonge le vol, retarde le poser et — d'après la mécanique S2 — laisse le DCM latéral diverger
> davantage avant le contact, ce qui a été la cause dominante des chutes S2. S3 hérite donc
> **directement** de la fragilité de timing de S2 : monter demande des pas plus hauts, or des
> pas plus hauts aggravent précisément le mode d'échec déjà diagnostiqué. Cette interaction doit
> être surveillée dès le premier run.

---

## 5. Métriques S3

### 5.1 Métriques principales (§9.4)

| Métrique | Définition opérationnelle | Instrumentation |
|----------|---------------------------|-----------------|
| **Foot clearance** | Hauteur minimale du point le plus bas de la semelle de balancement **au-dessus du nez de la marche franchie**, sur toute la phase de vol. Positive = franchit ; négative = collision. | Nouvelle : distance semelle↔arête de marche par tick pendant le swing (via `d.xpos` du pied + géométrie connue des marches). |
| **Forces de contact** | Pic et moyenne de la GRF normale au poser sur chaque marche ; distribution make/break (rebonds). | **Déjà disponible** : `_grf_z(fb)` + fenêtre `_land_metrics_tick` (réutilisées telles quelles de `_timing`). |

### 5.2 Métriques transverses (report, seuils the thesis)

CoM tracking RMSE (< 3 cm steady-state), QP feasibility rate (> 99 %), friction-cone compliance
(100 % par construction), QP solve time (< 1 ms mean / < 5 ms p99), task success rate
(montée complète des 3 marches + équilibre final, > 90 %, Wilson 95 % CI, N ≥ 20).

> **Insight critique.** Le *foot clearance* est la seule métrique **nouvelle** de la campagne et
> la seule sans seuil ancré dans la littérature : the thesis ne le référence pas. Il
> faudra le déclarer explicitement *design commitment* (comme les autres seuils non ancrés) et
> proposer un seuil justifié — a minima « clearance > 0 sur 100 % des franchissements réussis »,
> idéalement une marge positive (ex. ≥ 2 cm) reliant clearance et robustesse. Ne pas laisser ce
> seuil implicite, sous peine de reproduire la critique reviewer « métriques arbitraires » (§18).

---

## 6. Protocole de campagne

Cohérent avec la méthodologie S1/S2 (Wilson CI, N ≥ 20, seed reproductible) et avec le **caveat
de non-déterminisme** documenté pour S2 (l'unité reproductible est le **taux agrégé**, pas le
label par seed) :

1. **Bring-up déterministe** (seed unique, sans bruit) : valider qu'une montée nominale des 3
   marches est possible avant toute statistique. Livrable : une trajectoire `s3_run.npz` + vidéo.
2. **Balayage de conception** : `h_riser ∈ {0.05, 0.10, 0.15}` × cadence (via `T_STEP`) pour
   localiser la frontière de validité de l'approximation par plateaux (§3). Non statistique —
   sonde de faisabilité.
3. **Campagne statistique** : au meilleur point de conception, batch N ≥ 20 seeds → taux de
   succès + Wilson CI, sur le modèle exact de `s2_batch.py`. Report : clearance (min/mean/CI),
   GRF (pic/mean), CoM RMSE par marche, ctrl p99.
4. **Ablations** : garde au sol `STEP_H` haute vs basse ; double-appui de montée long vs court.

> **Insight critique.** Reprendre `s2_batch.py` garantit la comparabilité méthodologique entre
> S1, S2 et S3 — atout pour l'argument « protocole d'évaluation reproductible » (contribution
> PQ4). Mais le non-déterminisme par seed diagnostiqué en S2 (bifurcation discrète de l'ensemble
> actif du QP) **se propagera à S3** et sera probablement **amplifié** : les contacts au ras des
> arêtes de marche multiplient les décisions marginales d'inclusion de contact. Il faut donc
> anticiper que le taux agrégé S3 aura une CI plus large à N égal, et dimensionner N en
> conséquence (peut-être N ≥ 35 comme il a fallu le faire pour trancher S2).

---

## 7. Points de contact code (résumé pour l'implémentation)

Sous-classe `DCMWalkS3(DCMWalkT)` — aucune modification de `talos_dcm_walk.py`, `talos_wbc.py`,
ni `talos_dcm_walk_timing.py` (même discipline que le passage S1→S2) :

- `self.fz` scalaire → tableau `fz_k` indexé par appui (ou fonction `foot_height(k)`).
- `omega` unique → `omega_k` recalculé au changement de marche.
- `z_swing` : cloche relative à `max(fz_takeoff, fz_land)` + `STEP_H` majoré.
- garde de poser `foot_z <= fz + FOOT_TOL` → viser `fz_land,k`.
- nouvelle métrique clearance (par tick de swing) ajoutée au `log`.
- nouvelle scène `scene_stairs.xml` (3 `box` enfants du worldbody).
- `_grf_z` / `_land_metrics_tick` / `_foot_contact` : **réutilisés sans changement**.

> **Insight critique.** La discipline « sous-classe uniquement, noyau intact » a permis de garder
> S1 valide pendant tout le développement S2. La tenir pour S3 est ce qui rend la campagne
> défendable : si S3 échoue, l'échec est localisé dans la couche escalier, pas dans un exécuteur
> QP qu'il faudrait re-valider. Toute tentation de « corriger vite » `talos_dcm_walk.py` pour
> faire passer S3 briserait cette traçabilité et invaliderait rétroactivement S1/S2.

---

## 8. Risques spécifiques S3

| Risque | Type | Mitigation |
|--------|------|------------|
| Approximation par plateaux invalide au-delà de `h_riser` seuil | Modèle | Balayage §6.2 : transformer la limite en résultat mesuré ; bascule 3D-DCM = future work |
| Garde au sol ↔ divergence DCM (pas hauts aggravent le mode d'échec S2) | Technique | Ablation `STEP_H` ; surveiller clearance vs t_chute dès le bring-up |
| Non-déterminisme QP amplifié par contacts au ras des arêtes | Reproductibilité | Report du taux agrégé (pas du label seed) ; N ≥ 35 si CI trop large |
| Semelle plus longue que le giron → sur-débord | Géométrie | Vérifier semelle TALOS réelle vs `d_tread` avant de figer la scène |
| Sources Englsberger2015 / Caron2019VHIP non vérifiées | Intégrité | Vérifier vol/pp/DOI avant insertion `references.bib` (règle §17-5) |

> **Insight critique.** Le risque dominant n'est pas technique mais **narratif** : S2 étant à
> 70 % (FAIL vs seuil 90 %), lancer S3 sur le même noyau fragile expose à une seconde métrique
> sous le seuil. La parade n'est pas de « faire mieux à tout prix » mais de cadrer S3, comme S2,
> en résultat honnête et diagnostiqué — la contribution de la thèse étant le *protocole* et les
> *diagnostics*, pas un taux de succès. Ce document pose ce cadre avant le premier run.

---

## 9. Prochaines actions

1. **Vérifier la cinématique TALOS** (longueur de semelle, hauteur de hanche) sur le modèle
   chargé pour figer `d_tread` et `h_riser` de départ.
2. **Écrire `scene_stairs.xml`** (3 `box`, palier départ/arrivée).
3. **Écrire `DCMWalkS3`** (sous-classe : `fz_k`, `omega_k`, cloche sur-élevée, métrique clearance).
4. **Bring-up déterministe** → `s3_run.npz` + vidéo.
5. **Vérifier + insérer** `Englsberger2015` et `Caron2019VHIP` dans `references.bib` (DOI validés).
6. Rédiger **Ch5 §5.6** une fois le bring-up obtenu.

---

*Ce document est la spécification de conception S3. Il ne modifie aucun résultat validé
(S1) ni le noyau de contrôle. Toute implémentation doit s'y conformer ou signaler l'écart.*
