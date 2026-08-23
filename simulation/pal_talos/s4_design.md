# S4 — Loco-manipulation : Design Document (marche + port bi-manuel d'une charge)

*Kickoff : 2026-07-10 | Statut : conception + première implémentation | Portée : port statique bi-bras pendant marche plane*
*Satellite de the thesis (S4), §9.2 (métriques), §13 (flow), §10 (définitions), §5 (hypothèse H1)*
*Pipeline cible : `pal_talos/` (MuJoCo natif + ProxQP), sous-classe de `talos_dcm_walk_timing.py` (S2)*

> **But du document.** Spécifier — avant de figer l'implémentation — ce que le scénario S4
> (marche plane avec port/saisie d'un objet, métriques *end-effector error* + *CoM tracking*,
> the thesis) exige au-delà du marcheur plan S2, statuer sur la variante de manipulation
> et sur la manière d'injecter la tâche de manipulation dans le QP-WBC, et fixer le protocole de
> campagne. S4 est le **test décisif de l'hypothèse H1** (« un QP-WBC hiérarchique permet une
> coordination *stable et simultanée* locomotion + manipulation ») — hypothèse que the thesis §6 marque encore *ouverte/défavorable* faute de scénario loco-manip exécuté.

---

## 1. Objectif et positionnement

S4 est le seul scénario de la campagne qui exerce **directement la question de recherche** telle
qu'elle est verrouillée par la question de recherche : la *simultanéité* locomotion + manipulation. S1 (équilibre),
S2 (marche plane) et S3 (franchissement) valident la couche locomotion du framework ; aucun ne
sollicite d'effecteur. S4 ajoute une **tâche d'organe terminal** aux tâches de la couche d'exécution
(CoM, orientation de base, posture, pied de balancement) et mesure si le QP peut arbitrer
l'ensemble sous les mêmes contraintes de dynamique corps-rigide et de cône de friction, sans
sacrifier la stabilité de marche.

Conformément à the thesis, la métrique principale est le couple *(end-effector error,
CoM tracking)*, avec report des métriques transverses (QP feasibility, temps de solve, task
success) déjà instrumentées pour S1/S2/S3. Le positionnement dans la chaîne logique (§13) est le
suivant : S2 fournit un marcheur dont la fragilité est *déjà caractérisée* (70 %, non-déterminisme
par seed, mode d'échec = divergence DCM latérale) ; S4 **greffe** la manipulation sur ce substrat
et pose une question incrémentale et falsifiable — la charge et la tâche EE **dégradent-elles** le
taux de succès et le CoM tracking de S2, et de combien ?

> **Insight critique.** La valeur de S4 pour la thèse ne tient pas à « faire porter un objet à
> TALOS » — de nombreux travaux loco-manip l'ont fait (Sentis 2010 ; Dietrich 2012 ; Murooka 2021).
> Elle tient à ce que S4 est le **seul point de mesure de H1** : sans lui, H1 reste une conjecture
> non testée et la contribution se réduit à un protocole appliqué à la locomotion seule. Greffer la
> manipulation sur le marcheur S2 *déjà borné en CI* transforme H1 en différence mesurable (Δ taux
> de succès, Δ CoM RMSE, EE RMSE) plutôt qu'en affirmation qualitative — cohérent avec la ligne
> éditoriale de reporting honnête adoptée pour les résultats négatifs S2 (Ch6).

---

## 2. Le verrou technique : coupler une tâche d'effecteur à un marcheur à marge mince

Le marcheur S2 (`DCMWalkT`) construit à chaque tick un QP sur `x = [qdd, tau, f]` dont le coût
agrège quatre tâches pondérées `w·‖J·qdd − a_des‖²` (CoM, orientation de base, posture, pied de
balancement en appui simple), sous égalité de dynamique `M·qdd − Sᵀ·tau − Jcᵀ·f = −h` et
inégalités de cône de friction + limites de couple (`talos_dcm_walk.py::control`). Ajouter la
manipulation soulève trois difficultés distinctes.

1. **Où injecter la tâche EE ?** La formulation QP-WBC absorbe naturellement une tâche
   supplémentaire : il suffit d'ajouter au coût un terme `W_EE·‖J_EE·qdd − a_EE‖²`, où `J_EE` est
   le jacobien translationnel de la main et `a_EE` une accélération désirée type PD. Le solveur
   arbitre alors *dans le même programme* manipulation et locomotion, sous les mêmes contraintes —
   c'est précisément l'avantage revendiqué du QP-WBC sur le task-switching.
   La difficulté n'est pas théorique mais **d'autorité relative** : un poids `W_EE` trop élevé vole
   au CoM et au pied de balancement les degrés de liberté et les couples nécessaires à l'équilibre ;
   trop faible, la main dérive et la métrique EE explose. `W_EE` est donc le paramètre d'arbitrage
   central de S4.

2. **Que représente « l'objet » ?** Deux modélisations sont défendables et **non équivalentes** :
   - *Charge connue* : la masse de l'objet est ajoutée aux corps de main dans le modèle MuJoCo. Le
     WBC calcule `M` et `h = qfrc_bias` sur **ce même modèle chargé** → le terme de gravité de la
     charge entre dans `h` et le contrôleur la compense en feedforward. On teste alors la
     *coordination* (tenir la main pendant la marche), pas l'estimation de masse.
   - *Charge inconnue (perturbation)* : la masse n'est pas dans le modèle du WBC ; on applique une
     force verticale externe aux mains (`xfrc_applied`). Le poids devient une **perturbation non
     modélisée** que seules les boucles de rétroaction (CoM, EE, posture) rejettent. On teste alors
     la *robustesse* à une charge non identifiée.
   Ces deux cas correspondent à deux régimes physiques réels (objet pesé vs objet surprise) et
   fournissent une **ablation** naturelle, pertinente pour H3 (contact/task-priority > contrôle
   naïf).

3. **La tâche EE doit-elle « suivre le corps » ?** Porter un objet signifie tenir la main **fixe
   relativement au tronc** : quand la base translate et tourne pendant la marche, la référence de
   main doit se translater/tourner avec elle, sinon la tâche EE combat le mouvement de marche. La
   tâche doit donc être formulée en **mouvement relatif main↔base** (position ET vitesse), et non
   comme une cible fixe en repère monde. Une cible monde fixe reviendrait à demander au robot de
   laisser sa main immobile pendant qu'il avance — l'opposé d'un port.

| Variante de manipulation | Ce qu'elle teste | Coût / risque | Décision S4 |
|--------------------------|------------------|---------------|-------------|
| **Port statique bi-bras** (mains fixes/tronc, charge portée) | H1 pur : simultanéité loco + tenue d'effecteur | Faible : pas de contact de préhension, pas de séquencement | **RETENUE (1er jet)** |
| Suivi de cible monde (main suit une trajectoire indépendante) | Découplage main/base explicite | Moyen : réf mobile, tracking sous marche | Future work / ablation |
| Reach → grasp → carry | Cycle réaliste complet | Élevé : contact de préhension + machine à états ⇒ proche des écueils S3 | Future work |

> **Insight critique.** Le choix du port statique bi-bras n'est pas une facilité mais une
> **discipline expérimentale** : il isole la variable que H1 interroge (peut-on ajouter *une* tâche
> d'effecteur sans casser la marche ?) en neutralisant deux sources de risque orthogonales — le
> contact de préhension (qui rouvrirait le débat modèle-de-contact de Ch3/Ch6) et le séquencement
> événementiel (dont S3 a démontré l'incompatibilité avec le timing adaptatif du marcheur). Toute
> variante plus riche introduite maintenant confondrait l'échec éventuel de H1 avec un échec de
> préhension ou de séquenceur. La configuration bi-bras symétrique (choix David) réduit en outre la
> perturbation latérale nette de la charge, ce qui donne à H1 sa chance la plus juste avant les
> variantes asymétriques.

---

## 3. Approche retenue pour le premier jet : tâche EE bi-manuelle relative-base dans le QP

**Décision : port statique à deux mains, charge modélisée comme masse ajoutée aux corps de main
(cas « connu ») avec ablation force externe (cas « inconnu »), tâche EE injectée comme coût
supplémentaire dans le QP du marcheur S2.** Justification : cohérence avec le risque the thesis framework trop complexe → design modulaire » (§8) et avec la ligne S1→S2→S3 (sous-classe, noyau
intact).

Mécanique (surcharge dans `DCMWalkS4(DCMWalkT)` ; `talos_wbc.py`, `talos_dcm_walk.py`,
`talos_dcm_walk_timing.py` **inchangés**) :

1. **Corps d'effecteur.** `arm_left_7_link` et `arm_right_7_link` (dernier segment de bras, parent
   du gripper) servent de trames de main. Vérifiés présents dans `talos.xml` ; TALOS = 2 × 7 DOF de
   bras (14 articulations actionnées de bras), déjà dans le vecteur de commande du QP.
2. **Référence relative-base figée à l'init.** Pour chaque main, on enregistre la pose locale
   `p_local = R_baseᵀ (p_main − p_base)` à l'instant initial (posture *home* fléchie). À chaque
   tick, la référence monde est reconstruite `p_ref = p_base + R_base · p_local` : la main est
   ainsi commandée à rester **rigidement solidaire du tronc**, donc l'objet porté reste stable par
   rapport au corps pendant que le robot marche.
3. **Tâche EE en mouvement relatif.** On forme le jacobien de la main `J_main` (translationnel, via
   `mj_jac`) et le jacobien du **point de référence rigidement attaché à la base** `J_ref`
   (`mj_jac` du point `p_ref` sur `base_link`). La tâche pilote le mouvement *relatif* :
   `J_rel = J_main − J_ref`, `a_EE = KP_EE·(p_ref − p_main) − KD_EE·(J_rel · qvel)`, ajoutée au coût
   par `add(J_rel, a_EE, W_EE)`. Formuler la tâche sur `J_rel` garantit qu'elle n'exige **aucun**
   couple pour le mouvement de corps rigide commun main+base (la marche « emmène » la main
   gratuitement) et ne pénalise que l'écart main-vs-tronc — c'est ce qui distingue un *port* d'un
   *maintien monde*.
4. **Charge.** `--payload M` (kg) répartie 50/50 : cas connu = `m.body_mass[main] += M/2` avant
   construction du WBC (⇒ `M`, `h` du QP intègrent la charge, compensation feedforward) ; cas
   inconnu (`--payload-unknown`) = `d.xfrc_applied[main] = [0,0,−(M/2)g]` à chaque tick, modèle WBC
   non chargé (perturbation pure). Aucune nouvelle scène : la charge est une propriété d'inertie /
   une force, pas un corps saisi.
5. **Instrumentation EE.** À chaque tick, `e_EE[side] = ‖p_main − p_ref‖` (monde) est loggé par
   main ; report RMSE et pic, séparé régime *transfert initial* vs *marche établie* vs *équilibre
   final*, pour comparer aux seuils §9.2.

> **Insight critique.** L'injection de la tâche EE comme simple terme de coût additionnel est
> exactement ce que la thèse revendique comme force du QP-WBC face au task-priority strict
> (null-space) : pas de projection hiérarchique, pas de commutation, un seul programme sous
> contraintes dures. Mais cette élégance a un prix mesurable — le QP est un arbitrage à **somme
> pondérée**, donc *soft* : rien ne garantit *a priori* que l'équilibre l'emporte sur la
> manipulation quand les deux entrent en conflit (bras tendu près d'une limite de couple, ou charge
> tirant le CoM hors du polygone d'appui). La question « le QP sacrifie-t-il la marche pour tenir la
> main, ou l'inverse ? » est empirique et sera lue directement dans le couple (EE error, CoM RMSE)
> le long d'un essai : c'est le cœur du résultat S4, et il faut résister à la tentation de le
> pré-régler à un compromis flatteur — le balayage `W_EE` (§6) doit exposer la frontière, pas la
> cacher.

---

## 4. Paramètres de conception

Valeurs de départ (statut : *design commitment*, non ancrées littérature sauf mention ; à
raffiner au bring-up) :

| Paramètre | Valeur de départ | Justification / à vérifier |
|-----------|------------------|----------------------------|
| Charge `M` | **2.0 kg** (2 × 1.0 kg) | Ordre de grandeur d'un objet manipulable ; ~2.5 % de la masse TALOS (~95 kg). À balayer {0, 1, 2, 4, 6} pour cartographier la dégradation. |
| Poids tâche EE `W_EE` | **50** | Entre posture (8) et CoM (200) : la main doit être tenue fermement mais **jamais** au détriment de l'équilibre. À balayer {10, 50, 100, 200}. |
| Gains EE `KP_EE, KD_EE` | **400, 40** | Raideur de main comparable aux gains de pied (KP_SW=420) ; amortissement ~critique. |
| Corps d'effecteur | `arm_{left,right}_7_link` | Dernier segment de bras (parent gripper). Alternative : base gripper — écart ~12 cm, non critique pour un port. |
| Posture de bras de départ | *home* du keyframe | Bras le long du corps (posture par défaut). Une posture « objet devant » (coudes fléchis) serait plus réaliste mais change le bras de levier ; ablation possible. |

> **Insight critique.** Le paramètre décisif est le rapport `W_EE / W_COM`. Trop haut, le robot
> privilégie une main parfaite et laisse le CoM diverger (chute type S2 aggravée par la charge) ;
> trop bas, la main lâche l'objet (EE error hors seuil) mais la marche survit. Il **n'existe pas**
> de raison théorique de préférer un réglage : la valeur de S4 est de *tracer cette courbe
> d'arbitrage*, pas de trouver un point unique. La posture de bras est le second levier caché : des
> bras tendus le long du corps minimisent le bras de levier de la charge sur le CoM (cas favorable),
> des bras tendus en avant le maximisent (cas sévère) — à déclarer explicitement pour ne pas
> surestimer H1 avec la géométrie la plus clémente.

---

## 5. Métriques S4

### 5.1 Métriques principales (§9.4)

| Métrique | Définition opérationnelle | Instrumentation |
|----------|---------------------------|-----------------|
| **End-effector error** | `‖p_main − p_ref‖` par main (repère monde), la référence étant la pose rigide-tronc. RMSE et pic, séparés par régime. | Nouvelle : distance main↔référence par tick, loggée gauche+droite. |
| **CoM tracking** | RMSE trajectoire CoM (xy) vs référence DCM, **déjà** dans `log["com_err"]` de S2 — réutilisé tel quel pour comparaison directe chargé vs non chargé. | **Déjà disponible** (`DCMWalkT`). |

### 5.2 Métriques transverses (report, seuils the thesis)

QP feasibility rate (> 99 %), friction-cone compliance (100 % par construction), QP solve time
(< 1 ms mean / < 5 ms p99), task success rate (marche 3 m + tenue EE sous seuil + équilibre final,
> 90 %, Wilson 95 % CI, N ≥ 20). Seuil EE (§9.2, statut *extrapolé*) : **RMSE < 2 cm** en reach
statique ; **pic < 5 cm** en marche — à réemployer, en soulignant son statut non ancré.

> **Insight critique.** Le CoM tracking étant strictement la métrique de S2, S4 offre une
> **comparaison A/B propre** (même marcheur, même seed, charge on/off) : c'est l'argument le plus
> solide de tout le chapitre, car il isole l'effet de la manipulation *toutes choses égales par
> ailleurs*. Le seuil EE, en revanche, est le maillon faible : the thesis le donne
> *extrapolé* (workspace TALOS via Pinocchio), sans source expérimentale. Il faut donc le présenter
> comme *engagement de conception* et privilégier la **grandeur relative** (EE error chargé vs à
> vide, EE error marche vs statique) plutôt qu'un verdict binaire vs un seuil arbitraire — sous
> peine de rouvrir la critique reviewer « métriques arbitraires » (§18).

---

## 6. Protocole de campagne

Cohérent avec S1/S2/S3 (Wilson CI, N ≥ 20, seed reproductible) et avec le **caveat de
non-déterminisme** hérité de S2 (l'unité reproductible est le taux agrégé, pas le label par seed) :

1. **Bring-up déterministe** (seed unique, charge nominale 2 kg) : vérifier qu'une marche 3 m avec
   port bi-manuel est possible avant toute statistique. Livrable : `s4_run.npz` + vidéo, courbe
   EE error(t) et CoM err(t) superposées.
2. **A/B charge** : même seed, `--payload 0` vs `2` — quantifier Δ CoM RMSE et Δ taux de succès dus
   à la seule charge (isolation de l'effet manipulation).
3. **Balayage d'arbitrage** : `W_EE ∈ {10, 50, 100, 200}` × `M ∈ {0, 2, 4, 6}` — cartographier la
   frontière EE error ↔ CoM RMSE ↔ survie. Non statistique : sonde de faisabilité.
4. **Ablation connu/inconnu** : `--payload` (masse au modèle) vs `--payload-unknown` (force externe)
   — pertinence H3 (feedforward vs rejet pur).
5. **Campagne statistique** : au meilleur point, batch N ≥ 20 seeds (réutilise le patron
   `s2_batch.py`) → taux de succès + Wilson CI ; report EE (RMSE/pic/CI), CoM RMSE, QP feas, ctrl p99.

> **Insight critique.** Réutiliser `s2_batch.py` garantit la comparabilité méthodologique S1–S4
> (atout pour la contribution « protocole reproductible », PQ4). Mais le non-déterminisme par seed
> diagnostiqué en S2 (bifurcation discrète de l'ensemble actif du QP) **se propagera** à S4, et la
> tâche EE **ajoute des degrés de liberté à l'arbitrage** — donc potentiellement de nouvelles
> décisions marginales du solveur. Il faut anticiper une CI au moins aussi large qu'en S2 à N égal
> et, comme pour S2, rapporter le **taux agrégé** plutôt que le sort d'un seed. Le point A/B (même
> seed charge on/off) est le seul qui échappe partiellement à ce caveat et doit donc porter
> l'argument principal.

---

## 7. Points de contact code (résumé pour l'implémentation)

Sous-classe `DCMWalkS4(DCMWalkT)` — aucune modification de `talos_wbc.py`, `talos_dcm_walk.py`,
`talos_dcm_walk_timing.py` :

- `control()` **surchargé** : copie fidèle de `DCMWalk.control()` (même assemblage QP, mêmes
  contraintes) + insertion des deux tâches EE `add(J_rel, a_EE, W_EE)` avant la régularisation.
  La copie est assumée (pas de hook dans la base) et commentée comme telle — même discipline que le
  séquenceur QS de S3.
- injection payload à la construction (`m.body_mass` cas connu) ou par tick (`d.xfrc_applied` cas
  inconnu).
- références EE relatives-base figées à l'init ; recomputées par tick.
- nouveau log `ee_err_L`, `ee_err_R` + régime (transfert / marche / fin).
- `update()` **hérité tel quel** de `DCMWalkT` : la manipulation ne touche pas la couche de
  planification de marche (le plan ZMP/DCM reste celui de S2).
- `_grf_z` / `_foot_contact` / `_land_metrics_tick` : réutilisés sans changement.

> **Insight critique.** Hériter `update()` sans le toucher est le choix qui rend S4 défendable : la
> couche de locomotion reste *bit-pour-bit* celle de S2, donc tout écart de taux de succès est
> imputable à la seule tâche EE + charge, et non à une re-planification opportuniste. C'est la
> même logique de traçabilité qui a préservé S1 pendant le développement de S2/S3. La seule dette
> technique est la copie de `control()` : elle devra être re-synchronisée si le `control()` de base
> évolue — à signaler en tête du fichier.

---

## 8. Risques spécifiques S4

| Risque | Type | Mitigation |
|--------|------|------------|
| `W_EE` trop haut ⇒ CoM diverge (chute aggravée par la charge) | Arbitrage | Balayage §6.3 ; borne `W_EE` sous `W_COM` ; lire le couple (EE, CoM) au lieu d'un point unique |
| Charge déplace le CoM hors du polygone (bras en avant) | Mécanique | Posture bras le long du corps au 1er jet ; déclarer la géométrie ; ablation posture |
| Limites de couple de bras/épaule saturées sous charge | Technique | Report du taux de feasibility ; le QP dégrade proprement (repli grav+PD) déjà en place |
| Non-déterminisme QP amplifié par la tâche EE | Reproductibilité | Report du taux agrégé (pas du label seed) ; A/B même seed pour l'argument principal |
| Seuil EE non ancré (§9.2 extrapolé) | Intégrité | Déclarer *design commitment* ; privilégier grandeurs relatives chargé/à vide |
| Marche S2 déjà à 70 % ⇒ S4 ≤ 70 % | Narratif | Cadrer S4 comme *différentiel* vs S2, pas comme un absolu ; résultat honnête (cf. S2/S3) |

> **Insight critique.** Le risque dominant est, comme pour S3, **narratif** avant d'être technique :
> bâtir S4 sur un marcheur à 70 % expose mécaniquement à un troisième taux sous le seuil de 90 %.
> La parade n'est pas de sur-régler pour « passer », mais de recadrer la question S4 en **delta**
> mesuré par rapport à S2 (la charge coûte-t-elle X points de succès et Y mm de CoM RMSE ?), ce qui
> reste informatif *même si l'absolu est bas*. Un S4 à 55 % avec une dégradation CoM de +8 mm sous
> 2 kg est un résultat publiable ; un S4 « réglé à 92 % » sur la posture et la charge les plus
> clémentes, sans balayage, serait fragile en peer-review. Ce document engage la première voie.

---

## 9. Prochaines actions

1. **Implémenter `DCMWalkS4`** (`talos_s4_locomanip.py`) — livré avec ce document (bring-up non
   exécuté : sandbox tronque les .py ; lancer sur machine conda / Vertex AI).
2. **Bring-up déterministe** → `s4_run.npz` + vidéo ; vérifier feasibility + tenue EE.
3. **A/B charge** (payload 0 vs 2, même seed) → premier chiffre différentiel pour H1.
4. **Balayage `W_EE` × M** → frontière d'arbitrage.
5. Rédiger **Ch5 §5.7** (S4) une fois le bring-up obtenu, sur le patron §5.5/§5.6.
6. **Vérifier + insérer** les références loco-manip citées (Sentis2010, Dietrich2012, Murooka2021)
   dans `references.bib` uniquement après validation vol/pp/DOI (règle the thesis — jamais
   inventer).

**Commandes de bring-up :**
```
python talos_s4_locomanip.py --seed 0 --payload 2.0 --save s4_run.npz         # port connu 2 kg
python talos_s4_locomanip.py --seed 0 --payload 0                             # A/B à vide
python talos_s4_locomanip.py --seed 0 --payload 2.0 --payload-unknown         # charge perturbation
python talos_s4_locomanip.py --seed 0 --payload 2.0 --wee 100 --viewer        # inspection visuelle
```

---

*Ce document est la spécification de conception S4. Il ne modifie aucun résultat validé (S1) ni le
noyau de contrôle (QP-WBC, planificateur DCM, marcheur S2). Toute implémentation doit s'y conformer
ou signaler l'écart.*
