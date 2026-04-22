# Overleaf Template — Guide d'utilisation
**Fichier :** `humanoid_wbc_thesis_overleaf.zip` (racine du projet IRP)  
**Dernière mise à jour :** 2026-04-22  
**Compilé avec :** TeX Live 2022+ / pdflatex — **0 erreur, 0 warning, 41 pages**

---

## 1. Import dans Overleaf

1. Aller sur [overleaf.com](https://www.overleaf.com) → **New Project** → **Upload Project**
2. Sélectionner `humanoid_wbc_thesis_overleaf.zip`
3. Overleaf détecte `main.tex` comme racine automatiquement
4. Compiler avec **pdfLaTeX** (menu Compiler → pdfLaTeX)
5. Le fichier `latexmkrc` lance `makeglossaries` automatiquement pour la liste des acronymes

---

## 2. Structure du template

```
main.tex                        ← Racine : packages, macros, notation canonique
references.bib                  ← 9 sources pré-chargées (§19 master_memory)
latexmkrc                       ← Configuration Overleaf (makeglossaries auto)
figures/                        ← Déposer les figures .pdf / .png ici
frontmatter/
  titlepage.tex                 ← Page de titre (remplir \theinstitution, \thesupervisor)
  abstract.tex                  ← Résumé (~250 mots) + mots-clés
  acknowledgements.tex          ← Remerciements
chapters/
  ch1_introduction.tex          ← Motivation, RQ, H1/H2/H3, scope, contributions
  ch2_state_of_the_art.tex      ← WBC model-based, MPC, RL, gaps (researchgap boxes)
  ch3_methodology.tex           ← EOM, QP complet, centroidal MPC, contact modeling
  ch4_simulation_framework.tex  ← Simulateur, robot URDF, stack logiciel, boucle de contrôle
  ch5_experiments.tex           ← Scénarios S1–S5, métriques, baselines, résultats
  ch6_discussion.tex            ← Évaluation H1/H2/H3, limites, sim-to-real
  ch7_conclusion.tex            ← Contributions, travaux futurs
appendices/
  appendix_a_math.tex           ← Rappels mathématiques (dynamique corps rigides, QP, friction cone)
  appendix_b_code.tex           ← Listings de code (Pinocchio/OSQP, EKF)
```

---

## 3. Environnements personnalisés

Ces environnements sont définis dans `main.tex` et **obligatoires** (§17 master_memory rule 8) :

### `criticalinsight` — fin de chaque section
```latex
\begin{criticalinsight}
  Texte synthétisant les limites ou questions ouvertes de la section.
\end{criticalinsight}
```
> Boîte bleue — titre automatique "Critical Insight"

### `hypothesis{H1}` — hypothèses numérotées
```latex
\begin{hypothesis}{H1}
  Énoncé de l'hypothèse H1.
\end{hypothesis}
```
> Boîte verte — titre "Hypothesis H1"

### `researchgap` — lacunes de la littérature
```latex
\begin{researchgap}
  Description de la lacune identifiée.
\end{researchgap}
```
> Boîte rouge — titre "Research Gap"

---

## 4. Notation canonique (§10 master_memory)

Toutes les macros sont définies **une seule fois** dans `main.tex`. Ne jamais les redéfinir dans les chapitres.

| Macro | Rendu | Signification |
|-------|-------|---------------|
| `\q` | $\mathbf{q}$ | Coordonnées généralisées |
| `\dq` | $\dot{\mathbf{q}}$ | Vitesses généralisées |
| `\ddq` | $\ddot{\mathbf{q}}$ | Accélérations généralisées |
| `\torque` | $\boldsymbol{\tau}$ | Couple articulaire |
| `\M` | $\mathbf{M}(\mathbf{q})$ | Matrice d'inertie |
| `\Cg` | $\mathbf{C}(\mathbf{q},\dot{\mathbf{q}})\dot{\mathbf{q}} + \mathbf{g}(\mathbf{q})$ | Coriolis + gravité |
| `\J` | $\mathbf{J}$ | Jacobien |
| `\Jc` | $\mathbf{J}_c$ | Jacobien de contact |
| `\fc` | $\mathbf{f}_c$ | Torseur de contact (6D) |
| `\com` | $\mathbf{p}_{\mathrm{CoM}}$ | Position du centre de masse |
| `\lmom` | $\mathbf{l}$ | Quantité de mouvement linéaire |
| `\amom` | $\mathbf{k}$ | Moment cinétique |
| `\zqp` | $\mathbf{z}$ | Variable de décision QP |
| `\Hqp` | $\mathbf{H}$ | Hessien du QP |
| `\norm{x}` | $\lVert x \rVert$ | Norme |
| `\transpose` | ${}^{\top}$ | Transposée |
| `\WBC` | Whole-Body Control (WBC) | Acronyme — 1ère occurrence par chapitre |
| `\QP` | Quadratic Program (QP) | Acronyme — 1ère occurrence par chapitre |
| `\MPC` | Model Predictive Control (MPC) | Acronyme — 1ère occurrence par chapitre |

> **Règle :** Pour les contacts indexés (contact $i$), ne pas écrire `\fc_i` (double indice). Écrire `\bm{f}_{c,i}` directement.

---

## 5. Bibliographie

Le fichier `references.bib` contient les 9 sources du tableau de traçabilité (§19 master_memory) :

| Clé BibTeX | Source | Statut |
|------------|--------|--------|
| `Sentis2005` | Sentis & Khatib — IJRR 2005 | ⏳ À vérifier |
| `Wensing2013` | Wensing & Orin — ICRA 2013 | ⏳ À vérifier |
| `Koolen2016` | Koolen et al. — IJHR 2016 | ⏳ À vérifier |
| `Orin2013` | Orin et al. — Autonomous Robots 2013 | ⏳ À vérifier |
| `Caron2020` | Caron et al. — ICRA 2020 | ⏳ À vérifier |
| `Winkler2018` | Winkler et al. (TOWR) — RAL 2018 | ⏳ À vérifier |
| `Pinocchio2019` | Carpentier et al. — SII 2019 | ⏳ À vérifier |
| `Crocoddyl2020` | Mastalli et al. — ICRA 2020 | ⏳ À vérifier |
| `Kumar2021` | Kumar et al. (RMA) — RSS 2021 | ⏳ À vérifier |
| `MuJoCo2012` | Todorov et al. — IROS 2012 | ⏳ À vérifier |

Citer avec `\cite{Sentis2005}` ou `\citep{Sentis2005}` (natbib, style IEEE).  
Ajouter les nouvelles sources directement dans `references.bib` en bas du fichier (template vide fourni).

---

## 6. Acronymes / Glossaire

Les acronymes sont définis dans `main.tex` via le package `glossaries`.  
Utiliser `\gls{wbc}` (première occurrence développée automatiquement) ou `\acrshort{wbc}` / `\acrlong{wbc}` pour contrôle manuel.

Acronymes pré-définis : WBC, QP, MPC, CoM, DOF, EKF, LCP, URDF, RL, LIP, SEA, IMU.

Pour ajouter un acronyme :
```latex
% Dans main.tex, section "ACRONYMS / GLOSSARY"
\newacronym{aba}{ABA}{Articulated Body Algorithm}
```

---

## 7. Règles de cohérence (§17 master_memory)

Ces règles sont à respecter dans tous les fichiers de chapitres :

1. **Terme principal :** `Whole-Body Control (WBC)` — jamais sans tiret, jamais "wholebody control"
2. **Robot :** `humanoid robot` — pas "bipedal robot" sauf si contexte spécifique
3. **QP :** développer "Quadratic Program (QP)" à la première occurrence de chaque chapitre
4. **Éviter :** "clearly", "obviously", "simply"
5. **Toute affirmation factuelle** doit être suivie d'une citation `\cite{}`
6. **Notation** : définie une fois en §3.1, réutilisée partout sans redéfinition
7. **"Simulation"** (singulier) quand on désigne la méthode générale
8. **Fin de section** : toujours un bloc `criticalinsight`

---

## 8. TODOs prioritaires (lier aux PQs de master_memory)

| Fichier | TODO | Bloquant pour | PQ lié |
|---------|------|---------------|--------|
| `frontmatter/titlepage.tex` | Remplir `\theinstitution`, `\thesupervisor` | — | — |
| `frontmatter/abstract.tex` | Écrire l'abstract (~250 mots) | Soumission | — |
| `ch1_introduction.tex` | Finaliser la research question | Tout | PQ4 |
| `ch1_introduction.tex` | Délimiter la contribution vs Crocoddyl | Ch.1 | PQ4 |
| `ch3_methodology.tex` | Justifier le choix du modèle de contact | Ch.3 | PQ2 |
| `ch4_simulation_framework.tex` | Choisir simulateur + robot URDF | Ch.3, Ch.4 | PQ1, PQ2 |
| `ch5_experiments.tex` | Ancrer les seuils métriques (X, Y, Z) | Ch.5 | PQ6 |
| Toutes les figures | Remplacer `\fbox{...}` par les vraies figures | — | — |
| `references.bib` | Marquer les sources ✅ après lecture complète | Manuscrit final | — |

---

## 9. Workflow recommandé

```
Rédiger un chapitre (Overleaf)
    ↓
Vérifier la cohérence avec master_memory.md
    ↓
Committer sur GitHub (branch chapter/N-title)
    ↓
Mettre à jour master_memory.md §23 (Session Log)
    ↓
Relecture critique (adversarial mode)
    ↓
Merge vers main
```

**Convention de commit (§20 master_memory) :**
```
[chapter-1] Add introduction skeleton
[chapter-2] Draft WBC model-based section
[lit] Add Orin2013 to references.bib
[mem] Update master_memory session log
```

---

*Ce guide est synchronisé avec `master_memory.md` §12 (structure), §17 (règles d'écriture), §19 (sources), §20 (GitHub).*
