# Conditionnement du QP du WBC — contact mou vs égalité dure

## Problème
La tâche de non-glissement des contacts était un **coût mou** à très fort poids
(`W_CONTACT = 1e4`). Ajoutée au Hessien comme `1e4·Jcᵀ Jc`, à côté des autres tâches
(poids 1–100), elle étale énormément le spectre de `G` → **mauvais conditionnement**
mesuré : `κ(G) ≈ 2.1×10¹⁰`. Le solveur (quadprog) s'en sortait (0 repli) mais le QP
était numériquement fragile.

Le choix initial du coût mou était motivé par le **rang déficient** : imposer en dur les
24 contraintes des coins de contact (4 coins × 3 ddl × 2 pieds) est redondant — un pied
plat rigide n'a que 6 ddl, donc 12 des 24 lignes sont dépendantes, et quadprog rejette
une égalité non de plein rang.

## Correction testée (opt-in : `wbc.hard_contact = True`)
On exprime le no-slip en **égalité dure sur la pose 6D du corps-pied** (3 translation +
3 rotation par pied via `mj_jac`), soit **12 lignes indépendantes** en double appui —
plein rang, pas de redondance. La tâche molle `1e4·Jcᵀ Jc` est retirée du coût ; les
forces de contact `f` restent (pyramide de friction inchangée).

## Résultat (debout, identique par ailleurs)

| Formulation | κ(G) médian | replis QP | debout | err CoM | push 150/400 N |
|---|---:|---:|---|---:|---|
| coût mou (défaut) | **2.1×10¹⁰** | 0 | UPRIGHT | 0.01 mm | OK / OK |
| **égalité dure** (`hard_contact`) | **1.0×10⁷** | 0 | UPRIGHT | 0.01 mm | OK / OK |

**Conditionnement amélioré ×2000** (2.1e10 → 1.0e7), sans aucune perte de performance ni
de robustesse, et toujours 0 repli (le rang déficient est évité par la formulation 6D).

## Limite / extension
`hard_contact` fige **tous** les pieds de `self.feet` : valable en **double appui**
(debout). Pour la **marche** en simple appui, il faudrait n'imposer la contrainte que sur
le(s) pied(s) réellement en contact (pied d'appui), pas le pied de balancement — le walker
`DCMWalk` ayant son propre `control()`, l'extension y est à faire séparément.

## Recommandation
Garder `hard_contact` comme l'option recommandée pour l'équilibre/poussée (meilleur
conditionnement, formulation plus propre). À documenter en Méthodologie comme la
formulation de référence, le coût mou étant l'historique. Activation :
`wbc = WBC(m, d); wbc.hard_contact = True`.

---

## Mise à jour — formulation dure adoptée par défaut (équilibre + marche)

`hard_contact` (équilibre) et `use_hard_contact` (walker, sur le pied d'appui seul) sont
désormais les **valeurs par défaut**. Résultats mesurés :

| Scénario | κ(G) avant (mou) | κ(G) après (dur) | performance |
|---|---:|---:|---|
| Équilibre debout | 2.1×10¹⁰ | **1.0×10⁷** | UPRIGHT, push 150/400 N OK, CoM 0.01 mm |
| Marche (`--walk-cl`) | 2.08×10¹⁰ | **2.07×10⁷** | **4 pas propres** (identique), 0 repli |

Gain de conditionnement ~×1000–2000 sans changement de performance. En simple appui,
seule la contrainte du **pied d'appui** est imposée (le pied de balancement reste libre),
via la liste `active` du walker.

**Note de reproductibilité** : changer la formulation de contact par défaut décale
légèrement la *trajectoire* du baseline open-loop *après la chute* (le robot tombé glisse
différemment ; ex. x≈1.5 m au lieu de 0.82 m à 8 s). Les métriques **significatives** sont
préservées : il tombe toujours (open-loop), 0 repli QP, équilibre/poussée inchangés, et
4 pas propres en walk-cl. Pour revenir à l'historique : `wbc.hard_contact=False` /
`c.use_hard_contact=False`.
