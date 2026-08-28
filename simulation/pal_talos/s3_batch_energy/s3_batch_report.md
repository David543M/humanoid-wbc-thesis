# S3 batch — 35 essais (seeds 0..34)

Config gelee : `--rate 0.18 --tswing 0.65 --ttrmax 3.0 --postol 0.035 --clear 0.14 --hriser 0.10`

Verdict pre-enregistre : UPRIGHT et 3/3 marches et etat final DONE.

| Metrique | Valeur |
|---|---|
| Success rate | **11/35 = 31.4 %** |
| Wilson 95 % CI | [0.186, 0.480] |
| Critere Table 3.1 (borne basse > 0.90) | **FAIL** |
| Avancee (succes seuls, m) | 1.08 +/- 0.04 |
| Gain CoM z (succes seuls, m) | 0.26 +/- 0.02 |
| Pic GRF (tous, N) | 8480.00 +/- 3094.80 |
| Ctrl loop mean (tous, ms) | 4.00 +/- 0.75 |
| Ctrl loop p99 (tous, ms) | 12.12 +/- 3.60 |
| QP feasible (tous, %) | 100.00 +/- 0.00 |

Marches atteintes : 1/3 -> 2 essai(s), 2/3 -> 14 essai(s), 3/3 -> 18 essai(s)

Echecs : seed 0 (ok, 2/3, etat=SWING, t=17.9 s), seed 3 (ok, 2/3, etat=TRANSFER, t=19.6 s), seed 6 (ok, 2/3, etat=TRANSFER, t=20.3 s), seed 7 (ok, 3/3, etat=TRANSFER, t=22.5 s), seed 9 (ok, 2/3, etat=TRANSFER, t=19.0 s), seed 10 (ok, 2/3, etat=TRANSFER, t=20.1 s), seed 11 (ok, 2/3, etat=TRANSFER, t=20.0 s), seed 12 (ok, 3/3, etat=DONE, t=25.0 s), seed 15 (ok, 3/3, etat=SWING, t=24.1 s), seed 17 (ok, 3/3, etat=TRANSFER, t=22.4 s), seed 18 (ok, 2/3, etat=TRANSFER, t=20.1 s), seed 19 (ok, 2/3, etat=TRANSFER, t=21.0 s), seed 20 (ok, 2/3, etat=TRANSFER, t=20.5 s), seed 21 (ok, 2/3, etat=TRANSFER, t=18.5 s), seed 23 (ok, 1/3, etat=TRANSFER, t=15.2 s), seed 24 (ok, 3/3, etat=DONE, t=24.8 s), seed 25 (ok, 3/3, etat=DONE, t=25.3 s), seed 26 (ok, 2/3, etat=TRANSFER, t=17.4 s), seed 27 (ok, 3/3, etat=TRANSFER, t=22.7 s), seed 28 (ok, 2/3, etat=TRANSFER, t=18.4 s), seed 29 (ok, 2/3, etat=TRANSFER, t=20.6 s), seed 31 (ok, 2/3, etat=TRANSFER, t=17.4 s), seed 32 (timeout, 0/3, etat=?, t=nan s), seed 33 (ok, 1/3, etat=TRANSFER, t=14.8 s)

> ATTENTION metrique clearance : le logger `_clear_tick` du sequenceur
> remonte 0 franchissement sur un run complet (item P4 du backlog polish,
> non corrige — toucher le swing avait regresse la montee 3 fois). La garde
> de swing n'est donc PAS chiffrable depuis ce batch ; ne pas la rapporter.
