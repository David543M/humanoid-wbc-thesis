# S3 batch — 35 essais (seeds 0..34)

Config gelee : `--rate 0.18 --tswing 0.65 --ttrmax 3.0 --postol 0.035 --clear 0.14 --hriser 0.10`

Verdict pre-enregistre : UPRIGHT et 3/3 marches et etat final DONE.

| Metrique | Valeur |
|---|---|
| Success rate | **10/35 = 28.6 %** |
| Wilson 95 % CI | [0.163, 0.451] |
| Critere Table 3.1 (borne basse > 0.90) | **FAIL** |
| Avancee (succes seuls, m) | 1.09 +/- 0.02 |
| Gain CoM z (succes seuls, m) | 0.27 +/- 0.00 |
| Pic GRF (tous, N) | 8692.23 +/- 3092.99 |
| Ctrl loop mean (tous, ms) | 1.46 +/- 0.86 |
| Ctrl loop p99 (tous, ms) | 3.89 +/- 2.79 |
| QP feasible (tous, %) | 100.00 +/- 0.00 |

Marches atteintes : 1/3 -> 2 essai(s), 2/3 -> 14 essai(s), 3/3 -> 18 essai(s)

Echecs : seed 0 (ok, 2/3, etat=SWING, t=20.7 s), seed 1 (ok, 3/3, etat=DONE, t=23.6 s), seed 2 (ok, 2/3, etat=TRANSFER, t=18.0 s), seed 3 (ok, 2/3, etat=TRANSFER, t=19.7 s), seed 4 (ok, 2/3, etat=TRANSFER, t=19.9 s), seed 5 (ok, 3/3, etat=SWING, t=23.6 s), seed 6 (ok, 2/3, etat=TRANSFER, t=19.4 s), seed 7 (ok, 3/3, etat=TRANSFER, t=22.5 s), seed 9 (ok, 2/3, etat=TRANSFER, t=19.0 s), seed 10 (ok, 3/3, etat=TRANSFER, t=21.8 s), seed 12 (ok, 3/3, etat=DONE, t=25.0 s), seed 15 (ok, 3/3, etat=SWING, t=23.4 s), seed 18 (ok, 2/3, etat=TRANSFER, t=20.8 s), seed 19 (timeout, 0/3, etat=?, t=nan s), seed 20 (ok, 2/3, etat=TRANSFER, t=19.5 s), seed 21 (ok, 2/3, etat=TRANSFER, t=18.5 s), seed 23 (ok, 1/3, etat=TRANSFER, t=15.0 s), seed 24 (ok, 3/3, etat=TRANSFER, t=22.4 s), seed 26 (ok, 2/3, etat=TRANSFER, t=17.4 s), seed 27 (ok, 3/3, etat=TRANSFER, t=22.0 s), seed 28 (ok, 2/3, etat=TRANSFER, t=18.4 s), seed 29 (ok, 2/3, etat=TRANSFER, t=20.6 s), seed 31 (ok, 2/3, etat=TRANSFER, t=17.4 s), seed 32 (ok, 2/3, etat=SWING, t=21.8 s), seed 33 (ok, 1/3, etat=TRANSFER, t=14.6 s)

> ATTENTION metrique clearance : le logger `_clear_tick` du sequenceur
> remonte 0 franchissement sur un run complet (item P4 du backlog polish,
> non corrige — toucher le swing avait regresse la montee 3 fois). La garde
> de swing n'est donc PAS chiffrable depuis ce batch ; ne pas la rapporter.
