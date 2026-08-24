# Câblage du museau — pas à pas (bumper 4 switchs + sonar HC-SR04)

## Repères, avant tout
- **Repère n° 1 : le fil 5 V du lidar déjà branché.** Il est sur l'une des deux broches 5 V voisines (2 et 4). L'extrémité de
  l'en-tête où il se trouve = l'extrémité « 1-2-3-4 ». Sa rangée = la **rangée PAIRE** (2, 4, 6 … 40). L'autre rangée = **IMPAIRE**.
- **Gauche/droite du rover** = comme si tu étais assis dessus, regardant vers le bumper neuf. (Quand tu fais face au museau,
  c'est inversé.)
- **Pattes du V-156** : marquées sur le flanc `COM` / `NO` / `NC`. Au doute, multimètre en continuité : `COM–NC` passe au repos,
  `COM–NO` passe quand on presse le levier. **On utilise COM et NO** (NC reste vide).

## Barrette A — 5 fils, rangée IMPAIRE, positions 15 à 19 en partant de l'extrémité du lidar (= broches 29-31-33-35-37)
En partant du côté extrémité-lidar de la barrette :
1. **A1 → broche 29** : patte **NO** du switch le plus à **GAUCHE**
2. **A2 → broche 31** : patte **NO** du switch **centre-gauche**
3. **A3 → broche 33** : patte **NO** du switch **centre-droit**
4. **A4 → broche 35** : patte **NO** du switch le plus à **DROITE**
5. **A5 → broche 37** : **ECHO** du sonar — direct si le sonar est alimenté en 3,3 V (essai validé), sinon via le pont
   diviseur : `Echo —[R]—●—[2R]— GND`, le point ● va en A5.

## Barrette B — 3 fils groupés, rangée PAIRE, positions 16-17-18 (= broches 32-34-36)
1. **B1 → broche 32** : le **bus COM** — un fil qui part en 32, va à la patte **COM** du premier switch, puis se **chaîne de COM
   en COM** sur les quatre (4 petits ponts sertis ou soudés). C'est le 3,3 V « logiciel » qui alimente les switchs.
2. **B2 → broche 34** : la **masse commune** — un fil chaîné : GND du sonar, puis bas du pont diviseur (s'il existe). Les
   switchs, eux, n'ont **pas** de masse.
3. **B3 → broche 36** : **TRIG** du sonar.

## Barrette C — 1 à 2 fils, extrémité du lidar
1. **C1 → la broche 5 V restée libre (2 ou 4)** : **VCC** du sonar (ou broche 1 si l'essai 3,3 V a été concluant).
2. (option) **C2 → broche 6** : masse de secours, si tu préfères chaîner la masse ici plutôt qu'en 34.

## Vérifications avant d'allumer (multimètre, rover éteint)
1. Continuité **32 → chaque COM** (le bus est entier).
2. Pour chaque switch : continuité **NO → sa broche** (29/31/33/35), et `COM–NO` qui ne passe **que** levier pressé.
3. **Aucune** continuité entre 32 et 29/31/33/35 au repos (sinon un switch est câblé sur NC).
4. Continuité GND : broche 34 → GND sonar (→ bas du diviseur).
5. Rien entre 5 V et GND.

## Vérification logicielle (moi, rover allumé, moteurs coupés)
`BUMP #n` dans le log à chaque pression de chaque coin, la bonne position dans la carte, et la distance sonar qui suit ta main.
