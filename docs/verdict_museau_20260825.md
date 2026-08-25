J'ai lu le dossier de faits et vérifié moi-même les deux PDF NVIDIA (je cite du verbatim, pas des résumés d'agents). Voilà le verdict.

---

# Verdict — museau VECTOR, 25/08/2026

## 1. La cause racine

Sur le Jetson Orin Nano DevKit, **toutes** les broches du connecteur 40 (sauf les 4 broches I²C) passent par une petite puce d'adaptation, un **TXB0108**, qui pousse en permanence la ligne à travers une résistance interne d'environ 4 kΩ — elle ne lâche jamais la ligne. Tes résistances de 3,3 k et 10 k tombent pile dans la zone où elles ne gagnent ni ne perdent contre cette puce : la ligne se retrouve à ~1,4–2,4 V, c'est-à-dire ni un vrai 0 ni un vrai 1, et elle **reste collée** sur le dernier état vu. C'est exactement ta signature : états qui persistent, mapping différent à chaque passe, appuis visibles sur deux lignes et jamais sur les deux autres.

**Certitude : très élevée.** Ce n'est pas une théorie d'agent, c'est écrit noir sur blanc dans deux documents NVIDIA que j'ai relus ligne à ligne :

- Spec de la carte (SP-11324-001, tableau 3-3, note 3), portée par tes 8 broches 29/31/32/33/35/36/37/38 : *« These pins connect to TI TXB0108 level translators. Due to the design of these devices, the output drivers are very weak »*.
- Note d'application NVIDIA (DA-09753) : *« The TXB level shifters have output buffers with ~4kΩ resistors in series which make them very weak »* ; pour forcer une ligne il faut *« more than ±2 mA »*, soit une résistance *« ~1.65kΩ or stronger »* ; et une résistance qui ne doit PAS gêner doit être *« > 50kΩ »*.
- Même spec, section 3.3 : *« Any pull-up or pull-down resistors on the signals (except I2C) must be weak (limited to >50 kΩ). »*

Traduction : il fallait soit **moins de 1,65 kΩ**, soit **plus de 50 kΩ**. Tu es à 3,3 k et 10 k. Le pire endroit possible, et c'est un choix que 100 % des gens font naturellement.

**Ce qui n'est PAS en cause, et je le dis franchement : ton câblage.** Les switchs sont bons, les soudures sont bonnes, la masse est bonne, tes mesures au multimètre étaient toutes justes. Deux jours contre une puce de la carte, pas contre ton travail. Le seul truc que ton multimètre ne pouvait pas voir, c'est que tes 3,3 V sur les COM étaient mesurés **à vide** : la broche 32 débite au mieux quelques dizaines de microampères (NVIDIA annonce ±20 µA sur ces broches, contre ±2 mA sur les broches I²C). Dès qu'un switch se ferme, ce « bus » s'effondre.

Deux corollaires qui closent les questions ouvertes du dossier :
- **Les pulls internes ne peuvent RIEN faire ici** (question A). Ils sont côté puce Jetson, en 1,8 V, *derrière* l'adaptateur. Le connecteur n'en voit jamais rien. Aucune quantité de devmem n'y changera quoi que ce soit — c'était un cul-de-sac, pas une erreur de lecture du TRM.
- **La broche 29 n'est probablement pas morte.** « Sortie OK, entrée qui ne lit rien » est exactement ce que produit le mécanisme ci-dessus. À ne pas condamner.

*(Deux détails mineurs, pour l'honnêteté : le « 1 sur 6 » était en fait 1 sur 4 — les broches 29 et 37 n'avaient pas de résistance, leur « échec » était normal. Et tes scripts d'écoute avaient de vrais bugs, notamment aucun anti-rebond dans l'UI et une référence prise sur un seul échantillon. Ils ont amplifié le chaos, ils ne l'ont pas créé.)*

## 2. Le Jetson est-il capable de lire 4 switchs proprement ?

**Oui — mais à des conditions strictes, et il ne le fera jamais avec le montage actuel.**

Il faut inverser complètement la logique : **switch vers la MASSE**, logique actif-bas, et une résistance de rappel vers le 3,3 V d'**au plus 1,5 kΩ**. C'est littéralement le schéma officiel de NVIDIA (note d'application, figure 6 : 3,3 V → 1,5 kΩ → bouton → broche). Ça marche parce que l'appui devient un court-circuit franc à la masse : 0 Ω bat n'importe quelle puce faible.

⚠️ **Piège à ne surtout pas suivre :** un des rapports recommande un rappel de 50 k à 100 kΩ. **C'est faux et ça t'aurait coûté un troisième jour.** Le « > 50 kΩ » de NVIDIA parle d'un cas différent ; à 100 k, ta ligne resterait bloquée en bas après le premier appui, pour toujours. Pour un bouton, c'est **1,0 à 1,5 kΩ, pas plus**.

Deuxième condition, indépendante : ton JetPack 6.2 (L4T r36.4.3) a un **bug GPIO reconnu par NVIDIA** — le pilote remet la broche en « fonction spéciale » toute seule, et NVIDIA dit que c'est corrigé seulement à partir de JetPack 6.2.2. Donc même bien câblé, le 40-pin reste une plateforme sur laquelle il faut se battre.

C'est pour ça que je ne te recommande pas cette voie-là.

## 3. La voie recommandée : un ESP32 en USB, et on quitte le problème

Une seule route, et c'est celle qui **sort complètement** du terrain où tu t'es fait avoir. Tu connais déjà ce chemin par cœur : c'est exactement l'architecture Vita (ESP32 → série → service Python) et celle du RoArm.

**Ce que tu fais, toi (environ 30 min de fer à souder) :**

1. Tu débranches tout le faisceau du connecteur 40 broches du Jetson. Tu ne touches ni aux switchs, ni à leurs fils.
2. **Tu retires complètement le peigne de 4 résistances.** C'est important : l'ESP a ses propres rappels internes, et les 3,3 k / 10 k les écraseraient. Zéro résistance sur les switchs.
3. La chaîne des COM (celle qui allait au « bus » 3,3 V) part maintenant sur **GND de l'ESP32**. Un seul fil déplacé.
4. Les 4 fils NO vont sur 4 broches de l'ESP32 (je te donnerai les numéros exacts avec le firmware).
5. Le sonar passe sur l'ESP aussi : TRIG direct, et **ECHO à travers un diviseur 1 kΩ / 2 kΩ** (2 résistances). C'est obligatoire, et c'est le seul vrai danger matériel du montage actuel : l'ECHO sort du 5 V sur une broche prévue pour 3,3 V, hors spec des deux côtés.
6. Un câble USB de l'ESP au Jetson.

**Ce que je fais, moi (logiciel) :**

- Un firmware ESP32 d'une quarantaine de lignes : `INPUT_PULLUP` sur les 4 entrées, anti-rebond 20 ms, et une ligne de texte envoyée à chaque changement (`B1:1`, `B1:0`…), plus la distance du sonar toutes les 100 ms.
- Côté Jetson : un lecteur Python qui ouvre le port série. **Zéro GPIO, zéro pinmux, zéro devmem, zéro sudo.**
- Une petite page de test pour identifier quel coin correspond à quel numéro, une bonne fois.

**Pourquoi ça ne peut pas re-échouer :**

- Il n'y a **plus d'adaptateur de niveau** entre le switch et la puce qui lit. C'est du silicium direct.
- Les rappels internes de l'ESP32 **existent vraiment et sont pilotables** — contrairement au Jetson, où la bibliothèque affiche littéralement *« Jetson.GPIO ignores setup()'s pull_up_down parameter »*.
- « Appuyé » = 0 Ω vers la masse. Il n'y a plus de combat de résistances, plus de zone grise, plus rien à latcher.
- L'anti-rebond tourne sur une boucle nue à quelques kHz, pas dans un Python sous Linux qui partage le processeur.
- Et surtout : **tu peux tout tester avant de le monter sur le rover.** Tu branches l'ESP sur n'importe quelle machine, tu ouvres le moniteur série, tu presses les coins à la main. Si ça marche là, ça marchera sur VECTOR — il n'y a plus aucune variable Jetson dans l'équation.

**Critère de recette (avant qu'on dise « ça marche ») :** chaque coin pressé sort son identifiant en moins de 50 ms et revient à 0 au relâchement, 10 fois sur 10, et les 4 coins gardent le même numéro après 3 redémarrages. Pas de « ça a l'air bon » — un aller-retour vérifié, coin par coin.

*Optionnel, si tu veux voir la preuve de tes yeux avant de dessouder (2 minutes, multimètre seul) : bus piloté, pointe sur un fil de signal, tu appuies sur ce coin et tu lis la tension. Si elle plafonne entre 1,0 et 1,5 V au lieu de monter à 3,3 V, le diagnostic est confirmé et le montage actuel est condamné quoi qu'on fasse côté logiciel.*

## 4. Plan B

**Si l'ESP32 ne se fait pas (pas de carte dispo, ou tu veux vraiment rester en direct sur le Jetson) : le montage canonique NVIDIA sur le 40-pin.**

- Le peigne de résistances **dégage** (les 3,3 k et 10 k sont la cause du problème, pas la solution).
- La chaîne des COM va à **GND** (broche 6, 9, 14, 20, 25, 30, 34 ou 39) — plus de « bus 3,3 V », la broche 32 est libérée.
- Chaque fil NO reçoit une résistance de **1 kΩ (ou 1,5 kΩ)** vers le **3,3 V** (broche 1 ou 17).
- Lecture en **actif-bas** : au repos la ligne lit 1, appuyé elle lit 0. Je réécris les scripts en conséquence.
- Et **avant de tirer des conclusions** : passer en JetPack 6.2.2, ou appliquer le patch noyau `sfsel` — sinon le bug GPIO de ta version brouillera les résultats et tu ne sauras pas qui blâmer.

Le sonar, lui, je le sortirais du Jetson dans tous les cas : mesurer une impulsion de quelques dizaines de microsecondes depuis Python sous Linux n'est pas fiable, indépendamment de toute cette histoire.

**Plan C (dernier recours, si même ça déraille) :** un petit circuit MCP23017 sur les broches I²C 27 et 28. Ce sont, avec les broches 3 et 5, les **seules** du connecteur reliées directement à la puce Jetson, sans adaptateur — la spec le dit note 2, et elles ont déjà 1,5 kΩ de rappel sur le module. Le problème du TXB0108 disparaît par construction.

---

**En une phrase :** ton montage était correct pour à peu près n'importe quelle carte du marché ; il est incompatible avec cette carte-là à cause d'une puce que NVIDIA a mise entre le connecteur et le processeur. On déplace les 4 switchs sur un ESP32 en USB, et le problème n'existe plus.