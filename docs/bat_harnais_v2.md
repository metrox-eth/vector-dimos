# BAT — Harnais v2 (monde sans HAT) — à vérifier par metrox avant tout câblage

> Bon à tirer du 25/08/2026. Chaque ligne porte sa source : [mesuré] = vérifié
> électriquement les 24-25/08 ; [doc] = documentation officielle ; **[ROUGE]** =
> incertitude à lever AVANT de câbler, jamais à deviner.
> Rien ne se branche avant le « oui c'est ça » de metrox.

## 0. Décisions de base

| Décision | Statut |
|---|---|
| Le HAT Waveshare RS485/CAN est retiré | décidé 25/08 |
| Moteurs ZLAC → dongle **USB-RS485 Waveshare** (A/B différentiel) | branché par metrox |
| Lidar C1 → adaptateur **USB-TTL HW-597B** (CH340, pilote déjà installé [mesuré]) | livré |
| Capteurs museau (4 switchs + sonar) → 40 broches en direct | inchangé |
| Shunt PZEM-017 → son dongle USB-RS485 (CH340) | en place [mesuré] |

## 1. Adaptateur lidar HW-597B — câblage

| Fil lidar C1 | Broche HW-597B | Source |
|---|---|---|
| (jumper jaune) | **VCC ↔ 3V3** — logique 3,3 V, OBLIGATOIRE (RX du C1 = 3,5 V max) | [doc module + manuel SLAMTEC] |
| 5V | 5V (vrai 5 V USB, indépendant du jumper) | [doc] |
| GND | GND | [doc] |
| TX (lidar) | RXD (croisé) | [doc] |
| RX (lidar) | TXD (croisé) | [doc] |

Conséquence : le lidar n'a PLUS besoin du 5 V des broches 2/4 du Jetson —
il est alimenté par l'USB. Les deux broches 5 V du header redeviennent libres.

## 2. Header 40 broches — carte finale

Bloc A (5 fils, rangée impaire) : **INCHANGÉ, sur 29-31-33-35-37.**
Le HAT parti, la broche 29 n'est plus squattée par son signal CAN.
**[ROUGE] À valider à la 1re mise sous tension, sans toucher au matériel :
lecture de repos de la 29 — attendue basse. Si elle reste haute sans le HAT,
on décale UNE décision, pas les fils, et on repasse par ce BAT.**

| Broche | Rôle | Polarité au repos | Source |
|---|---|---|---|
| 29 | switch 1 | **[ROUGE]** à lire post-HAT | — |
| 31 | switch 2 | HAUT au repos (câblé NC, appui → bas) | [mesuré 24/08] |
| 33 | switch 3 | bas au repos (câblé NO, appui → HAUT) | [mesuré 24/08] |
| 35 | switch 4 | bas au repos (NO, appui → HAUT) | [mesuré 24/08] |
| 37 | ECHO sonar | bas au repos | [mesuré 24/08] |
| 32 | bus COM switchs (sortie 3,3 V logicielle) | déverrouillée à chaque boot (cron devmem, en place) | [mesuré] |
| 34 | masse commune (sonar + diviseur éventuel) | — | [doc] |
| 36 | TRIG sonar (sortie) | déverrouillée au boot (cron) | [mesuré] |
| 2 ou 4 | VCC sonar 5 V (les DEUX libres désormais) | — | [doc] |
| 1 / 17 | 3,3 V (option sonar si essai 3,3 V retenu) | — | [doc] |

La correspondance coin-physique ↔ broche = les 4 appuis isolés (2 min,
logiciel prêt), APRÈS remontage, avant fermeture du capot.

**[ROUGE] Sonar : lisait « 0,00 m » le 24/08.** Suspect n° 1 : alimentation
(3,3 V insuffisant vs 5 V). À trancher AVANT de refermer le harnais :
VCC sur broche 2/4 (5 V) + pont diviseur sur ECHO, OU modèle 3,3 V confirmé.
Décision metrox au moment du remontage.

## 3. Plan des ports USB (stabilité des noms)

Trois adaptateurs série au moins, dont DEUX puces CH340 identiques (lidar +
shunt) qui n'ont pas de numéro de série → l'ordre /dev/ttyUSBx est une
loterie à chaque boot. **Parade : chaque adaptateur a SON port USB physique
attitré, et le logiciel cible par chemin physique (by-path), pas par numéro.**

| Port physique Jetson | Périphérique | Notes |
|---|---|---|
| USB-A n°1 (à définir par metrox) | dongle RS-485 moteurs | **[ROUGE] puce à identifier au 1er branchement** (FTDI = idéal, CH34x = latence à mesurer) |
| USB-A n°2 | adaptateur TTL lidar | CH340 |
| USB-A n°3 | dongle RS-485 shunt | CH340 |
| USB 3 (bleu) | RealSense D455F | inchangé |
| (hub si besoin) | ReSpeaker, joypad | inchangé |

Une fois les ports choisis PHYSIQUEMENT (étiquette sur chaque câble),
je fige la table by-path dans le code : plus jamais de loterie.

**[ROUGE] Cadence moteurs : l'UART matériel faisait 6-7 ms par moteur
[mesuré 22/08]. Le passage en USB ajoute de la latence — je la MESURE au banc
encodeurs (roues en l'air, procédure du 22/08) avant tout roulage.**

## 4. Vérifications — UNE passe, dans l'ordre

Avant mise sous tension (multimètre, rover éteint) :
1. Continuité broche 32 → chaque COM (bus entier).
2. `COM–NO` / `COM–NC` de chaque switch : identifier la patte réellement
   câblée (le montage miroir du 24/08 rend les deux valides — on NOTE, on ne
   recâble pas).
3. Aucune continuité 5 V ↔ GND, ni 3,3 V ↔ GND.
4. ECHO (broche 37) : continuité avec le fil echo du sonar uniquement.

À la 1re mise sous tension (logiciel seul, personne ne touche) :
5. Lecture de repos des 6 broches (29 comprise — lève le ROUGE n° 1).
6. Énumération USB : identités des 3 adaptateurs → je fige la table by-path.
7. Banc encodeurs roues en l'air → latence bus moteurs (lève le dernier ROUGE).

## 5. Ce que je change côté code (après signature du BAT)

- `adapter.py` / `zlac8015d.py` : port moteurs → by-path (fini ttyTHS1).
- `c1_serial.py` : port lidar → by-path.
- `bumper.py` : coins + polarité PAR switch (table des 4 appuis), echo 37.
- Réflexe directionnel : choc avant → recule, choc arrière → avance.
