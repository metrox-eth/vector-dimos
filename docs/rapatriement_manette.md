# Rapatriement à la manette après un run (metrox, 30/08/2026 : « que ce soit clean, qu'on ait un flightcheck pour ça »)

Le cas : un run d'exploration finit ailleurs qu'au point de départ (la cuisine
en bas de la rampe de 10 cm, 30/08). L'opérateur reprend la main à la manette
pour ramener le rover. La transition passe TOUJOURS par les portes — jamais de
pilotage sur une stack d'exploration encore vivante.

## La séquence, depuis le rig

1. **Fin du run** (trois gestes, dans cet ordre, le même que fly.sh imprime) :
   ```
   ssh metrox@192.168.0.56 'cd ~/vector-dimos && TRANSPORT=lcm .venv/bin/python tools/explore_ctl.py stop'
   ssh metrox@192.168.0.56 'cd ~/vector-dimos && .venv/bin/python tests/estop_rs485.py && .venv/bin/dimos stop'
   ```
   Un `E-STOP INCOMPLETE` pendant que la stack meurt est normal (le bus est
   encore tenu) : rejouer l'e-stop après `dimos stop` → `E-STOP DONE`.
2. **Manette prête** : le récepteur USB est sur le Jetson et `js0` existe
   (`ssh … ls /dev/input/js0`). S'il est branché mais invisible (vécu 30/08),
   c'est la couche USB du Jetson qui l'a perdu → `sudo reboot` du Jetson,
   ~1 min, re-vérifier.
3. **Relance en mode manette** :
   ```
   GAMEPAD=1 REPOSITIONNE=1 tools/fly.sh
   ```
   Les portes refont TOUT le flightcheck (dont sweep fantômes/zombies porte 0
   et présence de js0 porte 1 — elle refuse sans manette, c'est voulu). Mode
   DRY : la stack monte, les écrans reviennent, AUCUNE exploration ne part.
4. **Piloter** : homme-mort tenu, sticks (Y gauche = avancer, X = strafe).
   Manette au repos = zéro commande ; relâcher l'homme-mort = 0,5 s de frein
   puis silence.
5. **Fin du rapatriement** : reposer le rover à son point de départ, puis les
   deux gestes du 1 (e-stop, stack) — le prochain run repart des portes.

## Pièges connus

- La rampe de 10 cm SE MONTE très bien à la manette (caoutchouc, pilotage
  franc — mesuré 30/08). Le patinage n'arrive qu'avec les hésitations du
  pilotage autonome (stop-start dans la pente) : limite de style de conduite,
  pas de mécanique.
- Le pilotage n'écrit PAS la carte persistante : la fenêtre d'écriture est
  fermée sans `explore_cmd` (commit bef2238) — un rapatriement ne grave rien.
- Ne jamais brancher/tester la manette pendant qu'une chaîne est ARMÉE
  (incident rover 27/08 : jamais de neuf dans une chaîne armée).
