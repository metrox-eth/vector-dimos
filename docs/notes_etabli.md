# Notes d'établi (observations terrain de metrox — on documente, on ne règle pas forcément tout de suite)

## 30/08/2026 — après le vol de validation 17h35 et le rapatriement manette

- **La rampe de 10 cm SE MONTE très bien en téléopération** : caoutchouc dessus,
  aucun patinage quand le pilotage est franc. Le patinage n'apparaît qu'avec
  les hésitations du pilotage AUTONOME (stop-start dans la pente). Corrige la
  note « aller simple » de rapatriement_manette.md : le franchissement n'est
  pas une limite mécanique, c'est une limite de style de conduite.
  → Piste future : un profil « frans » (pas d'arrêt) sur les pentes détectées,
  ou zone de conduite spéciale — PAS maintenant.
- **Le cockpit dIMOS (image RealSense) lague par moments** : l'image gèle ou
  disparaît pendant que le rover continue d'explorer (on l'entend). Affichage
  seulement — la profondeur publie normalement dans les logs pendant ces gels.
  → À observer ; pas bloquant, pas de réglage immédiat.
- **Faux « MORT » de la vigie/panneau** (4 occurrences ce jour) : les seuils de
  fraîcheur du panneau déclenchent sur des ralentissements passagers (odométrie
  à 130 ms/tour sous charge) et sur le silence par-design de la manette
  (homme-mort relâché = zéro commande). → Chantier vigie : seuils par mode.
- **Le pilotage manette écrit la carte** (ouvre-secours 5 cm de bef2238) :
  vu à l'écran pendant le rapatriement. Question de conception ouverte pour
  PERSISTENT_MAP=1 : cartographie manuelle voulue, ou gel hors mission ?

## 30/08/2026 — pendant le rapatriement manette n°2 (metrox pilote)

- **Cockpit : 1 image/seconde suffirait** à l'opérateur (pas flatteur pour les
  vidéos réseaux sociaux, mais opérationnel). Questions à instruire : format
  d'image ? flux image-par-image (pas un flux vidéo) = images lourdes sur le
  wifi ; qu'est-ce qui mange la bande passante, d'où vient le lag (18-37 s vus
  pendant le run 2) ; le paramétrage est-il optimal ?
- **Téléop : il manque une dead band en translation** (petits mouvements de
  stick autour du neutre → le rover bouge alors qu'on veut le neutre).
- **Transitions de commandes pas propres** : relâcher un stick puis le remettre
  vite → lag / « collision de commandes ». Suspect : le frein de 0,5 s de zéros
  post-homme-mort qui se superpose aux nouvelles commandes.
- Le bon moment : « c'est comme si je jouais avec une voiture télécommandée de
  27 kg » — la téléop marche, elle est amusante.
