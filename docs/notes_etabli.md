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

## 30/08/2026 soir — le clignotement caméra DATÉ et suspecté

- **Metrox avait raison depuis le début** : le clignotement RealSense est né
  pendant les 3 jours de rebuild. Claude-mem + git ont daté le changement :
  **24/08, commit 964a0a5 — VectorCamera fait passer l'IMU 200 Hz par le MÊME
  pipeline que la profondeur** (le 2e pipeline dimOS plantait sur le build
  RSUSB). Profondeur + motion entrelacées sur un pipeline RSUSB = recette
  connue des gels librealsense de plusieurs secondes.
- Le fix anti-flicker du 27/08 (cache 0,5 s, dd1cbb3) est toujours en place
  mais ne couvre que le battement 10 vs 7,5 Hz — pas les VRAIS trous mesurés :
  9 trous >2 s (max 10,3 s) au run 17h35, 42 trous (max 19 s) au run 19h05.
  Le lidar, lui, est parfait dans les deux (zéro trou).
- **Test à une variable, prêt à coder (~5 lignes)** : env pour sauter
  `_start_imu` → un vol → si la caméra devient continue, coupable signé.
  ⚠ Coût du test : sans cette IMU, plus de « prior gyro » pour kiss-icp ni
  d'ImuSlipDetector → pose dégradée PENDANT ce vol (diagnostic, pas remède).
- Remèdes candidats si signé : baisser la cadence IMU (`config.imu_hz`, bouton
  existant), pipeline/thread séparé, ou sortir du build RSUSB (kernel UVC).
- **Inertie téléop excessive** (rapatriement 19h25) : stick lâché → le rover
  glisse encore ~1,5 m. Pas le cas à l'époque de la pile ROS de Sam. Suspects :
  la manette n'envoie que 0,5 s de zéros puis silence (BRAKE_S), et la rampe de
  décélération ZLAC (`accel_ms` de l'adaptateur) adoucit l'arrêt → roue libre.
  Bouton probable : rampe de décel plus courte ou frein actif au lâcher.
  À côté : verdict positif du pilote — très puissant, franchit tout, bumpers
  rassurants ; « un petit peu massif donc un petit peu dangereux ».
- **A/B IMU immobile (19h36-19h50, MAXN)** : manche A (IMU ON) et manche B
  (IMU OFF) toutes deux PARFAITES — zéro trou >2 s, max 0,6 s sur ~6 min
  chacune. L'IMU est innocentée À L'ARRÊT sous MAXN. Matrice d'état :
  arrêt 25 W = trous (max 10,3 s) ; arrêt MAXN = zéro trou ×2 ; vol 25 W
  intérieur = 9 trous ; vol MAXN avec passage plein soleil = 42 trous
  (confondu par le soleil sur l'IR). **Case manquante : un vol MAXN en
  intérieur, IMU ON** — propre = MAXN a guéri les trous et l'IMU est
  innocente ; troué = refaire le même vol avec VECTOR_CAM_IMU=0.
