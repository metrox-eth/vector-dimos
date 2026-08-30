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
  (~~confondu par le soleil~~ — metrox 19h53 : le soleil était COUCHÉ à
  19h10, théorie solaire morte). Lecture corrigée : les trous suivent la
  CHARGE, pas la lumière — arrêt 25 W = déjà limite → trous ; arrêt MAXN =
  marge → propre ×2 ; VOL = pics de charge (planeur+carte+mouvement) → trous
  même sous MAXN. Prochains discriminants : un vol intérieur MAXN avec
  VECTOR_CAM_IMU=0 (l'IMU en vol reste suspecte), et/ou prioriser le worker
  caméra pendant les pics.

## 30/08/2026 20h30 — verdict de fin de journée (metrox) + le découpage prouvé

- **Bataille gagnée : la RealSense est revenue** — stable tout le vol, plus de
  clignotement (la garde 8°/s retirée, `01c0554`). **La guerre, non** : le
  comportement d'exploration est inchangé — « on a vraiment l'impression que
  la RealSense ne le renseigne pas sur les obstacles » (metrox).
- **L'autopsie choc-par-choc du dernier run le prouve et découpe en 2 familles**
  (carte fraîche 0,1-0,5 s à chaque choc — pas un problème de retard) :
  choc 1 : 0 cellule occupée devant = l'obstacle JAMAIS écrit (famille
  MAPPING : règle des deux points de vue trop lente en approche ? bande de
  hauteur qui croppe ?) ; chocs 2-3 : 13 et 3 cellules occupées devant = la
  carte savait, il est entré quand même (famille PLANEUR : inflation vs corps
  46 cm ? inertie 1,5 m ?).
- **Ouverture de la prochaine session (30 min, sans robot)** : rejouer la
  fenêtre du choc 1 depuis l'enregistrement — les points caméra de CET objet
  sont-ils dans le nuage fusionné /lidar ? OUI → le portier costmap les refuse
  (deux points de vue) ; NON → le crop dans lidar_odometry (bande z /
  OBSTACLE_MAX). Une réponse binaire, un fichier.

## 30/08/2026 20h37 — les objets des chocs nommés par l'opérateur + son test d'architecture

- **Choc 1 = pied de table ~25 mm** (à vue de nez). Plus fin qu'une cellule de
  carte (5 cm) — la classe exacte que la règle des deux points de vue peine à
  écrire. Le replay du choc 1 doit répondre : les points du pied sont-ils dans
  le nuage fusionné ? refusés par le portier ?
- **Choc 2 = LE SOFA — « aucune excuse »** : lidar le capte, la RealSense doit
  le capter, la carte AVAIT 13 cellules occupées… et le planeur est entré.
  Même à l'époque du sonar actif il se prenait le sofa. Famille conversion
  carte→action, la plus grave.
- **Un choc « contre mon pied »** : metrox l'a arrêté au pied (il lui arrivait
  dessus) — le réflexe bump-stop « marche assez bien ». Et des chocs NON
  comptés : il croit s'être fait rentrer dedans avant la cuisine — vérifier si
  l'enregistrement s'arrête avant la fin réelle du run.
- **Sa synthèse** : « tous nos capteurs fonctionnent mais ils ne sont jamais
  convertis en action ». Le rover navigue (au lidar, pense-t-il) ; le lidar 2D
  devrait CALER la RealSense, pas fabriquer le cadre d'obstacles.
- **Son test à monter** : navigation SANS le lidar comme source d'obstacles —
  lidar = ancrage SLAM seulement, la RealSense (+ son IMU) navigue seule,
  « comme à la période de Sam ». C'est le work-item (a) de sa doctrine du
  27/08 (ORDRES.md), resté inexécuté depuis — l'interrupteur était censé être
  câblé « au prochain vol ».

## 30/08/2026 20h52 — LA DOCTRINE VALIDÉE EN VOL (RealSense-only v1)

- **Verdict metrox, en direct** : « il explore, il voit les obstacles. Il a vu
  les pieds de la table. Il détecte beaucoup mieux, beaucoup plus loin, il est
  beaucoup plus précis dans ses manœuvres. Il descend la rampe magnifiquement
  bien. » `VECTOR_LIDAR_TO_MAP=0` : la caméra dessine le monde, le lidar cale
  la pose. La doctrine dite onze fois, validée au premier vol propre.
- Fin du run : porte des chiottes oubliée ouverte — il a fini AUX CHIOTTES.
  « Une belle fin. »
- **Réserves à régler (boutons nommés)** : (1) le sofa toujours invisible —
  mesurer UNE FOIS ce que la profondeur retourne face au sofa (tissu qui
  absorbe l'IR ? géométrie vs bande de hauteur ?) ; (2) « un petit peu trop de
  temps pour qu'un obstacle bloque » = `OCCUPIED_AT=2` + `NEW_VIEWPOINT_M=0.10`
  — baisser = blocage plus vif, plus de fantômes ; réglage fin à un vol.
- Leçon de câblage du soir : le premier interrupteur était au MAUVAIS étage
  (filtre du nuage dans lidar_odometry → pipeline affamé, carte morte) ; le
  bon étage existait déjà (`LIDAR_WRITES_OBSTACLES` dans costmap2d, prévu
  depuis le 27/08, jamais basculé). La règle du producteur a attrapé l'erreur
  en 10 min.
- **Requalification finale du choc-sofa (metrox, 20h56)** : le sofa EST écrit
  (le gros L rouge en haut à gauche de la carte) — le rover le percute parce
  que le voxel devient « obstacle » TROP TARD par rapport à l'approche. La
  chaîne du retard, additive : 2 impacts exigés depuis des points de vue
  écartés de 10 cm (`OCCUPIED_AT=2`, `NEW_VIEWPOINT_M=0.10`, ~0,5 s) +
  publication tous les 5 tours (`PUBLISH_EVERY=5`, ~0,6 s) + replanification +
  inertie 1,5 m au lâcher. Total 1-3 s ≈ 20-60 cm à vitesse de croisière.
  **Quatre boutons, une ligne chacun, un vol chacun** : OCCUPIED_AT 2→1 pour
  les impacts caméra ; NEW_VIEWPOINT 0.10→0.05 ; PUBLISH_EVERY 5→1-2 en
  autonomie ; rampe de décélération ZLAC (l'inertie).
- **Chantier téléop (session dédiée)** : la téléop use l'opérateur avec les
  défauts notés (dead band manquante, collision de commandes au ré-engagement,
  inertie 1,5 m). Faire une PASSE complète et comparer à l'ère Sam : a-t-on
  dérivé, ou y a-t-il simplement à améliorer.

## Vol de 21h03 — caméra seule + fast-block : RATÉ, long patinage (autopsie 21h20)

- Verdict metrox : « Le dernier run était complètement raté, long patinage. »
  E-stop manuel. La base (`explore.db`, 321 s) confirme : blocage t+124 s,
  pose figée (+1.02,-1.70) pendant 197 s jusqu'à la mort de la pile.
- **La RealSense a été parfaite tout le vol** : 804 camera_floor, médiane
  400 ms, zéro trou > 2 s. La bataille du clignotement tient.
- **La chaise de bureau n'a JAMAIS été peinte** : au blocage, cône 10-60 cm
  devant = 0 occupée / 70 libres ; après 3 min le nez dessus, carte finale =
  3 cellules. PAS un problème de délai — un TROU DE PERCEPTION courte portée :
  approche en pivot sur place (15 cm dans les 10 dernières s), la chaise est
  entrée dans la zone aveugle de profondeur sans avoir
  été peinte de loin — CORRECTION 21h25 (grille Intel) : à 640×480 la zone
  aveugle n'est que ~25-30 cm (MinZ = focale×baseline/126, linéaire en
  résolution) → le suspect n°1 remonte : la conversion bande-de-sol qui
  élague les pieds fins, pas la zone aveugle seule. Le disparity shift
  rapporte peu ici (shift 10 : mini 27 cm mais max ~3,7 m ; shift 50 :
  max ~0,7 m — il mangerait le « beaucoup plus loin » du vol 20h48) ; pieds bas et fins sous le pare-chocs ET sous la bande
  de sol convertie en obstacles.
- **Fast-block : verdict REPORTÉ** — jamais rencontré d'obstacle peint à
  bloquer. Pas d'explosion de fantômes (5,9 m² d'obstacles carte finale).
  À retester sur un vol à DEUX couches.
- **Ce que le raté prouve** : caméra seule = test d'isolation, pas un mode de
  prod. La colonne de chaise, le lidar la voit à 37 cm de haut à toute
  distance — exactement le trou que la couche lidar bouche. Doctrine deux
  couches renforcée.
- **Mystère e-stop (metrox, 21h07)** : patinage stoppé net mais moteurs
  restés RAIDES après le bouton physique. Un moteur sans courant est mou →
  signature d'un « quick stop » ZLAC (bouton sur l'entrée e-stop du drive,
  qui reste alimenté et enabled), pas d'un coupe-circuit. Côté logiciel :
  AUCUN paramètre ZLAC changé (seul registre jamais écrit : watchdog de comm
  0x2000=1000 ms à l'enable, commit a5711f3 du 22/08). Vérification câblage
  bouton en main, domaine metrox. Possible dérive pendant le rebuild des
  3 jours (bus HAT → dongle USB-RS485, 3ca55a4).

## E-stop : il fait L'INVERSE (metrox, bouton en main, 21h35)

- Fait observé : bouton APPUYÉ = torque tenu (robot indéplaçable) ; bouton
  RELÂCHÉ (position normale, armée) = torque libéré. Résolution du mystère
  de 21h07 : l'e-stop physique n'est PAS un coupe-circuit — il pilote l'état
  du drive, et à l'envers du modèle mental.
- ⚠ Question de sécurité à trancher au câblage (domaine metrox) : un e-stop
  de sécurité se câble NC (normalement fermé) — appuyer OUVRE le circuit,
  un câble arraché = arrêt (défaillance sûre). Si le comportement observé
  vient d'un câblage NO sur l'entrée e-stop du ZLAC, un câble coupé =
  PAS d'arrêt d'urgence. À vérifier avant le prochain vol armé.
