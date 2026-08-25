"""Cold tests EspSensors — parsing, filtre sonar, patchs monde. Regle #2."""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from vector_dimos.esp_sensors import (  # noqa: E402
    CORNERS, SONAR_MAX_TRUSTED, SonarFilter, parse_line,
)

FAIL = 0

def check(label, ok, detail=""):
    global FAIL
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if not ok:
        FAIL = 1

print("A. parse_line (lignes ESP reelles)")
check("SW nominal", parse_line("SW 0 1 0 0") == ("sw", (0, 1, 0, 0)))
check("SONAR nominal", parse_line("SONAR 0.263") == ("sonar", 0.263))
check("SONAR -1", parse_line("SONAR -1") == ("sonar", -1.0))
check("bruit -> None", parse_line("MUSEAU-ESP v2: boot") is None)
check("SW corrompu -> None", parse_line("SW 0 x 0 0") is None)

print("B. filtre sonar (mediane 3, cap 0,55, ecart 0,10)")
f = SonarFilter()
check("1 lecture -> None", f.feed(0.30) is None)
check("2 lectures -> None", f.feed(0.31) is None)
med = f.feed(0.29)
check("3 lectures stables -> mediane 0.30", med is not None and abs(med - 0.30) < 1e-9, f"{med}")
f2 = SonarFilter()
f2.feed(0.30); f2.feed(0.31)
check("au-dela du cap 0,55 -> rejete", f2.feed(0.80) is None)
f3 = SonarFilter()
f3.feed(0.20); f3.feed(0.45)
check("ecart >= 0,10 -> None", f3.feed(0.30) is None)
f4 = SonarFilter()
f4.feed(-1.0)
check("-1 (pas d'echo) ignore", len(f4._readings) == 0)

print("C. coins (carte validee 25/08)")
names = [c[0] for c in CORNERS]
check("ordre GPIO 1-4", names == ["avant-gauche", "arriere-gauche", "arriere-droit", "avant-droit"])
check("avant = x positif", CORNERS[0][1][0] > 0 and CORNERS[3][1][0] > 0)
check("arriere marques rear", CORNERS[1][2] and CORNERS[2][2] and not CORNERS[0][2])

print("D. patch monde (pose connue -> position connue)")
from vector_dimos.esp_sensors import EspSensors  # noqa: E402
import numpy as np  # noqa: E402
clouds = []
mic = EspSensors.__new__(EspSensors)
mic._pose = (1.0, 2.0, math.pi / 2)   # face +y
mic.world_frame = "world"
class FakeOut:
    def publish(self, c):
        clouds.append(c)
mic.lidar = FakeOut()
mic._publish_patch((0.30, 0.0))       # 30 cm devant -> monde (1.0, 2.3)
pts = clouds[0].as_numpy()[0]
if pts is None:
    check("patch publie 3x", len(clouds) == 3)
else:
    cx, cy = float(np.mean(pts[:, 0])), float(np.mean(pts[:, 1]))
    check("centre du patch = (1.0, 2.3)", abs(cx - 1.0) < 0.01 and abs(cy - 2.3) < 0.01, f"({cx:.2f}, {cy:.2f})")
check("3 publications (costmap veut 2 hits)", len(clouds) == 3)

sys.exit(FAIL)
