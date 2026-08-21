"""Cold bench: known twists -> known wheel patterns, and roundtrip identity."""
import sys; from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from vector_dimos.kinematics import MecanumGeometry, forward, inverse

G = MecanumGeometry(wheel_radius_m=0.09, half_wheelbase_m=0.25, half_track_m=0.25)
ok = True


def check(cond, label):
    global ok
    print(("  OK  " if cond else "  KO  ") + label)
    ok = ok and cond


# pure forward: all wheels equal and positive
w = inverse(1.0, 0.0, 0.0, G)
check(all(abs(x - w[0]) < 1e-12 for x in w) and w[0] > 0,
      "pure vx -> 4 equal forward wheels")
# pure strafe: FL/RR negative, FR/RL positive (the mecanum signature)
w = inverse(0.0, 1.0, 0.0, G)
check(w[0] < 0 and w[1] > 0 and w[2] > 0 and w[3] < 0,
      "pure vy -> diagonal pattern (FL-,FR+,RL+,RR-)")
# pure spin: left side backward, right side forward
w = inverse(0.0, 0.0, 1.0, G)
check(w[0] < 0 and w[1] > 0 and w[2] < 0 and w[3] > 0,
      "pure wz -> left back, right forward")
# roundtrip identity on an arbitrary twist (rule #2: known in -> known out)
tw = (0.37, -0.21, 0.83)
back = forward(*inverse(*tw, G), G)
check(all(abs(a - b) < 1e-12 for a, b in zip(tw, back)),
      f"roundtrip {tw} -> wheels -> {tuple(round(x, 4) for x in back)}")

print("\nTEST " + ("PASSED" if ok else "FAILED"))
raise SystemExit(0 if ok else 1)
