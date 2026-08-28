#!/usr/bin/env python3
"""Cold bench for the two open3d-CUDA build ovens (tools/four_open3d_cuda.sh,
tools/four_open3d_cuda_vast.sh). The ovens themselves are hours of nvcc: nothing
is compiled here. What runs is the cheap logic they carry.

  A. bash -n on both ovens
  B. the native oven carries the Pair.cuh patch, BEFORE it configures cmake
  C. the patch block, extracted from the native oven and run on a real v0.19.0
     Pair.cuh fixture: known input `constexpr __device__ inline Pair() {}` ->
     known output `Pair() = default;`, idempotent, and the grep gate exits 1
     when the sed matches nothing (upstream renamed the line = fail in seconds,
     not after hours of make)
  D. the vast oven's delivery gate refuses an empty / missing .so path. Without
     that guard `ctypes.CDLL("")` is dlopen(NULL): it loads the main program and
     the oven logs "CHARGEMENT OK - roue saine" for a wheel it never opened
  E. the vast oven keeps its own Pair.cuh patch

Run:   PYTHONPATH=. .venv/bin/python3 tests/test_four_ovens_cold.py
       ... tests/test_four_ovens_cold.py <native.sh> <vast.sh>   (points the
       bench at copies - how the pre-fix bite was run on 2026-08-28: the
       pre-fix pair failed B, C and D.)
"""

import ctypes
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NATIVE = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "tools" / "four_open3d_cuda.sh"
VAST = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else ROOT / "tools" / "four_open3d_cuda_vast.sh"
OK = 0
KO = 0

# cpp/open3d/core/nns/kernel/Pair.cuh, Open3D v0.19.0, the part that matters
PAIR_CUH = """// Open3D v0.19.0 excerpt
template <typename K, typename V>
struct Pair {
    constexpr __device__ inline Pair() {}

    constexpr __device__ inline Pair(K key, V value) : k(key), v(value) {}

    K k;
    V v;
};
"""

TMPD = Path(tempfile.mkdtemp(prefix="four_ovens_"))


def check(label, cond, detail=""):
    global OK, KO
    if cond:
        OK += 1
        print(f"  OK   {label}")
    else:
        KO += 1
        print(f"  KO   {label}" + (f" -- {detail}" if detail else ""))


def lines_of(script):
    return script.read_text().splitlines()


def grep_line(script, needle):
    """The first line of the script containing needle, with its 1-based index."""
    for i, line in enumerate(lines_of(script), 1):
        if needle in line:
            return i, line
    return 0, ""


def run_fragment(body, cwd, env_line=""):
    """Run script lines lifted from an oven, with log() stubbed."""
    frag = TMPD / "frag.sh"
    frag.write_text('log() { echo "[log] $*"; }\n' + env_line + body + "\n")
    return subprocess.run(["bash", str(frag)], cwd=str(cwd), capture_output=True, text=True, timeout=30)


print(f"native: {NATIVE}")
print(f"vast:   {VAST}")

print("A. the ovens parse")
for f in (NATIVE, VAST):
    check(f"bash -n {f.name}", subprocess.run(["bash", "-n", str(f)]).returncode == 0)

print("B. the native oven carries the Pair.cuh patch, before cmake")
sed_no, sed_line = grep_line(NATIVE, "Pair.cuh")
gate_no, gate_line = grep_line(NATIVE, 'grep -q "Pair() = default"')
cmake_no, _ = grep_line(NATIVE, "-DCMAKE_CUDA_ARCHITECTURES=87")
make_no, _ = grep_line(NATIVE, "make -j")
check("sed on cpp/open3d/core/nns/kernel/Pair.cuh", "sed -i" in sed_line, f"line {sed_no}: {sed_line!r}")
check("the patch is hard-gated (grep || exit 1)", gate_no > 0 and "exit 1" in gate_line, f"line {gate_no}")
check("the patch runs before cmake", 0 < sed_no < cmake_no, f"sed {sed_no}, cmake {cmake_no}")
check("the patch runs before make", 0 < gate_no < make_no, f"gate {gate_no}, make {make_no}")

print("C. the patch block on a real Pair.cuh (known input -> known output)")
if sed_no and gate_no:
    block = "\n".join(lines_of(NATIVE)[sed_no - 1:gate_no])
    src = TMPD / "cpp" / "open3d" / "core" / "nns" / "kernel"
    src.mkdir(parents=True, exist_ok=True)
    pair = src / "Pair.cuh"

    pair.write_text(PAIR_CUH)
    p = run_fragment(block, TMPD)
    got = pair.read_text()
    check("nvcc-rejected ctor is gone", "constexpr __device__ inline Pair() {}" not in got)
    check("replaced by `Pair() = default;`", "Pair() = default;" in got, got)
    check("the other ctor is untouched", "Pair(K key, V value) : k(key), v(value) {}" in got)
    check("the block exits 0", p.returncode == 0, p.stdout + p.stderr)

    p = run_fragment(block, TMPD)          # second pass on the patched file
    check("idempotent (re-run exits 0, file unchanged)",
          p.returncode == 0 and pair.read_text() == got, p.stdout + p.stderr)

    pair.write_text("struct Pair { Pair(K k) {} };\n")   # upstream renamed the line
    p = run_fragment(block, TMPD)
    check("the gate bites when the sed matches nothing (exit 1)", p.returncode == 1,
          f"rc {p.returncode}: {p.stdout + p.stderr}")
else:
    check("the patch block exists to run", False, "no sed/grep pair found in the native oven")

print("D. the vast oven's delivery gate refuses an empty .so path")
try:
    ctypes.CDLL("")
    print('  note CDLL("") loaded the main program here: unguarded, the gate is a false green')
except OSError as e:
    print(f'  note CDLL("") raised on this interpreter ({e}); the guard still owns the missing-file case')
guard_no, guard_line = grep_line(VAST, '[ -n "$SO" ]')
check("the gate has an empty-path guard", guard_no > 0, "no `[ -n \"$SO\" ]` before the load check")
cdll_no, _ = grep_line(VAST, "ctypes.CDLL")
check("the guard runs before the CDLL check", 0 < guard_no < cdll_no, f"guard {guard_no}, CDLL {cdll_no}")
if guard_no:
    real = TMPD / "pybind.so"
    real.write_bytes(b"\x7fELF")
    for so, want, why in (("", 1, "empty path"),
                          (str(TMPD / "absent.so"), 1, "missing file"),
                          (str(real), 0, "the .so is there")):
        p = run_fragment(guard_line, TMPD, env_line=f'SO="{so}"\n')
        check(f"{why} -> exit {want}", p.returncode == want, f"rc {p.returncode}: {p.stdout + p.stderr}")

print("E. the vast oven keeps its own patch")
check("vast: sed on Pair.cuh", grep_line(VAST, "Pair.cuh")[0] > 0)
check("vast: patch hard-gated", 'grep -q "Pair() = default"' in VAST.read_text())

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
