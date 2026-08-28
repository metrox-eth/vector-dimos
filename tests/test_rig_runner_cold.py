#!/usr/bin/env python3
"""Cold bench for tools/rig_runner.py: the rename must survive `dimos restart`.

The runner exists to set CoordinatorRPC.NAME="CoordinatorRig" before the CLI
runs. dimOS records the launching sys.argv in the run registry
(lifecycle.py: RunEntry(original_argv=sys.argv)) and `dimos restart` stops the
run then os.execvp(argv[0], argv). Whatever argv[0] holds IS the restarted rig
stack.

The REAL runner runs here, against a stub `dimos` package on PYTHONPATH (no
zenoh, no rover, no CLI): the stub's cli_main writes what lifecycle would have
recorded - original_argv + the CoordinatorRPC.NAME in force. The restart is then
replayed exactly as lifecycle.restart does it, os.execvp on that recorded argv,
with a fake `dimos` console script first on PATH standing in for the rig venv's
(it imports the stub CLI *without* the rename, like the real one).

  A. the file is re-execable at all - shebang + exec bit (execvp needs both)
  B. launch      - known input argv -> the CLI sees "run vector-dimos.reloc-rig
                   --daemon" untouched and the coordinator is named CoordinatorRig
  C. the registry- recorded argv[0] resolves to the runner, not to "dimos"
  D. restart     - execvp(recorded argv) -> CoordinatorRig again (the bug: with
                   argv[0]="dimos" the fake console script answers and the
                   restarted coordinator is plain "Coordinator")
  E. twice       - the restarted process records the same argv, so restart n+1
                   still lands on the runner

Run:   PYTHONPATH=. .venv/bin/python3 tests/test_rig_runner_cold.py
       ... tests/test_rig_runner_cold.py <a copy of rig_runner.py>   (this is how
       the pre-fix bite was run on 2026-08-28: `git show HEAD:tools/rig_runner.py`
       failed A, C, D and E - restart came back as "Coordinator".)

Live-only, deferred: that the rover's coordinator really answers the rig's probe
over the zenoh mesh. The rover is not reachable from the bench.
"""

import json
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
RUNNER = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else ROOT / "tools" / "rig_runner.py"
ARGS = ["run", "vector-dimos.reloc-rig", "--daemon"]
OK = 0
KO = 0

TMP = tempfile.TemporaryDirectory()
TMPD = Path(TMP.name)
STUB = TMPD / "stub"
BIN = TMPD / "bin"

# --- stub `dimos`: the two names the runner imports, and a cli_main that
# records exactly what lifecycle.run() would have put in the run registry.
CLI_MAIN = '''
import json, os, sys
from dimos.core.coordination.coordinator_rpc import CoordinatorRPC


def cli_main():
    """Stand-in for dimos.cli.dimos.cli_main + lifecycle.run()'s RunEntry."""
    path = os.environ["RIG_TEST_REGISTRY"]
    with open(path, "w") as f:
        json.dump({"original_argv": list(sys.argv), "name": CoordinatorRPC.NAME}, f)
'''
for pkg in ("dimos", "dimos/core", "dimos/core/coordination", "dimos/cli"):
    (STUB / pkg).mkdir(parents=True, exist_ok=True)
    (STUB / pkg / "__init__.py").write_text("")
(STUB / "dimos/core/coordination/coordinator_rpc.py").write_text(
    'class CoordinatorRPC:\n    NAME = "Coordinator"\n'
)
(STUB / "dimos/cli/dimos.py").write_text(CLI_MAIN)

# --- the rig venv's `dimos` console script, stubbed: the real one imports
# cli_main directly, so no rename. This is what execvp("dimos", ...) finds.
BIN.mkdir()
DIMOS_BIN = BIN / "dimos"
DIMOS_BIN.write_text(
    "#!/usr/bin/env python3\n"
    "import sys\n"
    "from dimos.cli.dimos import cli_main\n"
    "sys.exit(cli_main())\n"
)
DIMOS_BIN.chmod(0o755)

# --- lifecycle.restart's tail, verbatim: os.execvp(argv[0], argv).
EXECVP = TMPD / "execvp_restart.py"
EXECVP.write_text("import json, os, sys\nargv = json.loads(sys.argv[1])\nos.execvp(argv[0], argv)\n")


def check(label, ok, detail=""):
    global OK, KO
    print(f"  {'OK ' if ok else 'KO '} {label}{' - ' + detail if detail else ''}")
    if ok:
        OK += 1
    else:
        KO += 1


def env(registry):
    e = dict(os.environ)
    e["PYTHONPATH"] = str(STUB)
    e["PATH"] = f"{BIN}{os.pathsep}{os.environ['PATH']}"
    e["RIG_TEST_REGISTRY"] = str(registry)
    return e


def registry_after(cmd, registry, e):
    """Run cmd, return (entry|None, stderr tail)."""
    registry.unlink(missing_ok=True)
    p = subprocess.run(cmd, env=e, capture_output=True, text=True, timeout=60)
    if not registry.exists():
        return None, (p.stderr or p.stdout).strip().splitlines()[-1:] or ["<no output>"]
    return json.loads(registry.read_text()), []


print(f"rig_runner: {RUNNER}")

print("A. re-execable")
head = RUNNER.read_text().splitlines()[0] if RUNNER.exists() else ""
check("shebang on line 1", head.startswith("#!"), f"line 1: {head!r}")
check("exec bit set", os.access(RUNNER, os.X_OK), f"mode {oct(RUNNER.stat().st_mode & 0o777)}")

print("B. launch")
reg1 = TMPD / "reg1.json"
entry, err = registry_after([sys.executable, str(RUNNER), *ARGS], reg1, env(reg1))
check("the CLI ran", entry is not None, " ".join(err))
if entry is None:
    print(f"{OK} OK, {KO} KO")
    print("TEST FAILED")
    sys.exit(1)
check("the CLI got the blueprint args", entry["original_argv"][1:] == ARGS, str(entry["original_argv"][1:]))
check('coordinator named "CoordinatorRig"', entry["name"] == "CoordinatorRig", entry["name"])

print("C. what the run registry stores")
argv0 = entry["original_argv"][0]
same = Path(argv0).resolve() == RUNNER if os.sep in argv0 else False
check("original_argv[0] is this runner", same, f"argv[0]={argv0!r}")

print("D. dimos restart (execvp on the recorded argv)")
reg2 = TMPD / "reg2.json"
entry2, err = registry_after(
    [sys.executable, str(EXECVP), json.dumps(entry["original_argv"])], reg2, env(reg2)
)
check("the restart came up", entry2 is not None, " ".join(err))
check(
    'restarted coordinator still "CoordinatorRig"',
    entry2 is not None and entry2["name"] == "CoordinatorRig",
    (entry2 or {}).get("name", "<never started>"),
)
check(
    "the restart kept the blueprint args",
    entry2 is not None and entry2["original_argv"][1:] == ARGS,
    str((entry2 or {}).get("original_argv", [])[1:]),
)

print("E. restart is repeatable")
check(
    "the restarted process records the same argv",
    entry2 is not None and entry2["original_argv"] == entry["original_argv"],
    f"{(entry2 or {}).get('original_argv')} vs {entry['original_argv']}",
)

print(f"{OK} OK, {KO} KO")
print("TEST PASSED" if KO == 0 else "TEST FAILED")
sys.exit(1 if KO else 0)
