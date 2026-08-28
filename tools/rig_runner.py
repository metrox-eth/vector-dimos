#!/usr/bin/env python3
"""Launch dimos on the RIG with a namespaced coordinator.

dimOS's CoordinatorRPC is a bus-wide singleton (NAME="Coordinator", upstream
main included - checked 27/08 22h29): the rig's coordinator probe finds the
ROVER's over the zenoh mesh and refuses to start. The upstream "run two
blueprints" flow needs a namespace this version lacks - question queued upstream.

Until then: rename the rig's coordinator BEFORE the CLI runs. Side benefit:
`dimos stop` on the rig speaks only to "CoordinatorRig" and can never stop
the rover's stack by accident.

The rename must survive `dimos restart`: the run registry stores this process's
sys.argv (RunEntry.original_argv) and restart stops the run, then
os.execvp(argv[0], argv). So argv[0] stays a runnable path to THIS file - with
"dimos" there the restart re-execs the plain CLI, the rig coordinator comes back
as "Coordinator", the probe finds the rover's over the mesh and the rig stack
stays down. Hence the shebang and the exec bit: launch through the path and the
restart resolves the same python3 the launch did.

Usage (instead of `dimos ...`), with the rig venv active:
    ./tools/rig_runner.py run vector-dimos.reloc-rig --daemon
"""
import sys
from pathlib import Path

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

CoordinatorRPC.NAME = "CoordinatorRig"

from dimos.cli.dimos import cli_main  # noqa: E402  (patch must precede the CLI)

if __name__ == "__main__":
    # NOT "dimos": this argv is what `dimos restart` re-execs.
    sys.argv[0] = str(Path(__file__).resolve())
    cli_main()
