"""Launch dimos on the RIG with a namespaced coordinator.

dimOS's CoordinatorRPC is a bus-wide singleton (NAME="Coordinator", upstream
main included - checked 27/08 22h29): the rig's coordinator probe finds the
ROVER's over the zenoh mesh and refuses to start. lesh's "run two blueprints"
story needs a namespace this version lacks - question queued for him.

Until then: rename the rig's coordinator BEFORE the CLI runs. Side benefit:
`dimos stop` on the rig speaks only to "CoordinatorRig" and can never stop
the rover's stack by accident.

Usage (instead of `dimos ...`):
    .venv/bin/python tools/rig_runner.py run vector-dimos.reloc-rig --daemon
"""
import sys

from dimos.core.coordination.coordinator_rpc import CoordinatorRPC

CoordinatorRPC.NAME = "CoordinatorRig"

from dimos.cli.dimos import cli_main  # noqa: E402  (patch must precede the CLI)

if __name__ == "__main__":
    sys.argv[0] = "dimos"
    cli_main()
