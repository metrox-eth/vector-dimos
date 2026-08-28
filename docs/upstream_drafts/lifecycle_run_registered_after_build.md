# Draft issue for dimensionalOS/dimos — not sent

**Title:** `dimos run`: the run is registered only after the blueprint is built, so a
stack that hangs at startup cannot be stopped

**Environment:** dimOS 0.0.14b1 (main `a7d19f76`), Python 3.12, Jetson Orin Nano Super,
JetPack 6.2, aarch64. External blueprint package (mecanum rover), `dimos run <bp> --daemon`
over ssh.

## What happened

28 Aug 2026: two of our modules blocked inside `start()`. Each `start` RPC ran to its
1200 s timeout (`DEFAULT_RPC_TIMEOUTS["start"]`, `dimos/protocol/rpc/spec.py:40`):

    TimeoutError: RPC call to 'start' timed out after 1200.0 seconds

`ModuleCoordinator.build()` therefore did not return for ~20 minutes. During that whole
window, from a second ssh session:

    $ dimos status
    No running DimOS instance
    $ dimos stop
    No running DimOS instance          # exit 1
    $ ls ~/.local/state/dimos/runs
    (empty)
    $ pgrep -f '[b]in/dimos'
    (the daemon and its forkserver workers, all alive)

The stack was up, holding the motor bus and the cameras, and the CLI said nothing was
running. The only way out was `kill` by hand, then cleaning the workers by hand.

## Why

`dimos/cli/commands/lifecycle.py`, daemon path:

- 242 `coordinator = ModuleCoordinator.build(...)` — deploy + build + start, all of it
- 265 `entry.save()`
- 266 `spawn_watchdog(run_id, ...)`
- 267 `install_signal_handlers(entry, coordinator)`

Foreground path has the same order: 288 / 299 / 300.

`dimos stop` and `dimos status` resolve through `get_most_recent()`
(`dimos/core/run_registry.py:145`), which reads the registry directory, and
`cleanup_stale()` (:111) keys its descendant sweep off the same files. So for the entire
startup window there is no registry entry, no watchdog sidecar, and — on the daemon path —
no SIGTERM handler either: killing the daemon by hand skips `coordinator.stop()` and
`kill_run_processes()` (`dimos/core/daemon.py:124-135`), and the forkserver workers orphan
with their devices open.

`DIMOS_RUN_ID_ENV` is already exported before the fork (lifecycle.py:202), so the
descendants are tagged. Only the registry entry is missing.

## Repro

1. Blueprint with one module whose `start()` does `time.sleep(3600)`.
2. `dimos run <bp> --daemon` — the launcher stays blocked in `read_daemon_status`.
3. In another shell: `dimos status` → `No running DimOS instance`; `dimos stop` → same,
   exit 1; `ls ~/.local/state/dimos/runs` → empty; the daemon PID is alive.

Foreground (`dimos run <bp>`) is the same: the hang is before `entry.save()` at 299.

## Suggested fix

Register before building, in both paths: `entry.save()` and `spawn_watchdog()` above
`ModuleCoordinator.build()`. On the daemon path that is a move of a few lines inside the
same process (the entry is already written by the grandchild, after the fork), so the
recorded PID does not change. A SIGTERM handler installed at the same point would make a
hung startup killable through `dimos stop` — before `build()` returns there is no
coordinator to stop, so it can only remove the entry and call `kill_run_processes(run_id)`,
and be replaced by the full handler afterwards. A state field on `RunEntry` (`starting` /
`running`) would let `dimos status` say which one it is, and the existing `os._exit(1)`
failure paths would remove the entry.
