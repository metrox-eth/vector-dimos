# Draft issue for dimensionalOS/dimos — not sent

**Title:** Coordinator shutdown calls `module.stop()` through the RPC proxy without
checking it is an `@rpc`: it kills the worker, and still logs "Module stopped."

**Environment:** dimOS 0.0.14b1 (main `a7d19f76`), Python 3.12, Jetson Orin Nano Super,
JetPack 6.2, aarch64. External blueprint package (mecanum rover).

## What happens

`ModuleCoordinator.stop()` (`dimos/core/coordination/module_coordinator.py:108-114`)
walks the deployed proxies and calls `module.stop()`. The proxy is an `RPCClient`, whose
`__getattr__` turns the call into a real RPC only when the name is in `self.rpcs`
(`dimos/core/rpc_client.py:186`). `rpcs` is built from the attributes carrying `__rpc__`
(`dimos/core/module.py:321-329`), and that marker does not survive an override — a
subclass redefining `stop()` without re-applying `@rpc` silently drops `stop` from `rpcs`.
Checked on 0.0.14b1:

    class A(Module):
        @rpc
        def stop(self) -> None: ...
    class B(Module):
        def stop(self) -> None: ...          # override, no @rpc

    "stop" in A.rpcs  ->  True
    "stop" in B.rpcs  ->  False

For B the call falls through to the actor path: `Actor.__getattr__` sends a
`GetAttrRequest` (`dimos/core/coordination/python_worker.py:138`) and the worker answers
`WorkerResponse(result=getattr(instance, "stop"))` (:390) — a bound method, so the pickle
drags the whole module instance. Any module holding a lock or a thread fails:

    pickle.dumps(obj.stop)  ->  TypeError: cannot pickle '_thread.lock' object

That `conn.send(response)` is outside the per-request `try/except` (:432, which only
catches `BrokenPipeError`/`EOFError`), so the worker loop exits and the worker process
dies during shutdown, taking every module co-located in it with it. On the parent side the
`recv` fails and the loop logs, per module:

    Error stopping module    module=<name>
    Module stopped.          module=<name>

Line 114 runs unconditionally — including after the `except` — so a shutdown where nothing
stopped cleanly reads as a clean shutdown. Modules whose `start()` never completed get the
same treatment: `Deployed module.` then `Stopping module...` / `Module stopped.`, without
ever having started.

`_worker_entrypoint` still calls `instance.stop()` in its `finally` (:350-366), so the
modules of a worker that dies this way are stopped through the crash path rather than the
orderly one — after the loop has already blown up, with no ordering guarantee against the
parent's `WorkerManager.stop()` (5 s, then `terminate()`), and with the parent unable to
tell a real stop failure from this one.

The `stop()` that was dropped was ours (our camera module, fixed on our side). What cost
us the evening is that the framework never signalled it: not at deploy time, not at
shutdown, and the log said the modules had stopped.

## Repro

Two modules in the same worker, one overriding `stop()` without `@rpc` and holding a
`threading.Lock` (ours was a RealSense camera module). `dimos run <bp>`, then `dimos stop`
→ `Worker process error ... TypeError: cannot pickle '_thread.lock' object`, then
`Error stopping module` + `Module stopped.` for that module. The co-located module fails
next, since its own `stop` has no live worker left to answer it, and its `Module stopped.`
is logged all the same.

## Suggested fix

Cheapest first:

1. `module_coordinator.py:114` — log `Module stopped.` only when `stop()` returned; on the
   exception, say the module did not stop.
2. Do not let the stop call degrade into an attribute fetch. The worker already has an
   in-process route: `UndeployModuleRequest` calls `instance.stop()` directly
   (`python_worker.py:397-400`); `CallMethodRequest` is the same shape.
3. At deploy time, warn (or refuse) when `"stop"` is not in `cls.rpcs` — the coordinator
   already collects `rpc_names` for `ModuleDescriptor`.

Separately, `GetAttrRequest` returning a callable is a foot-gun for any caller: refusing
callables, or catching the pickling error around the response send, would keep a bad
request from taking the worker process down.
