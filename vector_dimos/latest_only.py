"""Latest-only delivery of velocity commands, on any transport.

Why. lcm_latest bounds the LCM queue of the command channels to one message,
and that clamp is the whole guard against stale twists (2026-08-22: the
coordinator fell behind, a 1 s command executed for 1.9 s, 2.08 m driven for
0.9 m asked). It is LCM-only. Under TRANSPORT=zenoh the same commands ride
ZenohPubSub, which offers nothing to clamp: `subscribe()` declares a plain
callback subscriber (dimos/protocol/pubsub/impl/zenohpubsub.py), dimOS never
hands zenoh a RingChannel/FifoChannel handler, and the one per-topic knob,
`Topic.qos`, configures the PUBLISHER (reliability, congestion control), not
the subscriber. Zenoh then delivers the backlog in order, in full, late.

How. Consumer side, so it holds whatever the transport is. The subscriber
callback becomes a slot of ONE: store the newest message, wake the drain
thread, return. The consumer runs on that drain thread and always takes the
newest message pending, so a command arriving while the consumer is busy
replaces the one waiting - nothing older than one publish period is executed.
Same net effect as an LCM capacity of 1, and the same structure dimOS itself
uses in ZenohPubSubBase.subscribe_all ("delivering only the latest per
topic"). A Twist carries no timestamp, so "newest" is arrival order in this
process, nothing else.

Only a velocity command may be dropped this way: a superseded twist has no
effect its successor does not have. The zero that ends a move is never dropped
in favour of motion - it IS the newest message - and JointVelocityTask's 0.2 s
watchdog still zeroes the base if the stream stops.
"""
from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable

DROP_LOG_PERIOD_S = 1.0
JOIN_TIMEOUT_S = 2.0

_EMPTY = object()


def _log() -> Any:
    try:
        from dimos.utils.logging_config import setup_logger
        return setup_logger()
    except Exception:
        return logging.getLogger(__name__)


class LatestOnly:
    """Callable: messages in from any thread, only the newest reaches `callback`.

    A slot of one, not a queue of one. `received` / `delivered` / `dropped` are
    the mux counters a run reads back.
    """

    def __init__(self, callback: Callable[[Any], None], name: str = "command") -> None:
        self._callback = callback
        self._name = name
        self._slot: Any = _EMPTY
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stopped = threading.Event()
        self._last_log = 0.0
        self.received = 0
        self.delivered = 0
        self.dropped = 0
        self._thread = threading.Thread(
            target=self._drain, name=f"latest-only-{name}", daemon=True)
        self._thread.start()

    def __call__(self, msg: Any) -> None:
        """Transport thread: O(1), never waits for the consumer."""
        with self._lock:
            superseded = self._slot is not _EMPTY
            self._slot = msg
            self.received += 1
            self.dropped += int(superseded)
            dropped = self.dropped
        self._wake.set()
        if superseded:
            self._log_drop(dropped)

    def rebind(self, callback: Callable[[Any], None]) -> None:
        """Point at a freshly built consumer callback: dimOS builds one per
        subscription, the drain thread outlives them."""
        with self._lock:
            self._callback = callback

    def stop(self, timeout: float = JOIN_TIMEOUT_S) -> None:
        """Stop the drain thread. It is a daemon, so a run never has to; the
        benches do."""
        self._stopped.set()
        self._wake.set()
        self._thread.join(timeout)

    def _log_drop(self, dropped: int) -> None:
        now = time.monotonic()
        if now - self._last_log < DROP_LOG_PERIOD_S:
            return
        self._last_log = now
        _log().warning(f"{self._name}: consumer behind, executing the newest command "
                       f"only ({dropped} stale dropped so far)")

    def _drain(self) -> None:
        while not self._stopped.is_set():
            self._wake.wait()
            self._wake.clear()
            with self._lock:
                msg, self._slot = self._slot, _EMPTY
                callback = self._callback
            if msg is _EMPTY:
                continue
            try:
                callback(msg)
            except Exception:
                _log().error(f"{self._name}: consumer raised", exc_info=True)
            self.delivered += 1
