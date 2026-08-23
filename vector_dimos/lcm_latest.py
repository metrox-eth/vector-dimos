"""Bound the LCM queue of velocity-command topics to ONE pending message.

Why. dimOS subscribes every topic with `set_queue_capacity(10000)`
(dimos/protocol/pubsub/impl/lcmpubsub.py). A Twist carries no timestamp, so
when the coordinator process falls behind (measured 2026-08-22 with the point
cloud + Rerun bridge on: 117 % CPU, a 1 s command executed for 1.9 s, 2.08 m
driven for 0.9 m asked) the backlog of old twists is executed late - the
robot keeps moving on stale orders, and the zero that ends a move is the last
one in the line. For a velocity command only the latest value is meaningful:
with a capacity of 1, LCM drops new datagrams while one is still pending, so
the command that gets executed is never more than one publish period old.
The JointVelocityTask watchdog (0.2 s) still zeroes the base when the stream
stops, dropped final zero included.

How. dimOS's LCM wrapper keeps the raw `lcm.LCM` in `self.l` and calls
`self.l.subscribe(channel, handler)` then `.set_queue_capacity(10000)` on the
result. `install()` swaps `self.l` for a proxy whose subscriptions clamp the
capacity on command channels; everything else is delegated untouched.
Installed from vector_dimos.blueprints, i.e. in the process that runs the
coordinator.
"""
from __future__ import annotations

import logging
from typing import Any, Callable

COMMAND_CHANNEL_MARKERS = ("cmd_vel", "twist_command")
COMMAND_QUEUE_CAPACITY = 1

def _log() -> Any:
    try:
        from dimos.utils.logging_config import setup_logger
        return setup_logger()
    except Exception:
        return logging.getLogger(__name__)


def capacity_for(channel: str) -> int | None:
    """Queue capacity override for a channel, None = leave dimOS's default."""
    return COMMAND_QUEUE_CAPACITY if any(m in channel for m in COMMAND_CHANNEL_MARKERS) else None


class _SubscriptionProxy:
    def __init__(self, sub: Any, cap: int):
        self._sub, self._cap = sub, cap

    def set_queue_capacity(self, n: int) -> None:
        self._sub.set_queue_capacity(min(int(n), self._cap))

    def __getattr__(self, name: str) -> Any:
        return getattr(self._sub, name)


class _RawLCMProxy:
    """Wraps the raw lcm.LCM object: same API, clamped queues on command channels."""

    def __init__(self, raw: Any):
        self._raw = raw

    def subscribe(self, channel: str, handler: Callable[..., Any]) -> Any:
        sub = self._raw.subscribe(channel, handler)
        cap = capacity_for(channel)
        if cap is None:
            return sub
        _log().info(f"LCM queue capacity clamped to {cap} on {channel}")
        return _SubscriptionProxy(sub, cap)

    def unsubscribe(self, sub: Any) -> Any:
        return self._raw.unsubscribe(getattr(sub, "_sub", sub))

    def __getattr__(self, name: str) -> Any:
        return getattr(self._raw, name)


_installed = False


def install() -> bool:
    """Patch dimOS's LCM.subscribe so command channels get a 1-deep queue."""
    global _installed
    if _installed:
        return True
    try:
        from dimos.protocol.pubsub.impl.lcmpubsub import LCM
    except Exception:  # dimos not importable here (cold benches) - nothing to patch
        return False
    original = LCM.subscribe

    def subscribe(self: Any, topic: Any, callback: Any) -> Any:
        raw = getattr(self, "l", None)
        if raw is not None and not isinstance(raw, _RawLCMProxy):
            self.l = _RawLCMProxy(raw)
        return original(self, topic, callback)

    subscribe.__wrapped__ = original  # type: ignore[attr-defined]
    LCM.subscribe = subscribe  # type: ignore[method-assign]
    _installed = True
    return True
