"""In-process asyncio event bus.

The bus has two responsibilities for batch 1:

* Fan out the periodic *state snapshot* (ROS + host + launches + mission)
  to any number of WebSocket subscribers at a fixed rate.
* Carry structured *log lines* produced by ``launch_manager`` so that the
  Launch panel can stream them live without re-reading log files.

The implementation is deliberately dependency-free (``asyncio`` + stdlib)
so it can be unit-tested without starting a ROS master.
"""

from __future__ import annotations

import asyncio
import logging
import time
from dataclasses import dataclass, field
from typing import Any, Dict, Optional, Set

from . import config

_LOG = logging.getLogger(__name__)

LOG_QUEUE_MAX = 500


# ---------------------------------------------------------------------------
# Snapshot dataclass
# ---------------------------------------------------------------------------


@dataclass
class StateSnapshot:
    """Frozen view of the system published to every WS subscriber.

    The field layout mirrors the JSON schema documented in
    ``WORKBENCH.md`` §4.1. Each batch fills more sub-fields; batch 1 just
    produces the top-level skeleton so the frontend can wire up without
    guessing the shape.
    """

    ts: float = field(default_factory=time.time)
    ros: Dict[str, Any] = field(default_factory=lambda: {"master": False, "nodes": []})
    topics: Dict[str, Dict[str, float]] = field(default_factory=dict)
    pose: Optional[Dict[str, float]] = None
    host: Dict[str, Any] = field(default_factory=dict)
    launches: list = field(default_factory=list)
    mission: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Return a plain-dict representation suitable for ``json.dumps``."""
        return {
            "ts": self.ts,
            "ros": self.ros,
            "topics": self.topics,
            "pose": self.pose,
            "host": self.host,
            "launches": list(self.launches),
            "mission": self.mission,
        }


# ---------------------------------------------------------------------------
# Bus
# ---------------------------------------------------------------------------


class StateBus:
    """Fan-out hub for state snapshots and log lines.

    * ``publish_state`` stores the latest snapshot and schedules a
      broadcast to every subscriber's queue. Full queues drop the oldest
      item instead of blocking the producer.
    * ``subscribe_state`` yields :class:`asyncio.Queue` instances that the
      WebSocket handler can ``await`` on.
    * ``publish_log`` is the same pattern for stdout/stderr lines, keyed
      by launch ``name`` so the UI can filter.
    """

    def __init__(self, queue_max: int = config.STATE_QUEUE_MAX) -> None:
        self._queue_max = queue_max
        self._latest: Optional[StateSnapshot] = None
        self._state_subs: Set[asyncio.Queue] = set()
        self._log_subs: Set[asyncio.Queue] = set()
        self._closed = False
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    def bind_loop(self, loop: asyncio.AbstractEventLoop) -> None:
        """Remember the server event loop so other threads can publish safely."""
        self._loop = loop

    def _dispatch(self, subs: Set[asyncio.Queue], payload: Any) -> None:
        """Offer ``payload`` to every queue, hopping onto the loop if needed.

        ``asyncio.Queue`` is not thread-safe: launch log readers run in
        plain threads, so their lines are scheduled with
        ``call_soon_threadsafe`` which also wakes the awaiting WebSocket.
        """
        loop = self._loop
        if loop is not None and not loop.is_closed():
            try:
                running = asyncio.get_running_loop()
            except RuntimeError:
                running = None
            if running is not loop:
                loop.call_soon_threadsafe(self._offer_all, subs, payload)
                return
        self._offer_all(subs, payload)

    def _offer_all(self, subs: Set[asyncio.Queue], payload: Any) -> None:
        for queue in list(subs):
            self._offer(queue, payload)

    # -- state --------------------------------------------------------------

    @property
    def latest(self) -> Optional[StateSnapshot]:
        """Last snapshot published (or ``None`` if the producer never ran)."""
        return self._latest

    def publish_state(self, snapshot: StateSnapshot) -> None:
        """Store ``snapshot`` and broadcast it. Non-blocking."""
        if self._closed:
            return
        self._latest = snapshot
        self._dispatch(self._state_subs, snapshot.to_dict())

    def subscribe_state(self) -> asyncio.Queue:
        """Register a new state subscriber and return its queue."""
        queue: asyncio.Queue = asyncio.Queue(maxsize=self._queue_max)
        self._state_subs.add(queue)
        if self._latest is not None:
            self._offer(queue, self._latest.to_dict())
        return queue

    def unsubscribe_state(self, queue: asyncio.Queue) -> None:
        self._state_subs.discard(queue)

    # -- logs ---------------------------------------------------------------

    def publish_log(self, name: str, line: str) -> None:
        """Broadcast a single stdout/stderr line from a launched process."""
        if self._closed:
            return
        self._dispatch(self._log_subs, {"name": name, "line": line, "ts": time.time()})

    def subscribe_log(self) -> asyncio.Queue:
        # Logs burst at launch start-up; keep a deeper queue than state.
        queue: asyncio.Queue = asyncio.Queue(maxsize=LOG_QUEUE_MAX)
        self._log_subs.add(queue)
        return queue

    def unsubscribe_log(self, queue: asyncio.Queue) -> None:
        self._log_subs.discard(queue)

    # -- lifecycle ----------------------------------------------------------

    def close(self) -> None:
        """Release all subscribers so their awaiting tasks can exit."""
        self._closed = True
        for queue in self._state_subs | self._log_subs:
            # Push a sentinel so blocked ``await queue.get()`` calls wake
            # up; consumers interpret ``None`` as "bus closed".
            self._offer(queue, None)
        self._state_subs.clear()
        self._log_subs.clear()

    # -- internals ----------------------------------------------------------

    @staticmethod
    def _offer(queue: asyncio.Queue, payload: Any) -> None:
        """Try to put ``payload`` into ``queue``; on overflow drop oldest.

        ``asyncio.Queue`` has no built-in "ring" behaviour. We emulate it
        by draining one item on ``QueueFull`` and retrying; a subsequent
        ``put_nowait`` cannot fail because we just made room.
        """
        try:
            queue.put_nowait(payload)
            return
        except asyncio.QueueFull:
            try:
                queue.get_nowait()
            except asyncio.QueueEmpty:  # pragma: no cover — race, treat as full
                pass
            try:
                queue.put_nowait(payload)
            except asyncio.QueueFull:  # pragma: no cover — should not happen
                _LOG.warning("state_bus queue still full after drop; dropping payload")


# ---------------------------------------------------------------------------
# Module-level singleton
# ---------------------------------------------------------------------------

_bus: Optional[StateBus] = None


def get_bus() -> StateBus:
    """Return the process-wide :class:`StateBus`, creating it on first use."""
    global _bus
    if _bus is None:
        _bus = StateBus()
    return _bus


def reset_bus_for_tests() -> None:
    """Drop the module-level bus. Tests only."""
    global _bus
    if _bus is not None:
        _bus.close()
    _bus = None
