"""Tests for :mod:`infra.state_bus`."""

from __future__ import annotations

import asyncio

import pytest

from infra import state_bus
from infra.state_bus import StateBus, StateSnapshot


@pytest.fixture(autouse=True)
def _reset():
    state_bus.reset_bus_for_tests()
    yield
    state_bus.reset_bus_for_tests()


def _run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def test_subscribe_receives_snapshot():
    async def scenario():
        bus = StateBus(queue_max=4)
        q = bus.subscribe_state()
        bus.publish_state(StateSnapshot(host={"cpu": 10}))
        payload = await asyncio.wait_for(q.get(), timeout=0.1)
        assert payload["host"] == {"cpu": 10}
    _run(scenario())


def test_latest_replayed_on_subscribe():
    async def scenario():
        bus = StateBus()
        bus.publish_state(StateSnapshot(host={"mem": 42}))
        q = bus.subscribe_state()
        payload = await asyncio.wait_for(q.get(), timeout=0.1)
        assert payload["host"] == {"mem": 42}
    _run(scenario())


def test_slow_subscriber_drops_oldest():
    async def scenario():
        bus = StateBus(queue_max=2)
        q = bus.subscribe_state()
        # Fill + overflow; oldest should be dropped so newest stays.
        for i in range(5):
            bus.publish_state(StateSnapshot(host={"i": i}))
        received = []
        while not q.empty():
            received.append(q.get_nowait())
        assert len(received) == 2
        # The most recent "i" must survive.
        assert received[-1]["host"]["i"] == 4
    _run(scenario())


def test_close_wakes_subscribers():
    async def scenario():
        bus = StateBus()
        q = bus.subscribe_state()
        bus.close()
        value = await asyncio.wait_for(q.get(), timeout=0.1)
        assert value is None
    _run(scenario())


def test_log_fanout():
    async def scenario():
        bus = StateBus()
        q1 = bus.subscribe_log()
        q2 = bus.subscribe_log()
        bus.publish_log("hardware", "hello")
        a = await asyncio.wait_for(q1.get(), timeout=0.1)
        b = await asyncio.wait_for(q2.get(), timeout=0.1)
        assert a["name"] == "hardware" == b["name"]
        assert a["line"] == "hello" == b["line"]
    _run(scenario())
