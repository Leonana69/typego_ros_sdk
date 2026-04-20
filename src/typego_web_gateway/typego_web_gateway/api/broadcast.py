"""Async fan-out of pose + events to WebSocket clients."""
from __future__ import annotations

import asyncio
import time
from typing import Dict, List, Optional, Set


class Broadcaster:
    def __init__(self) -> None:
        self._subscribers: Set[asyncio.Queue] = set()
        self._lock = asyncio.Lock()
        self._last_event_stamp: float = 0.0

    async def subscribe(self) -> asyncio.Queue:
        q: asyncio.Queue = asyncio.Queue(maxsize=256)
        async with self._lock:
            self._subscribers.add(q)
        return q

    async def unsubscribe(self, q: asyncio.Queue) -> None:
        async with self._lock:
            self._subscribers.discard(q)

    async def publish(self, message: Dict) -> None:
        async with self._lock:
            subs = list(self._subscribers)
        for q in subs:
            try:
                q.put_nowait(message)
            except asyncio.QueueFull:
                # slow consumer: drop the oldest, push the newest
                try:
                    q.get_nowait()
                except asyncio.QueueEmpty:
                    pass
                try:
                    q.put_nowait(message)
                except asyncio.QueueFull:
                    pass


async def pose_publisher(bridge, broadcaster: Broadcaster, hz: float = 10.0) -> None:
    period = 1.0 / max(hz, 0.1)
    while True:
        pose = bridge.get_pose()
        if pose is not None:
            await broadcaster.publish({
                'type': 'pose',
                'x': pose.x, 'y': pose.y, 'yaw': pose.yaw,
                'stamp': pose.stamp,
            })
        await asyncio.sleep(period)


async def event_publisher(bridge, broadcaster: Broadcaster,
                          hz: float = 5.0) -> None:
    """Drain new events from the ring buffer and fan them out."""
    period = 1.0 / max(hz, 0.1)
    last_stamp: float = 0.0
    while True:
        events = bridge.get_events(limit=200)
        for ev in events:
            stamp = float(ev.get('stamp', 0.0))
            if stamp <= last_stamp:
                continue
            last_stamp = stamp
            await broadcaster.publish({'type': 'event', **ev})
        await asyncio.sleep(period)
