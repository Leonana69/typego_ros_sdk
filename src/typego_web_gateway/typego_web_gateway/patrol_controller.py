"""Patrol loop ported from scripts/patrol.py into an asyncio task.

Loads waypoints from ``src/typego_sdk/resource/Map-<name>/waypoints.json``
(same path convention as patrol.py) and drives NavigateToPose goals in a
loop. Exposes start / stop / status for the REST layer.
"""
from __future__ import annotations

import asyncio
import json
import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

# action_msgs.msg.GoalStatus.STATUS_SUCCEEDED == 4 (the value patrol.py uses)
_STATUS_SUCCEEDED = 4


@dataclass
class PatrolState:
    running: bool = False
    map_name: str = ''
    indices: List[int] = field(default_factory=list)
    current_index: Optional[int] = None
    lap: int = 0
    last_error: str = ''


def _resolve_waypoints_path(map_name: str) -> str:
    """Resolve Map-<name>/waypoints.json.

    Search order:
      1. $TYPEGO_MAPS_DIR (if set) / Map-<name>/waypoints.json
      2. <pkg_share>/../typego_sdk/resource/Map-<name>/waypoints.json
         (typical colcon install layout)
      3. <repo_root>/src/typego_sdk/resource/Map-<name>/waypoints.json
         (matches scripts/patrol.py:26-31)
    """
    candidates = []
    override = os.environ.get('TYPEGO_MAPS_DIR')
    if override:
        candidates.append(os.path.join(override, f'Map-{map_name}', 'waypoints.json'))

    # share/typego_sdk/resource (installed) — shares the same ament_index root
    try:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('typego_sdk')
        candidates.append(os.path.join(share, 'resource', f'Map-{map_name}', 'waypoints.json'))
    except Exception:
        pass

    repo_root = os.path.dirname(os.path.dirname(os.path.dirname(
        os.path.dirname(os.path.abspath(__file__)))))
    candidates.append(os.path.join(
        repo_root, 'src', 'typego_sdk', 'resource',
        f'Map-{map_name}', 'waypoints.json',
    ))

    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f'waypoints.json for map "{map_name}" not found; searched: {candidates}'
    )


def load_waypoints(map_name: str) -> Dict[int, Tuple[float, float]]:
    """Load ``Map-<name>/waypoints.json`` as {index: (x, y)}."""
    path = _resolve_waypoints_path(map_name)
    out: Dict[int, Tuple[float, float]] = {}
    with open(path) as f:
        doc = json.load(f)
    items = doc.get('waypoints', doc) if isinstance(doc, dict) else doc
    for item in items:
        out[int(item['id'])] = (float(item['x']), float(item['y']))
    if not out:
        raise ValueError(f'waypoints file {path} produced no entries')
    return out


class PatrolController:
    def __init__(self, bridge):
        self._bridge = bridge
        self._state = PatrolState()
        self._task: Optional[asyncio.Task] = None
        self._lock = asyncio.Lock()

    @property
    def state(self) -> PatrolState:
        return self._state

    async def start(self, map_name: str, indices: List[int]) -> Tuple[bool, str]:
        async with self._lock:
            if self._state.running:
                return False, 'patrol already running'
            try:
                waypoints = load_waypoints(map_name)
            except (FileNotFoundError, ValueError) as exc:
                return False, str(exc)
            missing = [i for i in indices if i not in waypoints]
            if missing:
                return False, f'unknown waypoint indices: {missing}'
            if not indices:
                return False, 'indices list is empty'
            self._state = PatrolState(
                running=True,
                map_name=map_name,
                indices=list(indices),
                current_index=None,
                lap=0,
            )
            self._task = asyncio.create_task(self._run(waypoints))
            self._bridge.record_event({
                'type': 'patrol', 'status': 'started',
                'map_name': map_name, 'indices': list(indices),
                'stamp': time.time(),
            })
            return True, 'started'

    async def stop(self) -> Tuple[bool, str]:
        async with self._lock:
            if not self._state.running:
                return False, 'patrol not running'
            self._state.running = False
            task = self._task
        # cancel current goal so the current leg stops promptly
        await asyncio.to_thread(self._bridge.cancel_goal)
        if task is not None:
            task.cancel()
            try:
                await task
            except (asyncio.CancelledError, Exception):
                pass
        self._bridge.record_event({'type': 'patrol', 'status': 'stopped'})
        return True, 'stopped'

    async def _run(self, waypoints: Dict[int, Tuple[float, float]]) -> None:
        try:
            while self._state.running:
                self._state.lap += 1
                for idx in list(self._state.indices):
                    if not self._state.running:
                        break
                    x, y = waypoints[idx]
                    self._state.current_index = idx
                    self._bridge.record_event({
                        'type': 'patrol', 'status': 'waypoint',
                        'lap': self._state.lap, 'index': idx,
                        'x': x, 'y': y,
                    })
                    ok, msg = await asyncio.to_thread(
                        self._bridge.send_goal, x, y, 0.0,
                    )
                    if not ok:
                        self._state.last_error = msg
                        self._bridge.record_event({
                            'type': 'patrol', 'status': 'send_failed',
                            'index': idx, 'msg': msg,
                        })
                        await asyncio.sleep(1.0)
                        continue
                    status = await asyncio.to_thread(
                        self._bridge.wait_goal_result, 600.0,
                    )
                    if status != _STATUS_SUCCEEDED and self._state.running:
                        self._state.last_error = f'goal status {status}'
                        # brief backoff before the next index so we don't spin
                        await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            raise
        finally:
            self._state.running = False
            self._state.current_index = None
