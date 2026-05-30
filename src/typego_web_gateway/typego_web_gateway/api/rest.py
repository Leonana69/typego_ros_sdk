"""FastAPI application factory.

All REST + WebSocket routes live here. Handlers translate HTTP into
RosBridge / PatrolController / BagRotator calls. The app is mounted with
the built frontend (if present) at ``/``.
"""
from __future__ import annotations

import asyncio
import io
import os
import time
import zipfile
from pathlib import Path
from typing import List, Optional

from fastapi import FastAPI, HTTPException, Query, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse, Response, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from ..bag_rotator import BagRotator
from ..map_render import render_occupancy_grid
from ..patrol_controller import PatrolController
from ..ros_bridge import RosBridge
from .broadcast import Broadcaster, event_publisher, pose_publisher


class GoalRequest(BaseModel):
    x: float
    y: float
    yaw_deg: float = 0.0


class PatrolStartRequest(BaseModel):
    map_name: str = Field(..., min_length=1)
    indices: List[int]


class SpeedRequest(BaseModel):
    max_linear: float = -1.0
    max_autonomy: float = -1.0
    max_reverse: float = -1.0
    max_angular: float = -1.0


def build_app(
    bridge: RosBridge,
    patrol: PatrolController,
    bag_rotator: Optional[BagRotator],
    frontend_dir: Optional[str] = None,
) -> FastAPI:
    app = FastAPI(title='TypeGo Web Gateway', version='0.1.0')
    broadcaster = Broadcaster()
    state = {'broadcaster': broadcaster}

    @app.on_event('startup')
    async def _startup() -> None:
        state['pose_task'] = asyncio.create_task(
            pose_publisher(bridge, broadcaster))
        state['event_task'] = asyncio.create_task(
            event_publisher(bridge, broadcaster))

    @app.on_event('shutdown')
    async def _shutdown() -> None:
        for key in ('pose_task', 'event_task'):
            task = state.get(key)
            if task:
                task.cancel()

    # ── health / status ─────────────────────────────────────────────────
    @app.get('/api/status')
    async def status() -> JSONResponse:
        s = bridge.status()
        pose = bridge.get_pose()
        s['pose'] = (
            {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw, 'stamp': pose.stamp}
            if pose else None
        )
        s['patrol'] = {
            'running': patrol.state.running,
            'map_name': patrol.state.map_name,
            'indices': patrol.state.indices,
            'current_index': patrol.state.current_index,
            'lap': patrol.state.lap,
            'last_error': patrol.state.last_error,
        }
        s['bag'] = (
            {'dir': str(bag_rotator.bag_dir),
             'count': len(bag_rotator.list_bags())}
            if bag_rotator else None
        )
        return JSONResponse(s)

    # ── map ──────────────────────────────────────────────────────────────
    @app.get('/api/map')
    async def map_endpoint() -> Response:
        snap = bridge.get_map()
        if snap is None:
            raise HTTPException(status_code=503, detail='map not received yet')
        png = await asyncio.to_thread(
            render_occupancy_grid, snap.data, snap.width, snap.height,
        )
        headers = {
            'X-Map-Width': str(snap.width),
            'X-Map-Height': str(snap.height),
            'X-Map-Resolution': f'{snap.resolution:.6f}',
            'X-Map-Origin-X': f'{snap.origin_x:.6f}',
            'X-Map-Origin-Y': f'{snap.origin_y:.6f}',
            'X-Map-Origin-Yaw': f'{snap.origin_yaw:.6f}',
            'X-Map-Frame': snap.frame_id,
            'X-Map-Stamp': f'{snap.stamp:.3f}',
            'Cache-Control': 'no-store',
        }
        return Response(content=png, media_type='image/png', headers=headers)

    @app.get('/api/map/meta')
    async def map_meta() -> JSONResponse:
        snap = bridge.get_map()
        if snap is None:
            raise HTTPException(status_code=503, detail='map not received yet')
        return JSONResponse({
            'width': snap.width,
            'height': snap.height,
            'resolution': snap.resolution,
            'origin_x': snap.origin_x,
            'origin_y': snap.origin_y,
            'origin_yaw': snap.origin_yaw,
            'frame_id': snap.frame_id,
            'stamp': snap.stamp,
        })

    # ── waypoints ────────────────────────────────────────────────────────
    @app.get('/api/waypoints')
    async def waypoints() -> JSONResponse:
        return JSONResponse({'waypoints': bridge.get_waypoints()})

    # ── places (place graph) ─────────────────────────────────────────────
    @app.get('/api/places')
    async def places() -> JSONResponse:
        graph = bridge.get_place_graph()
        all_places = graph.get('places', [])
        connector_kinds = {'portal', 'open_transition', 'junction'}
        rooms = [p for p in all_places if p['kind'] not in connector_kinds]
        portals = [p for p in all_places if p['kind'] in connector_kinds]

        # Room adjacency is derived from the connector graph.
        adj: dict = {}
        for c in portals:
            ends = c.get('adjacent_place_ids', [])
            for a in ends:
                adj.setdefault(a, set()).update(e for e in ends if e != a)

        def label(p: dict) -> str:
            if p.get('label_locked') and p.get('operator_label'):
                return p['operator_label']
            return p.get('instance_label') or p.get('semantic_label') \
                or p['place_id']

        # id -> waypoint_role, so the planner can pick a waypoint by role
        # within the region it chose.
        wp_role = {int(w['id']): str(w.get('waypoint_role', ''))
                   for w in bridge.get_waypoints()}

        def wp_str(ids: list) -> str:
            parts = []
            for i in ids:
                r = wp_role.get(int(i), '')
                parts.append(f"{i}({r})" if r else str(i))
            return ' '.join(parts) or '-'

        # Region-first block: the planner matches a task to a region via
        # type / affordances / objects / summary, then picks a waypoint by role.
        lines = [f"PLACES (map {graph.get('map_version', '')}):", '']
        for p in rooms:
            wps = p.get('member_waypoint_ids', [])
            rtype = p.get('semantic_label') or '?'
            conf = float(p.get('confidence', 0.0))
            neighbors = ', '.join(sorted(adj.get(p['place_id'], []))) or '-'
            state = ' provisional' if p.get('provisional') else ''
            lines.append(
                f"[{p['place_id']}] {label(p)} ({rtype}) conf {conf:.2f}"
                f"{state}  adj: {neighbors}")
            if p.get('affordances'):
                lines.append(f"  for: {', '.join(p['affordances'])}")
            if p.get('objects'):
                lines.append(f"  objects: {', '.join(p['objects'])}")
            if p.get('summary'):
                lines.append(f"  \"{p['summary']}\"")
            lines.append(f"  waypoints: {wp_str(wps)}")
        lines.append('')
        lines.append('Portals:')
        for c in portals:
            wps = c.get('member_waypoint_ids', [])
            wp = wps[0] if wps else '-'
            ends = ' <-> '.join(c.get('adjacent_place_ids', []))
            width = c.get('width_m')
            wtxt = f", width {width:.2f} m" if width else ''
            lines.append(
                f"- {c['place_id']} (waypoint {wp}): {ends}{wtxt}")

        return JSONResponse({
            'map_version': graph.get('map_version', ''),
            'places': rooms,
            'portals': portals,
            'summary': '\n'.join(lines),
        })

    # ── goal ─────────────────────────────────────────────────────────────
    @app.post('/api/goal')
    async def post_goal(req: GoalRequest) -> JSONResponse:
        ok, msg = await asyncio.to_thread(
            bridge.send_goal, req.x, req.y, req.yaw_deg,
        )
        status = 200 if ok else 409
        return JSONResponse({'accepted': ok, 'message': msg}, status_code=status)

    @app.post('/api/goal/cancel')
    async def cancel_goal() -> JSONResponse:
        ok, msg = await asyncio.to_thread(bridge.cancel_goal)
        status = 200 if ok else 409
        return JSONResponse({'canceled': ok, 'message': msg}, status_code=status)

    # ── patrol ───────────────────────────────────────────────────────────
    @app.post('/api/patrol/start')
    async def patrol_start(req: PatrolStartRequest) -> JSONResponse:
        ok, msg = await patrol.start(req.map_name, req.indices)
        return JSONResponse(
            {'started': ok, 'message': msg},
            status_code=200 if ok else 409,
        )

    @app.post('/api/patrol/stop')
    async def patrol_stop() -> JSONResponse:
        ok, msg = await patrol.stop()
        return JSONResponse(
            {'stopped': ok, 'message': msg},
            status_code=200 if ok else 409,
        )

    @app.get('/api/patrol/status')
    async def patrol_status() -> JSONResponse:
        st = patrol.state
        return JSONResponse({
            'running': st.running,
            'map_name': st.map_name,
            'indices': st.indices,
            'current_index': st.current_index,
            'lap': st.lap,
            'last_error': st.last_error,
        })

    # ── speed ────────────────────────────────────────────────────────────
    @app.post('/api/speed')
    async def set_speed(req: SpeedRequest) -> JSONResponse:
        ok, msg = await asyncio.to_thread(
            bridge.set_speed,
            req.max_linear, req.max_autonomy,
            req.max_reverse, req.max_angular,
        )
        return JSONResponse(
            {'success': ok, 'message': msg},
            status_code=200 if ok else 409,
        )

    # ── config (read-only mirror of robot.yaml via typego_config) ────────
    @app.get('/api/config')
    async def config() -> JSONResponse:
        cfg = await asyncio.to_thread(bridge.get_robot_config)
        if cfg is None:
            return JSONResponse(
                {'available': False,
                 'message': 'typego_config service not reachable'},
                status_code=503,
            )
        return JSONResponse({'available': True, 'config': cfg})

    # ── events ───────────────────────────────────────────────────────────
    @app.get('/api/events')
    async def events(limit: int = Query(100, ge=1, le=500)) -> JSONResponse:
        return JSONResponse({'events': bridge.get_events(limit=limit)})

    # ── bag download ─────────────────────────────────────────────────────
    @app.get('/api/bag/latest')
    async def bag_latest(
        minutes: int = Query(60, ge=1, le=24 * 60),
    ) -> StreamingResponse:
        if bag_rotator is None:
            raise HTTPException(status_code=503, detail='bag rotator disabled')
        bags = bag_rotator.bags_covering(minutes)
        if not bags:
            raise HTTPException(status_code=404, detail='no bags available')
        return StreamingResponse(
            _stream_zip(bags),
            media_type='application/zip',
            headers={
                'Content-Disposition':
                    f'attachment; filename="typego_last_{minutes}min.zip"',
            },
        )

    # ── WebSocket stream ─────────────────────────────────────────────────
    @app.websocket('/api/stream')
    async def ws_stream(ws: WebSocket) -> None:
        await ws.accept()
        queue = await broadcaster.subscribe()
        # bootstrap with a snapshot
        pose = bridge.get_pose()
        if pose is not None:
            await ws.send_json({
                'type': 'pose', 'x': pose.x, 'y': pose.y,
                'yaw': pose.yaw, 'stamp': pose.stamp,
            })
        try:
            while True:
                msg = await queue.get()
                await ws.send_json(msg)
        except WebSocketDisconnect:
            pass
        finally:
            await broadcaster.unsubscribe(queue)

    # ── static frontend ──────────────────────────────────────────────────
    if frontend_dir and os.path.isdir(frontend_dir):
        app.mount('/', StaticFiles(directory=frontend_dir, html=True),
                  name='frontend')
    else:
        @app.get('/')
        async def placeholder() -> JSONResponse:
            return JSONResponse({
                'service': 'typego_web_gateway',
                'status': 'ok',
                'hint': 'frontend bundle not built; API is under /api/*',
            })

    return app


def _stream_zip(paths):
    """Generator yielding a zip archive of the given bag directories."""
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, mode='w', compression=zipfile.ZIP_STORED,
                         allowZip64=True) as zf:
        for bag in paths:
            bag = Path(bag)
            for fp in bag.rglob('*'):
                if fp.is_file():
                    arcname = f'{bag.name}/{fp.relative_to(bag).as_posix()}'
                    zf.write(fp, arcname=arcname)
    buf.seek(0)
    chunk = 64 * 1024
    while True:
        data = buf.read(chunk)
        if not data:
            break
        yield data
