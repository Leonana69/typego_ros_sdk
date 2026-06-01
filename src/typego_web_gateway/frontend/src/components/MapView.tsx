import { useEffect, useRef, useState } from 'react';
import type { MapMeta, Pose, Waypoint } from '../lib/api';
import { api, kUserIdBase } from '../lib/api';

export type MapMode = 'goal' | 'select' | 'add' | 'move';

interface Props {
  pose: Pose | null;
  mode?: MapMode;
  // The base interaction shown in the on-map Navigate/Label toggle.
  baseMode?: 'goal' | 'select';
  onBaseModeChange?: (m: 'goal' | 'select') => void;
  onPick?: (x: number, y: number) => void;
  onSelectWaypoint?: (wp: Waypoint) => void;
  onAddWaypoint?: (x: number, y: number) => void;
  onMoveWaypoint?: (x: number, y: number) => void;
  onSelectPlace?: (placeId: string) => void;
}

// Pixel radius for the select-mode marker hit-test.
const HIT_PX = 12;

/**
 * Canvas map viewer.
 *
 * The OccupancyGrid PNG is fit to the viewport with a uniform scale. Map
 * world coordinates are converted via:
 *
 *   pixel_x = (world_x - origin_x) / resolution
 *   pixel_y_from_bottom = (world_y - origin_y) / resolution
 *   pixel_y_from_top    = image_height_px - pixel_y_from_bottom
 *
 * The image_render module flips the grid vertically, so the PNG's top
 * row corresponds to the highest Y — that inversion is folded into the
 * transform below.
 */
export default function MapView({
  pose,
  mode = 'goal',
  baseMode = 'goal',
  onBaseModeChange,
  onPick,
  onSelectWaypoint,
  onAddWaypoint,
  onMoveWaypoint,
  onSelectPlace,
}: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [error, setError] = useState<string | null>(null);
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [parked, setParked] = useState<Waypoint[]>([]);
  // Keep the latest waypoints accessible to the click handler without
  // making it a draw dependency.
  const wpRef = useRef<Waypoint[]>([]);
  const parkedRef = useRef<Waypoint[]>([]);
  // Id of the waypoint under the cursor (read each frame by the draw loop so
  // hover-glow updates without re-rendering).
  const hoverIdRef = useRef<number | null>(null);

  // Load the map PNG + metadata on mount, re-poll occasionally so SLAM
  // updates show up without manual refresh.
  useEffect(() => {
    let alive = true;
    const load = async () => {
      try {
        const m = await api.mapMeta();
        if (!alive) return;
        setMeta(m);
        const img = new Image();
        img.onload = () => {
          if (!alive) return;
          imgRef.current = img;
          setError(null);
        };
        img.onerror = () => setError('failed to load map PNG');
        img.src = api.mapUrl();
      } catch (e) {
        setError(String(e));
      }
    };
    load();
    const t = setInterval(load, 10_000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  // Poll waypoints (active + parked) so markers overlay the map.
  useEffect(() => {
    let alive = true;
    const load = async () => {
      try {
        const r = await api.waypoints();
        if (!alive) return;
        setWaypoints(r.waypoints);
        setParked(r.parked);
        wpRef.current = r.waypoints;
        parkedRef.current = r.parked;
      } catch {
        /* ignore — transient */
      }
    };
    load();
    const t = setInterval(load, 4_000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  // Redraw on any pose, marker, or resize change.
  useEffect(() => {
    let raf = 0;
    const draw = () => {
      const canvas = canvasRef.current;
      const img = imgRef.current;
      if (!canvas) return;
      const parent = canvas.parentElement!;
      const dpr = window.devicePixelRatio || 1;
      const w = parent.clientWidth;
      const h = parent.clientHeight;
      if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
        canvas.width = w * dpr;
        canvas.height = h * dpr;
      }
      const ctx = canvas.getContext('2d')!;
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
      ctx.clearRect(0, 0, w, h);
      ctx.fillStyle = '#0d1013';
      ctx.fillRect(0, 0, w, h);

      if (!img || !meta) return;

      const iw = img.width;
      const ih = img.height;
      const scale = Math.min(w / iw, h / ih);
      const dw = iw * scale;
      const dh = ih * scale;
      const dx = (w - dw) / 2;
      const dy = (h - dh) / 2;
      ctx.imageSmoothingEnabled = false;
      ctx.drawImage(img, dx, dy, dw, dh);

      // World -> canvas pixel for a marker/pose (PNG already flipped).
      const toCanvas = (wx: number, wy: number): [number, number] => {
        const px = (wx - meta.origin_x) / meta.resolution;
        const pyBottom = (wy - meta.origin_y) / meta.resolution;
        const pyTop = ih - pyBottom;
        return [dx + px * scale, dy + pyTop * scale];
      };

      // Draw waypoint markers (parked first, dimmed, then active on top).
      const hover = hoverIdRef.current;
      for (const wp of parked)
        drawMarker(ctx, toCanvas(wp.x, wp.y), wp, true, wp.id === hover);
      for (const wp of waypoints)
        drawMarker(ctx, toCanvas(wp.x, wp.y), wp, false, wp.id === hover);

      if (pose) {
        const [cx, cy] = toCanvas(pose.x, pose.y);
        ctx.save();
        ctx.translate(cx, cy);
        // Canvas Y is inverted w.r.t. world Y, so flip the yaw sign.
        ctx.rotate(-pose.yaw);
        ctx.fillStyle = '#2a6cf0';
        ctx.strokeStyle = '#ffffff';
        ctx.lineWidth = 1.5;
        ctx.beginPath();
        ctx.moveTo(10, 0);
        ctx.lineTo(-7, 6);
        ctx.lineTo(-7, -6);
        ctx.closePath();
        ctx.fill();
        ctx.stroke();
        ctx.restore();
      }
    };

    const loop = () => {
      draw();
      raf = requestAnimationFrame(loop);
    };
    loop();
    const ro = new ResizeObserver(draw);
    if (canvasRef.current?.parentElement) {
      ro.observe(canvasRef.current.parentElement);
    }
    return () => {
      cancelAnimationFrame(raf);
      ro.disconnect();
    };
  }, [meta, pose, waypoints, parked]);

  // Hover hit-test: mark the nearest waypoint within HIT_PX for the glow, and
  // switch the cursor to a pointer over it.
  const handleMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    const img = imgRef.current;
    if (!canvas || !img || !meta) return;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const iw = img.width;
    const ih = img.height;
    const scale = Math.min(rect.width / iw, rect.height / ih);
    const dx = (rect.width - iw * scale) / 2;
    const dy = (rect.height - ih * scale) / 2;
    let best: number | null = null;
    let bestD = HIT_PX * HIT_PX;
    for (const wp of [...wpRef.current, ...parkedRef.current]) {
      const cx = dx + ((wp.x - meta.origin_x) / meta.resolution) * scale;
      const cy = dy + (ih - (wp.y - meta.origin_y) / meta.resolution) * scale;
      const d = (cx - mx) ** 2 + (cy - my) ** 2;
      if (d <= bestD) {
        bestD = d;
        best = wp.id;
      }
    }
    hoverIdRef.current = best;
    canvas.style.cursor = best != null ? 'pointer' : 'crosshair';
  };

  const handleLeave = () => {
    hoverIdRef.current = null;
  };

  const handleClick = async (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!meta || !imgRef.current) return;
    const canvas = canvasRef.current!;
    const rect = canvas.getBoundingClientRect();
    const mx = e.clientX - rect.left;
    const my = e.clientY - rect.top;
    const iw = imgRef.current.width;
    const ih = imgRef.current.height;
    const w = rect.width;
    const h = rect.height;
    const scale = Math.min(w / iw, h / ih);
    const dw = iw * scale;
    const dh = ih * scale;
    const dx = (w - dw) / 2;
    const dy = (h - dh) / 2;
    const px = (mx - dx) / scale;
    const pyFromTop = (my - dy) / scale;
    if (px < 0 || px > iw || pyFromTop < 0 || pyFromTop > ih) return;
    const pyFromBottom = ih - pyFromTop;
    const worldX = meta.origin_x + px * meta.resolution;
    const worldY = meta.origin_y + pyFromBottom * meta.resolution;

    switch (mode) {
      case 'goal':
        onPick?.(worldX, worldY);
        break;
      case 'add':
        onAddWaypoint?.(worldX, worldY);
        break;
      case 'move':
        onMoveWaypoint?.(worldX, worldY);
        break;
      case 'select': {
        // World->canvas matching the draw transform, for the hit-test.
        const toCanvas = (wx: number, wy: number): [number, number] => {
          const cpx = (wx - meta.origin_x) / meta.resolution;
          const cpyBottom = (wy - meta.origin_y) / meta.resolution;
          const cpyTop = ih - cpyBottom;
          return [dx + cpx * scale, dy + cpyTop * scale];
        };
        let best: Waypoint | null = null;
        let bestD = HIT_PX * HIT_PX;
        for (const wp of [...wpRef.current, ...parkedRef.current]) {
          const [cx, cy] = toCanvas(wp.x, wp.y);
          const ddx = cx - mx;
          const ddy = cy - my;
          const d = ddx * ddx + ddy * ddy;
          if (d <= bestD) {
            bestD = d;
            best = wp;
          }
        }
        if (best) {
          onSelectWaypoint?.(best);
          break;
        }
        try {
          const r = await api.resolvePlace(worldX, worldY);
          if (r.place_id) onSelectPlace?.(r.place_id);
        } catch {
          /* ignore — void click is a normal no-op */
        }
        break;
      }
    }
  };

  return (
    <div className="map-pane">
      <div className="map-head">
        <h2>Map</h2>
        <div className="seg" role="group" aria-label="map interaction">
          <button
            className={baseMode === 'goal' ? 'active' : ''}
            onClick={() => onBaseModeChange?.('goal')}
            title="Click the map to set a navigation goal"
          >
            Navigate
          </button>
          <button
            className={baseMode === 'select' ? 'active' : ''}
            onClick={() => onBaseModeChange?.('select')}
            title="Click a waypoint or region to edit its label"
          >
            Label
          </button>
        </div>
      </div>
      <div className="map-frame">
        <canvas
          ref={canvasRef}
          onClick={handleClick}
          onMouseMove={handleMove}
          onMouseLeave={handleLeave}
        />
        {error && (
          <div
            style={{
              position: 'absolute',
              top: 10,
              left: 10,
              color: '#ff8888',
              fontSize: 12,
            }}
          >
            {error}
          </div>
        )}
        {!meta && !error && (
          <div
            style={{ position: 'absolute', top: 10, left: 10 }}
            className="muted"
          >
            waiting for /map…
          </div>
        )}
      </div>
      <div className="muted map-topic">
        {meta
          ? `${meta.topic ?? '/map'}${meta.frame_id ? ` · ${meta.frame_id}` : ''}`
          : 'map topic unknown'}
      </div>
    </div>
  );
}

// A diamond glyph for operator pins, a dot for auto/frontier anchors.
// Parked pins render dimmed/outlined only. The hovered marker grows and glows.
function drawMarker(
  ctx: CanvasRenderingContext2D,
  [cx, cy]: [number, number],
  wp: Waypoint,
  isParked: boolean,
  isHover: boolean,
) {
  const isUser = wp.id >= kUserIdBase;
  const isFrontier = wp.waypoint_role === 'frontier'
    || wp.source === 'frontier';
  const fill = isUser ? '#f2c15d' : isFrontier ? '#7aa2ff' : '#7ed491';
  const grow = isHover ? 3 : 0;
  ctx.save();
  ctx.globalAlpha = isParked ? (isHover ? 0.75 : 0.45) : 1.0;
  ctx.lineWidth = isHover ? 2.5 : 1.5;
  ctx.strokeStyle = isHover ? '#ffffff' : '#0d1013';
  if (isHover) {
    ctx.shadowColor = fill;
    ctx.shadowBlur = 16;
  }
  if (isUser) {
    // Diamond glyph for operator-created pins.
    const r = 8 + grow;
    ctx.beginPath();
    ctx.moveTo(cx, cy - r);
    ctx.lineTo(cx + r, cy);
    ctx.lineTo(cx, cy + r);
    ctx.lineTo(cx - r, cy);
    ctx.closePath();
  } else {
    ctx.beginPath();
    ctx.arc(cx, cy, 6 + grow, 0, Math.PI * 2);
  }
  if (isParked) {
    ctx.strokeStyle = isHover ? '#ffffff' : fill;
    ctx.stroke();
  } else {
    ctx.fillStyle = fill;
    ctx.fill();
    ctx.stroke();
  }
  ctx.restore();
}
