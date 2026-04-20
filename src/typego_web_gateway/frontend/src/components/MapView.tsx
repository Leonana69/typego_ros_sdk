import { useEffect, useRef, useState } from 'react';
import type { MapMeta, Pose } from '../lib/api';
import { api } from '../lib/api';

interface Props {
  pose: Pose | null;
  onPick?: (x: number, y: number) => void;
}

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
export default function MapView({ pose, onPick }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const imgRef = useRef<HTMLImageElement | null>(null);
  const [meta, setMeta] = useState<MapMeta | null>(null);
  const [error, setError] = useState<string | null>(null);

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

  // Redraw on any pose or resize change.
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

      if (pose) {
        // World -> image pixel (PNG is already flipped vertically)
        const px = (pose.x - meta.origin_x) / meta.resolution;
        const pyBottom = (pose.y - meta.origin_y) / meta.resolution;
        const pyTop = ih - pyBottom;
        const cx = dx + px * scale;
        const cy = dy + pyTop * scale;

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
  }, [meta, pose]);

  const handleClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!onPick || !meta || !imgRef.current) return;
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
    onPick(worldX, worldY);
  };

  return (
    <div className="map-pane">
      <canvas ref={canvasRef} onClick={handleClick} />
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
        <div style={{ position: 'absolute', top: 10, left: 10 }} className="muted">
          waiting for /map…
        </div>
      )}
    </div>
  );
}
