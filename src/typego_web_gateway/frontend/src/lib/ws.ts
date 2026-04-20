import { useEffect, useRef, useState } from 'react';

export type StreamMessage =
  | { type: 'pose'; x: number; y: number; yaw: number; stamp: number }
  | { type: 'event'; [key: string]: unknown };

export interface UseStreamOptions {
  onMessage?: (msg: StreamMessage) => void;
}

/** Subscribe to /api/stream with auto-reconnect. */
export function useStream(opts: UseStreamOptions = {}) {
  const [connected, setConnected] = useState(false);
  const handlerRef = useRef(opts.onMessage);
  handlerRef.current = opts.onMessage;

  useEffect(() => {
    let ws: WebSocket | null = null;
    let closed = false;
    let reconnectTimer: number | null = null;

    const connect = () => {
      if (closed) return;
      const proto = location.protocol === 'https:' ? 'wss' : 'ws';
      ws = new WebSocket(`${proto}://${location.host}/api/stream`);
      ws.onopen = () => setConnected(true);
      ws.onclose = () => {
        setConnected(false);
        if (!closed) {
          reconnectTimer = window.setTimeout(connect, 1500);
        }
      };
      ws.onerror = () => ws?.close();
      ws.onmessage = (e) => {
        try {
          const msg = JSON.parse(e.data) as StreamMessage;
          handlerRef.current?.(msg);
        } catch {
          // ignore malformed frames
        }
      };
    };

    connect();

    return () => {
      closed = true;
      if (reconnectTimer !== null) window.clearTimeout(reconnectTimer);
      ws?.close();
    };
  }, []);

  return { connected };
}
