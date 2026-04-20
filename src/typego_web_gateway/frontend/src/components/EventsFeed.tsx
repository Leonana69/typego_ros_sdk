import { useEffect, useMemo, useRef, useState } from 'react';
import type { StreamMessage } from '../lib/ws';

interface EventRow {
  stamp: number;
  type: string;
  text: string;
  level: string;
}

interface Props {
  feed: StreamMessage[];
}

export default function EventsFeed({ feed }: Props) {
  const [rows, setRows] = useState<EventRow[]>([]);
  const boxRef = useRef<HTMLDivElement | null>(null);

  useEffect(() => {
    const additions: EventRow[] = [];
    for (const msg of feed) {
      if (msg.type !== 'event') continue;
      const m = msg as Record<string, unknown>;
      const type = (m['type'] as string) || 'event';
      const stamp = Number(m['stamp']) || Date.now() / 1000;
      const text = summarize(m);
      const level = severity(m);
      additions.push({ stamp, type, text, level });
    }
    if (additions.length) {
      setRows(prev => [...prev, ...additions].slice(-200));
    }
  }, [feed]);

  useEffect(() => {
    if (boxRef.current) {
      boxRef.current.scrollTop = boxRef.current.scrollHeight;
    }
  }, [rows]);

  const empty = useMemo(() => rows.length === 0, [rows]);

  return (
    <div className="panel">
      <h2>Events</h2>
      <div className="events" ref={boxRef}>
        {empty && <div className="muted">no events yet</div>}
        {rows.map((r, i) => (
          <div key={i} className={`ev level-${r.level}`}>
            <span className="t">
              {new Date(r.stamp * 1000).toLocaleTimeString()}
            </span>
            {r.text}
          </div>
        ))}
      </div>
    </div>
  );
}

function summarize(ev: Record<string, unknown>): string {
  const kind = ev['type'];
  if (kind === 'nav') {
    const bits = [`nav ${String(ev['status'] ?? '')}`];
    if (ev['x'] !== undefined) {
      bits.push(`@ (${Number(ev['x']).toFixed(2)}, ${Number(ev['y']).toFixed(2)})`);
    }
    return bits.join(' ');
  }
  if (kind === 'patrol') {
    return `patrol ${String(ev['status'] ?? '')} ${
      ev['index'] !== undefined ? `idx=${ev['index']}` : ''
    }`.trim();
  }
  if (kind === 'rosout') {
    return `[${ev['name']}] ${ev['msg']}`;
  }
  return JSON.stringify(ev);
}

function severity(ev: Record<string, unknown>): string {
  const kind = ev['type'];
  if (kind === 'nav') return 'nav';
  if (kind === 'patrol') return 'patrol';
  if (kind === 'rosout') {
    const lvl = Number(ev['level'] ?? 0);
    if (lvl >= 40) return 'error'; // FATAL / ERROR
    if (lvl >= 30) return 'warn';  // WARN
  }
  return 'info';
}
