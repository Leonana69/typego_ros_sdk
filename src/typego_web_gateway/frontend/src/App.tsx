import { useCallback, useEffect, useState } from 'react';
import BagDownload from './components/BagDownload';
import EventsFeed from './components/EventsFeed';
import GoalForm from './components/GoalForm';
import MapView from './components/MapView';
import PatrolPanel from './components/PatrolPanel';
import SpeedControl from './components/SpeedControl';
import type { GatewayStatus, Pose } from './lib/api';
import { api } from './lib/api';
import { useStream } from './lib/ws';
import type { StreamMessage } from './lib/ws';

export default function App() {
  const [status, setStatus] = useState<GatewayStatus | null>(null);
  const [pose, setPose] = useState<Pose | null>(null);
  const [picked, setPicked] = useState<{ x: number; y: number } | null>(null);
  const [feed, setFeed] = useState<StreamMessage[]>([]);

  // Poll status for the header pills; pose comes from the WS stream.
  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const s = await api.status();
        if (alive) {
          setStatus(s);
          // When no WS message has arrived yet, prime pose from /status.
          setPose(p => p ?? s.pose);
        }
      } catch {
        if (alive) setStatus(null);
      }
    };
    tick();
    const t = setInterval(tick, 3000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  const onMessage = useCallback((m: StreamMessage) => {
    if (m.type === 'pose') {
      setPose({ x: m.x, y: m.y, yaw: m.yaw, stamp: m.stamp });
    } else if (m.type === 'event') {
      setFeed(f => [...f.slice(-199), m]);
    }
  }, []);
  const { connected } = useStream({ onMessage });

  return (
    <div className="app">
      <header>
        <h1>TypeGo Operator Console</h1>
        <span className={`pill ${connected ? 'ok' : 'bad'}`}>
          {connected ? 'stream connected' : 'stream offline'}
        </span>
        <span className={`pill ${status?.nav_server_ready ? 'ok' : 'bad'}`}>
          nav {status?.nav_server_ready ? 'ready' : 'waiting'}
        </span>
        <span className={`pill ${status?.has_map ? 'ok' : 'bad'}`}>
          map {status?.has_map ? 'loaded' : 'waiting'}
        </span>
        {pose && (
          <span className="pill">
            pose ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}, {
              (pose.yaw * 180 / Math.PI).toFixed(0)
            }°)
          </span>
        )}
      </header>

      <MapView pose={pose} onPick={(x, y) => setPicked({ x, y })} />

      <aside className="sidebar">
        <GoalForm picked={picked} onPicked={() => setPicked(null)} />
        <PatrolPanel />
        <SpeedControl />
        <BagDownload bagCount={status?.bag?.count ?? null} />
        <EventsFeed feed={feed} />
      </aside>
    </div>
  );
}
