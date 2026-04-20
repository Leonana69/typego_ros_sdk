import { useEffect, useState } from 'react';
import type { PatrolStatus } from '../lib/api';
import { api } from '../lib/api';

export default function PatrolPanel() {
  const [mapName, setMapName] = useState('full-map');
  const [indices, setIndices] = useState('1');
  const [status, setStatus] = useState<PatrolStatus | null>(null);
  const [msg, setMsg] = useState('');

  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const s = await api.status();
        if (alive) setStatus(s.patrol);
      } catch {
        /* ignore */
      }
    };
    tick();
    const t = setInterval(tick, 2000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  const start = async () => {
    const parsed = indices
      .split(',')
      .map(s => s.trim())
      .filter(Boolean)
      .map(s => parseInt(s, 10))
      .filter(n => !Number.isNaN(n));
    if (!parsed.length) {
      setMsg('enter one or more waypoint indices, comma-separated');
      return;
    }
    try {
      const r = await api.startPatrol(mapName, parsed);
      setMsg(`${r.started ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    }
  };

  const stop = async () => {
    try {
      const r = await api.stopPatrol();
      setMsg(`${r.stopped ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    }
  };

  const running = status?.running ?? false;
  return (
    <div className="panel">
      <h2>Patrol</h2>
      <div className="row">
        <label>Map</label>
        <input type="text" value={mapName}
               onChange={e => setMapName(e.target.value)} />
      </div>
      <div className="row">
        <label>Indices</label>
        <input type="text" value={indices}
               onChange={e => setIndices(e.target.value)}
               placeholder="e.g. 1,2,3,2" />
      </div>
      <div className="row">
        <button onClick={start} disabled={running}>Start</button>
        <button className="danger" onClick={stop} disabled={!running}>Stop</button>
      </div>
      {status && (
        <>
          <div className="kv">
            <span className="k">running</span>
            <span className="v">{String(status.running)}</span>
          </div>
          <div className="kv">
            <span className="k">lap</span><span className="v">{status.lap}</span>
          </div>
          <div className="kv">
            <span className="k">current</span>
            <span className="v">
              {status.current_index ?? '—'}
            </span>
          </div>
          {status.last_error && (
            <div className="kv">
              <span className="k">last error</span>
              <span className="v" title={status.last_error}>
                {status.last_error.slice(0, 28)}
              </span>
            </div>
          )}
        </>
      )}
      {msg && <div className="muted" style={{ marginTop: 6 }}>{msg}</div>}
    </div>
  );
}
