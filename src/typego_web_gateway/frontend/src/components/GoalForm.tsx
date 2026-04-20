import { useEffect, useState } from 'react';
import { api } from '../lib/api';

interface Props {
  picked: { x: number; y: number } | null;
  onPicked?: () => void;
}

export default function GoalForm({ picked, onPicked }: Props) {
  const [x, setX] = useState('0.0');
  const [y, setY] = useState('0.0');
  const [yaw, setYaw] = useState('0');
  const [msg, setMsg] = useState<string>('');
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (picked) {
      setX(picked.x.toFixed(3));
      setY(picked.y.toFixed(3));
      onPicked?.();
    }
  }, [picked, onPicked]);

  const submit = async () => {
    setBusy(true);
    try {
      const r = await api.sendGoal(
        parseFloat(x), parseFloat(y), parseFloat(yaw),
      );
      setMsg(`${r.accepted ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    } finally {
      setBusy(false);
    }
  };

  const cancel = async () => {
    try {
      const r = await api.cancelGoal();
      setMsg(`${r.canceled ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    }
  };

  return (
    <div className="panel">
      <h2>Send Goal</h2>
      <div className="row">
        <label>X (m)</label>
        <input type="number" step="0.01" value={x}
               onChange={e => setX(e.target.value)} />
      </div>
      <div className="row">
        <label>Y (m)</label>
        <input type="number" step="0.01" value={y}
               onChange={e => setY(e.target.value)} />
      </div>
      <div className="row">
        <label>Yaw (°)</label>
        <input type="number" step="1" value={yaw}
               onChange={e => setYaw(e.target.value)} />
      </div>
      <div className="row">
        <button onClick={submit} disabled={busy}>Send</button>
        <button className="danger" onClick={cancel}>Cancel</button>
      </div>
      {msg && <div className="muted" style={{ marginTop: 6 }}>{msg}</div>}
      <div className="muted" style={{ fontSize: 11, marginTop: 6 }}>
        Tip: click anywhere on the map to prefill X/Y.
      </div>
    </div>
  );
}
