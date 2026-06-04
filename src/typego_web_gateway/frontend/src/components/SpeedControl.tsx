import { useState } from 'react';
import { api } from '../lib/api';

export default function SpeedControl() {
  const [linear, setLinear] = useState('0.5');
  const [angular, setAngular] = useState('1.0');
  // Acceleration fields are optional: blank means "no change" (-1), so adjusting
  // only speed never clobbers the running accel/decel limits, and the default
  // values live in the backend (launch / nav2 service) rather than the UI.
  const [accel, setAccel] = useState('');
  const [decel, setDecel] = useState('');
  const [angAccel, setAngAccel] = useState('');
  const [msg, setMsg] = useState('');

  const orNoChange = (s: string): number => {
    const t = s.trim();
    if (t === '') return -1;
    const n = parseFloat(t);
    return Number.isFinite(n) ? n : -1;
  };

  const submit = async () => {
    try {
      const r = await api.setSpeed(parseFloat(linear), parseFloat(angular), {
        max_accel: orNoChange(accel),
        max_decel: orNoChange(decel),
        max_angular_accel: orNoChange(angAccel),
      });
      setMsg(`${r.success ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    }
  };

  return (
    <div className="panel">
      <h2>Speed &amp; Acceleration</h2>
      <div className="row">
        <label>Linear</label>
        <input type="number" step="0.05" min="0" value={linear}
               onChange={e => setLinear(e.target.value)} />
      </div>
      <div className="row">
        <label>Angular</label>
        <input type="number" step="0.05" min="0" value={angular}
               onChange={e => setAngular(e.target.value)} />
      </div>
      <div className="row">
        <label>Accel</label>
        <input type="number" step="0.05" min="0" value={accel} placeholder="no change"
               onChange={e => setAccel(e.target.value)} />
      </div>
      <div className="row">
        <label>Decel</label>
        <input type="number" step="0.05" min="0" value={decel} placeholder="no change"
               onChange={e => setDecel(e.target.value)} />
      </div>
      <div className="row">
        <label>Ang accel</label>
        <input type="number" step="0.05" min="0" value={angAccel} placeholder="no change"
               onChange={e => setAngAccel(e.target.value)} />
      </div>
      <div className="row">
        <button onClick={submit}>Apply</button>
      </div>
      {msg && <div className="muted" style={{ marginTop: 6 }}>{msg}</div>}
    </div>
  );
}
