import { useState } from 'react';
import { api } from '../lib/api';

export default function SpeedControl() {
  const [linear, setLinear] = useState('0.5');
  const [angular, setAngular] = useState('1.0');
  const [msg, setMsg] = useState('');

  const submit = async () => {
    try {
      const r = await api.setSpeed(parseFloat(linear), parseFloat(angular));
      setMsg(`${r.success ? '✓' : '✗'} ${r.message}`);
    } catch (e) {
      setMsg(String(e));
    }
  };

  return (
    <div className="panel">
      <h2>Speed</h2>
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
        <button onClick={submit}>Apply</button>
      </div>
      {msg && <div className="muted" style={{ marginTop: 6 }}>{msg}</div>}
    </div>
  );
}
