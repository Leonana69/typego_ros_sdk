import { useState } from 'react';
import type { PlaceRegion } from '../lib/api';
import { AFFORDANCES, ROOM_TYPES, api } from '../lib/api';

interface Props {
  place: PlaceRegion;
  onClose: () => void;
  // Called after a successful save so the parent can re-poll.
  onSaved?: () => void;
}

export default function SemanticLabelEditor({ place, onClose, onSaved }: Props) {
  const [semanticLabel, setSemanticLabel] = useState(
    place.semantic_label || 'unknown',
  );
  const [instanceLabel, setInstanceLabel] = useState(
    place.instance_label || '',
  );
  const [objects, setObjects] = useState((place.objects || []).join(', '));
  const [affordances, setAffordances] = useState(
    new Set(place.affordances || []),
  );
  const [summary, setSummary] = useState(place.summary || '');
  const [msg, setMsg] = useState('');
  const [busy, setBusy] = useState(false);

  const toggleAffordance = (a: string) => {
    setAffordances(prev => {
      const next = new Set(prev);
      if (next.has(a)) next.delete(a);
      else next.add(a);
      return next;
    });
  };

  const save = async () => {
    setBusy(true);
    setMsg('');
    try {
      const r = await api.setPlaceLabel(place.place_id, {
        semantic_label: semanticLabel,
        instance_label: instanceLabel || undefined,
        objects: objects
          .split(',')
          .map(s => s.trim())
          .filter(Boolean),
        affordances: [...affordances],
        summary: summary || undefined,
      });
      if (r.success) {
        onSaved?.();
        onClose();
      } else {
        setMsg(`✗ ${r.message || 'label rejected'}`);
      }
    } catch (e) {
      // jsonOrThrow surfaces a 409 (rejection) as a thrown error.
      setMsg(String(e));
    } finally {
      setBusy(false);
    }
  };

  return (
    <div className="modal-overlay" onClick={onClose}>
      <div className="panel modal" onClick={e => e.stopPropagation()}>
        <h2>Edit Region Label</h2>
        <div className="muted" style={{ marginBottom: 8, fontSize: 11 }}>
          {place.place_id} · kind {place.kind} ·{' '}
          {place.member_waypoint_ids?.length ?? 0} waypoint(s)
        </div>
        <div className="kv">
          <span className="k">source</span>
          <span className="v">{place.source || '—'}</span>
        </div>
        <div className="kv">
          <span className="k">locked</span>
          <span className="v">{String(place.label_locked)}</span>
        </div>

        <div className="row">
          <label>Type</label>
          <select
            value={semanticLabel}
            onChange={e => setSemanticLabel(e.target.value)}
          >
            {ROOM_TYPES.map(t => (
              <option key={t} value={t}>
                {t}
              </option>
            ))}
          </select>
        </div>
        <div className="row">
          <label>Name</label>
          <input
            type="text"
            value={instanceLabel}
            placeholder="optional instance name"
            onChange={e => setInstanceLabel(e.target.value)}
          />
        </div>
        <div className="row">
          <label>Objects</label>
          <input
            type="text"
            value={objects}
            placeholder="comma-separated"
            onChange={e => setObjects(e.target.value)}
          />
        </div>
        <div className="row" style={{ alignItems: 'flex-start' }}>
          <label>Affordances</label>
          <div className="chips">
            {AFFORDANCES.map(a => (
              <button
                key={a}
                type="button"
                className={`chip ${affordances.has(a) ? 'on' : ''}`}
                onClick={() => toggleAffordance(a)}
              >
                {a}
              </button>
            ))}
          </div>
        </div>
        <div className="row" style={{ alignItems: 'flex-start' }}>
          <label>Summary</label>
          <textarea
            value={summary}
            rows={2}
            placeholder="optional"
            onChange={e => setSummary(e.target.value)}
          />
        </div>

        {msg && <div className="muted" style={{ marginTop: 6 }}>{msg}</div>}
        <div className="row" style={{ marginTop: 8 }}>
          <button onClick={save} disabled={busy}>Save</button>
          <button className="secondary" onClick={onClose}>Cancel</button>
        </div>
      </div>
    </div>
  );
}
