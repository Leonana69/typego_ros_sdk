import { useCallback, useEffect, useState } from 'react';
import type { PlaceRegion, Waypoint } from '../lib/api';
import { api, isLabelableKind, kUserIdBase } from '../lib/api';

interface Props {
  // App owns the interaction mode so the map click handler can react.
  onAdd: () => void;
  onMove: (id: number) => void;
  onEditPlace: (place: PlaceRegion) => void;
  // Lets App refresh the map overlay after a delete, and surface errors.
  onChanged?: () => void;
  onError?: (msg: string) => void;
  // Bumped by App to force a re-poll (e.g. after add/move/label edits).
  refreshToken?: number;
}

function placeLabel(place: PlaceRegion | undefined, wp: Waypoint): string {
  if (place) {
    return (
      place.operator_label ||
      place.instance_label ||
      place.semantic_label ||
      place.place_id
    );
  }
  return wp.label || wp.place_id || 'unknown';
}

export default function WaypointPanel({
  onAdd,
  onMove,
  onEditPlace,
  onChanged,
  onError,
  refreshToken,
}: Props) {
  const [waypoints, setWaypoints] = useState<Waypoint[]>([]);
  const [parked, setParked] = useState<Waypoint[]>([]);
  const [places, setPlaces] = useState<Map<string, PlaceRegion>>(new Map());

  const load = useCallback(async () => {
    try {
      const [wr, pr] = await Promise.all([api.waypoints(), api.places()]);
      setWaypoints(wr.waypoints);
      setParked(wr.parked);
      const m = new Map<string, PlaceRegion>();
      for (const p of [...pr.places, ...pr.portals]) m.set(p.place_id, p);
      setPlaces(m);
    } catch {
      /* ignore — transient */
    }
  }, []);

  useEffect(() => {
    let alive = true;
    const tick = () => {
      if (alive) load();
    };
    tick();
    const t = setInterval(tick, 4000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, [load, refreshToken]);

  const remove = async (id: number) => {
    try {
      const r = await api.deleteWaypoint(id);
      if (r.success === false) onError?.(r.message || 'delete rejected');
      onChanged?.();
      load();
    } catch (e) {
      onError?.(String(e));
    }
  };

  // Group active waypoints by place_id, preserving discovery order.
  const groups: Array<{ placeId: string; wps: Waypoint[] }> = [];
  const groupIdx = new Map<string, number>();
  for (const wp of waypoints) {
    const key = wp.place_id || '(unassigned)';
    let i = groupIdx.get(key);
    if (i === undefined) {
      i = groups.length;
      groupIdx.set(key, i);
      groups.push({ placeId: key, wps: [] });
    }
    groups[i].wps.push(wp);
  }

  const renderRow = (wp: Waypoint, dimmed: boolean) => {
    const isUser = wp.id >= kUserIdBase;
    return (
      <div key={wp.id} className={`wp-row ${dimmed ? 'parked' : ''}`}>
        <div className="wp-main">
          <span className="wp-glyph">{isUser ? '◆' : '●'}</span>
          <span className="wp-id">#{wp.id}</span>
          <span className="wp-pos">
            ({wp.x.toFixed(2)}, {wp.y.toFixed(2)})
          </span>
          {wp.waypoint_role && (
            <span className="pill">{wp.waypoint_role}</span>
          )}
          {wp.source && <span className="pill">{wp.source}</span>}
        </div>
        {isUser && (
          <div className="wp-actions">
            <button className="secondary" onClick={() => onMove(wp.id)}>
              Move
            </button>
            <button className="danger" onClick={() => remove(wp.id)}>
              Delete
            </button>
          </div>
        )}
      </div>
    );
  };

  return (
    <div className="panel">
      <h2>Waypoints</h2>
      <div className="row">
        <button onClick={onAdd}>Add waypoint</button>
      </div>

      {groups.length === 0 && (
        <div className="muted" style={{ marginTop: 6 }}>no waypoints</div>
      )}

      {groups.map(g => {
        const place = places.get(g.placeId);
        const label = placeLabel(place, g.wps[0]);
        const labelable = isLabelableKind(place?.kind);
        return (
          <div key={g.placeId} className="wp-group">
            <div className="wp-group-head">
              <span className="wp-region">{label}</span>
              {place && <span className="pill">{place.kind}</span>}
              {place?.label_locked && <span className="pill ok">locked</span>}
              {place?.source && <span className="pill">{place.source}</span>}
              {labelable && place && (
                <button
                  className="secondary"
                  onClick={() => onEditPlace(place)}
                >
                  Edit label
                </button>
              )}
            </div>
            {g.wps.map(wp => renderRow(wp, false))}
          </div>
        );
      })}

      {parked.length > 0 && (
        <div className="wp-group">
          <div className="wp-group-head">
            <span className="wp-region">Parked</span>
            <span className="muted" style={{ fontSize: 11 }}>
              withheld from navigation
            </span>
          </div>
          {parked.map(wp => renderRow(wp, true))}
        </div>
      )}
    </div>
  );
}
