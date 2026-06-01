import { useCallback, useEffect, useState } from 'react';
import BagDownload from './components/BagDownload';
import EventsFeed from './components/EventsFeed';
import GoalForm from './components/GoalForm';
import MapView from './components/MapView';
import type { MapMode } from './components/MapView';
import PatrolPanel from './components/PatrolPanel';
import RobotView from './components/RobotView';
import SemanticLabelEditor from './components/SemanticLabelEditor';
import SpeedControl from './components/SpeedControl';
import WaypointPanel from './components/WaypointPanel';
import type {
  GatewayStatus,
  PlaceRegion,
  Pose,
  RobotConfigResponse,
  Waypoint,
} from './lib/api';
import { api, isLabelableKind } from './lib/api';
import { useStream } from './lib/ws';
import type { StreamMessage } from './lib/ws';

type Tab = 'control' | 'waypoints';

export default function App() {
  const [status, setStatus] = useState<GatewayStatus | null>(null);
  const [pose, setPose] = useState<Pose | null>(null);
  const [picked, setPicked] = useState<{ x: number; y: number } | null>(null);
  const [feed, setFeed] = useState<StreamMessage[]>([]);
  const [config, setConfig] = useState<RobotConfigResponse | null>(null);

  const [tab, setTab] = useState<Tab>('control');
  // Map interaction: each tab has a default (Control→navigate, Waypoints→label),
  // but an explicit on-map toggle can override it in either tab. `transient`
  // holds the single-shot add/move that takes precedence over the base mode.
  const [modeOverride, setModeOverride] = useState<'goal' | 'select' | null>(
    null,
  );
  const [transient, setTransient] = useState<'add' | 'move' | null>(null);
  const [moveTargetId, setMoveTargetId] = useState<number | null>(null);
  const [editPlace, setEditPlace] = useState<PlaceRegion | null>(null);
  const [notice, setNotice] = useState<string>('');
  // Bumped to force the WaypointPanel/map overlays to re-poll immediately.
  const [refreshToken, setRefreshToken] = useState(0);
  // Counts for the header info line.
  const [counts, setCounts] = useState({ waypoints: 0, parked: 0, places: 0 });

  const refresh = useCallback(() => setRefreshToken(t => t + 1), []);
  const flash = useCallback((m: string) => {
    setNotice(m);
    window.setTimeout(() => setNotice(n => (n === m ? '' : n)), 6000);
  }, []);

  // Explicit Navigate/Label toggle — overrides the tab default and cancels any
  // pending add/move.
  const setBase = useCallback((m: 'goal' | 'select') => {
    setModeOverride(m);
    setTransient(null);
    setMoveTargetId(null);
  }, []);
  // Switching tabs resumes that tab's default interaction.
  const switchTab = useCallback((t: Tab) => {
    setTab(t);
    setModeOverride(null);
  }, []);

  // Poll status for the header pills; pose comes from the WS stream.
  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const s = await api.status();
        if (alive) {
          setStatus(s);
          setPose(p => p ?? s.pose);
        }
      } catch {
        if (alive) setStatus(null);
      }
      try {
        const c = await api.config();
        if (alive) setConfig(c);
      } catch {
        /* ignore — config service is optional */
      }
    };
    tick();
    const t = setInterval(tick, 5000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, []);

  // Poll counts for the header info line (no new endpoint).
  useEffect(() => {
    let alive = true;
    const tick = async () => {
      try {
        const [wr, pr] = await Promise.all([api.waypoints(), api.places()]);
        if (alive) {
          setCounts({
            waypoints: wr.waypoints.length,
            parked: wr.parked.length,
            places: pr.places.length,
          });
        }
      } catch {
        /* ignore */
      }
    };
    tick();
    const t = setInterval(tick, 5000);
    return () => {
      alive = false;
      clearInterval(t);
    };
  }, [refreshToken]);

  const onMessage = useCallback((m: StreamMessage) => {
    if (m.type === 'pose') {
      setPose({ x: m.x, y: m.y, yaw: m.yaw, stamp: m.stamp });
    } else if (m.type === 'event') {
      setFeed(f => [...f.slice(-199), m]);
    }
  }, []);
  const { connected } = useStream({ onMessage });

  // Resolve a place_id against the polled regions and dispatch on kind (R3).
  const dispatchPlace = useCallback(
    async (placeId: string) => {
      try {
        const pr = await api.places();
        const place =
          pr.places.find(p => p.place_id === placeId) ??
          pr.portals.find(p => p.place_id === placeId);
        if (place && isLabelableKind(place.kind)) {
          setEditPlace(place);
        } else {
          flash('doorway/transition — not labelable');
        }
      } catch (e) {
        flash(String(e));
      }
    },
    [flash],
  );

  const onSelectWaypoint = useCallback(
    (wp: Waypoint) => {
      if (wp.place_id) dispatchPlace(wp.place_id);
      else flash('waypoint has no region');
    },
    [dispatchPlace, flash],
  );

  const onAddWaypoint = useCallback(
    async (x: number, y: number) => {
      setTransient(null);
      try {
        await api.addWaypoint(x, y);
        flash('waypoint added');
        refresh();
      } catch (e) {
        // 409 hard-reject (unsafe spot) surfaces here via jsonOrThrow (R2).
        flash(`add rejected: ${String(e)}`);
      }
    },
    [flash, refresh],
  );

  const onMoveWaypoint = useCallback(
    async (x: number, y: number) => {
      const id = moveTargetId;
      setTransient(null);
      setMoveTargetId(null);
      if (id == null) return;
      try {
        await api.moveWaypoint(id, x, y);
        flash('waypoint moved');
        refresh();
      } catch (e) {
        flash(`move rejected: ${String(e)}`);
      }
    },
    [moveTargetId, flash, refresh],
  );

  const startAdd = useCallback(() => {
    setMoveTargetId(null);
    setTransient('add');
    flash('click the map to place a waypoint');
  }, [flash]);

  const startMove = useCallback(
    (id: number) => {
      setMoveTargetId(id);
      setTransient('move');
      flash(`click the map to move waypoint #${id}`);
    },
    [flash],
  );

  // Base interaction = explicit toggle override, else the tab's default
  // (Control→navigate, Waypoints→label). A pending add/move takes precedence.
  const baseMode: 'goal' | 'select' =
    modeOverride ?? (tab === 'waypoints' ? 'select' : 'goal');
  const effectiveMode: MapMode = transient ?? baseMode;

  return (
    <div className="app">
      <header>
        <h1>TypeGo Operator Console</h1>
        {config?.available && config.config && (
          <span className="pill">
            {config.config.robot.name} · {config.config.robot.type} ·{' '}
            {config.config.autonomy.type}
          </span>
        )}
        <span className={`pill ${connected ? 'ok' : 'bad'}`}>
          {connected ? 'stream connected' : 'stream offline'}
        </span>
        <span className={`pill ${status?.nav_server_ready ? 'ok' : 'bad'}`}>
          nav {status?.nav_server_ready ? 'ready' : 'waiting'}
        </span>
        <span className={`pill ${status?.has_map ? 'ok' : 'bad'}`}>
          map {status?.has_map ? 'loaded' : 'waiting'}
        </span>
        <span className="pill">
          {counts.places} regions · {counts.waypoints} wp
          {counts.parked ? ` · ${counts.parked} parked` : ''}
        </span>
        {pose && (
          <span className="pill">
            pose ({pose.x.toFixed(2)}, {pose.y.toFixed(2)}, {
              (pose.yaw * 180 / Math.PI).toFixed(0)
            }°)
          </span>
        )}
      </header>

      <aside className="sidebar">
        <div className="tab-bar">
          <button
            className={tab === 'control' ? 'active' : ''}
            onClick={() => switchTab('control')}
          >
            Control
          </button>
          <button
            className={tab === 'waypoints' ? 'active' : ''}
            onClick={() => switchTab('waypoints')}
          >
            Waypoints
          </button>
        </div>

        {notice && <div className="notice">{notice}</div>}

        {tab === 'control' && (
          <>
            <GoalForm picked={picked} onPicked={() => setPicked(null)} />
            <PatrolPanel />
            <SpeedControl />
            <BagDownload bagCount={status?.bag?.count ?? null} />
            <EventsFeed feed={feed} />
          </>
        )}

        {tab === 'waypoints' && (
          <WaypointPanel
            onAdd={startAdd}
            onMove={startMove}
            onEditPlace={setEditPlace}
            onChanged={refresh}
            onError={flash}
            refreshToken={refreshToken}
          />
        )}
      </aside>

      <RobotView />

      <MapView
        pose={pose}
        mode={effectiveMode}
        baseMode={baseMode}
        onBaseModeChange={setBase}
        onPick={(x, y) => setPicked({ x, y })}
        onSelectWaypoint={onSelectWaypoint}
        onAddWaypoint={onAddWaypoint}
        onMoveWaypoint={onMoveWaypoint}
        onSelectPlace={dispatchPlace}
      />

      {editPlace && (
        <SemanticLabelEditor
          place={editPlace}
          onClose={() => setEditPlace(null)}
          onSaved={refresh}
        />
      )}
    </div>
  );
}
