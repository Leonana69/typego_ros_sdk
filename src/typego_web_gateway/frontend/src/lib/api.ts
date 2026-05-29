export interface MapMeta {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
  origin_yaw: number;
  frame_id: string;
  stamp: number;
}

export interface Pose {
  x: number;
  y: number;
  yaw: number;
  stamp: number;
}

export interface GatewayStatus {
  action_name: string;
  robot_namespace: string;
  has_map: boolean;
  has_pose: boolean;
  waypoint_count: number;
  nav_server_ready: boolean;
  speed_service_ready: boolean;
  pose: Pose | null;
  patrol: PatrolStatus;
  bag: { dir: string; count: number } | null;
}

export interface PatrolStatus {
  running: boolean;
  map_name: string;
  indices: number[];
  current_index: number | null;
  lap: number;
  last_error: string;
}

export interface Waypoint {
  id: number;
  x: number;
  y: number;
  label: string;
  yaw?: number;
  semantic_context?: string;
  confidence?: number;
  source?: string;
  clearance_m?: number;
  generation_reason?: string;
}

export interface RobotConfigIdentity {
  id: string;
  type: 'go2' | 'kami';
  name: string;
}

export interface RobotConfigResponse {
  available: boolean;
  config?: {
    robot: RobotConfigIdentity;
    autonomy: { type: 'base' | 'full' };
    [key: string]: unknown;
  };
  message?: string;
}

async function jsonOrThrow<T>(req: Promise<Response>): Promise<T> {
  const res = await req;
  if (!res.ok) {
    const body = await res.text();
    throw new Error(`${res.status} ${res.statusText}: ${body}`);
  }
  return (await res.json()) as T;
}

export const api = {
  status: () => jsonOrThrow<GatewayStatus>(fetch('/api/status')),
  config: () =>
    fetch('/api/config').then(async (res) => {
      const body = await res.json();
      return body as RobotConfigResponse;
    }),
  mapMeta: () => jsonOrThrow<MapMeta>(fetch('/api/map/meta')),
  mapUrl: () => `/api/map?ts=${Date.now()}`,
  waypoints: () =>
    jsonOrThrow<{ waypoints: Waypoint[] }>(fetch('/api/waypoints')),
  events: (limit = 100) =>
    jsonOrThrow<{ events: Array<Record<string, unknown>> }>(
      fetch(`/api/events?limit=${limit}`),
    ),
  sendGoal: (x: number, y: number, yaw_deg: number) =>
    jsonOrThrow<{ accepted: boolean; message: string }>(
      fetch('/api/goal', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ x, y, yaw_deg }),
      }),
    ),
  cancelGoal: () =>
    jsonOrThrow<{ canceled: boolean; message: string }>(
      fetch('/api/goal/cancel', { method: 'POST' }),
    ),
  startPatrol: (map_name: string, indices: number[]) =>
    jsonOrThrow<{ started: boolean; message: string }>(
      fetch('/api/patrol/start', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ map_name, indices }),
      }),
    ),
  stopPatrol: () =>
    jsonOrThrow<{ stopped: boolean; message: string }>(
      fetch('/api/patrol/stop', { method: 'POST' }),
    ),
  setSpeed: (max_linear: number, max_angular: number) =>
    jsonOrThrow<{ success: boolean; message: string }>(
      fetch('/api/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ max_linear, max_angular }),
      }),
    ),
  bagDownloadHref: (minutes = 60) => `/api/bag/latest?minutes=${minutes}`,
};
