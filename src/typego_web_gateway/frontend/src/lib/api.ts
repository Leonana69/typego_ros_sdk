export interface MapMeta {
  width: number;
  height: number;
  resolution: number;
  origin_x: number;
  origin_y: number;
  origin_yaw: number;
  frame_id: string;
  stamp: number;
  topic?: string;
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
  place_id?: string;
  waypoint_role?: string;
}

export interface PlaceRegion {
  place_id: string;
  kind: string;
  semantic_label: string;
  instance_label: string;
  operator_label: string;
  label_locked: boolean;
  confidence: number;
  source: string;
  objects: string[];
  affordances: string[];
  summary: string;
  centroid_x: number;
  centroid_y: number;
  peak_x: number;
  peak_y: number;
  area_m2: number;
  clearance_m: number;
  frontier_ratio: number;
  elongation: number;
  extent_long_m: number;
  extent_short_m: number;
  provisional: boolean;
  stale: boolean;
  has_approach_yaw: boolean;
  approach_yaw: number;
  has_width_m: boolean;
  width_m: number;
  adjacent_place_ids: string[];
  member_waypoint_ids: number[];
}

export interface PlacesResponse {
  map_version: string;
  places: PlaceRegion[];
  portals: PlaceRegion[];
  summary: string;
}

export interface WaypointsResponse {
  waypoints: Waypoint[];
  parked: Waypoint[];
}

export interface CameraTopics {
  available: boolean;
  frame_id: string | null;
  topic: string;
}

// User/operator waypoints have ids >= kUserIdBase (auto [0,1e9), frontier
// [1e9,2e9), user [2e9,…)) — mirrors typego_sdk/.../waypoint_anchor.hpp.
export const kUserIdBase = 2_000_000_000;

// Mirrors src/place_graph/waypoint_label/waypoint_label/vlm_client.py:22-33.
export const ROOM_TYPES = [
  'bedroom', 'living_room', 'kitchen', 'dining_room', 'bathroom', 'toilet',
  'office', 'study', 'meeting_room', 'hallway', 'corridor', 'foyer',
  'entryway', 'closet', 'storage', 'pantry', 'laundry_room', 'garage',
  'lobby', 'open_area', 'unknown',
];
export const AFFORDANCES = [
  'sleep', 'rest', 'cook', 'make_beverages', 'eat', 'wash', 'toilet',
  'work', 'meet', 'transit', 'wait', 'store', 'seating', 'entertainment',
  'plant_care',
];

// Only region kinds are labelable; mirrors the VLM labeler's ROOM_KINDS
// (src/place_graph/waypoint_label/waypoint_label/semantic_labeler_node.py:39).
export const LABELABLE_KINDS = new Set(['room', 'corridor', 'open_area']);
export function isLabelableKind(kind: string | undefined): boolean {
  return kind !== undefined && LABELABLE_KINDS.has(kind);
}

export interface PlaceLabelBody {
  semantic_label: string;
  instance_label?: string;
  objects?: string[];
  affordances?: string[];
  summary?: string;
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
    jsonOrThrow<WaypointsResponse>(fetch('/api/waypoints')),
  places: () => jsonOrThrow<PlacesResponse>(fetch('/api/places')),
  setPlaceLabel: (placeId: string, body: PlaceLabelBody) =>
    jsonOrThrow<{ success: boolean; message: string }>(
      fetch(`/api/places/${encodeURIComponent(placeId)}/label`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      }),
    ),
  addWaypoint: (x: number, y: number, yaw?: number) =>
    jsonOrThrow<{ id: number }>(
      fetch('/api/waypoints', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(yaw === undefined ? { x, y } : { x, y, yaw }),
      }),
    ),
  moveWaypoint: (id: number, x: number, y: number, yaw?: number) =>
    jsonOrThrow<{ success?: boolean; message?: string }>(
      fetch(`/api/waypoints/${id}`, {
        method: 'PATCH',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(yaw === undefined ? { x, y } : { x, y, yaw }),
      }),
    ),
  deleteWaypoint: (id: number) =>
    jsonOrThrow<{ success?: boolean; message?: string }>(
      fetch(`/api/waypoints/${id}`, { method: 'DELETE' }),
    ),
  resolvePlace: (x: number, y: number) =>
    jsonOrThrow<{ place_id: string | null }>(
      fetch(`/api/places/resolve?x=${x}&y=${y}`),
    ),
  cameraStreamUrl: () => '/api/camera/stream',
  cameraTopics: () =>
    jsonOrThrow<CameraTopics>(fetch('/api/camera/topics')),
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
  setSpeed: (
    max_linear: number,
    max_angular: number,
    accel?: { max_accel?: number; max_decel?: number; max_angular_accel?: number },
  ) =>
    jsonOrThrow<{ success: boolean; message: string }>(
      fetch('/api/speed', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ max_linear, max_angular, ...accel }),
      }),
    ),
  bagDownloadHref: (minutes = 60) => `/api/bag/latest?minutes=${minutes}`,
};
