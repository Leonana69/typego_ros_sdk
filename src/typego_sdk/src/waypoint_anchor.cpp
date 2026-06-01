#include "typego_sdk/waypoint_anchor.hpp"

#include <cmath>

namespace typego_sdk {

namespace {

constexpr double kPi = 3.14159265358979323846;

bool is_soft_role(const std::string& role) {
    return role == "center" || role == "coverage";
}

const tpg::PlaceRegion* find_place(const tpg::PlaceGraphSnapshot& g,
                                   const std::string& place_id) {
    for (const auto& p : g.places)
        if (p.place_id == place_id) return &p;
    return nullptr;
}

}  // namespace

double normalize_angle(double a) {
    while (a > kPi) a -= 2.0 * kPi;
    while (a < -kPi) a += 2.0 * kPi;
    return a;
}

std::string effective_label(const tpg::PlaceRegion& p) {
    if (p.label_locked && !p.operator_label.empty()) return p.operator_label;
    if (!p.instance_label.empty()) return p.instance_label;
    if (!p.semantic_label.empty()) return p.semantic_label;
    return p.place_id;
}

// Re-derive the place-owned fields (label, clearance) on a published waypoint
// from its place_id, falling back to the raw place_id and zero clearance when
// the place is not present in the graph.
void refresh_place_info(const tpg::PlaceGraphSnapshot& g, PublishedWaypoint& w) {
    const tpg::PlaceRegion* p = find_place(g, w.place_id);
    w.label = p ? effective_label(*p) : w.place_id;
    w.clearance_m = p ? p->clearance_m : 0.0;
}

// All cells whose center is within `min_clear` *meters* of (x,y) are free in
// the input map. Uses the metric distance (not an inflated integer radius) so
// the threshold is exact: a 0.25 m requirement does not reject a wall 0.3 m
// away, which matters for doorway/connector anchors that sit in narrow gaps.
bool clearance_ok(double x, double y, const tpg::MapSnapshot& map,
                  double min_clear) {
    if (map.resolution_m <= 0.0) return true;
    int r = static_cast<int>(std::ceil(min_clear / map.resolution_m));
    int cx = static_cast<int>((x - map.origin_x) / map.resolution_m);
    int cy = static_cast<int>((y - map.origin_y) / map.resolution_m);
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            double d = std::hypot(static_cast<double>(dx),
                                  static_cast<double>(dy)) *
                       map.resolution_m;
            if (d > min_clear) continue;
            int nx = cx + dx, ny = cy + dy;
            if (!map.in_bounds(nx, ny)) return false;
            if (map.at(nx, ny) != tpg::CellState::kFree) return false;
        }
    }
    return true;
}

bool is_connector(tpg::PlaceKind k) {
    return k == tpg::PlaceKind::kPortal ||
           k == tpg::PlaceKind::kOpenTransition ||
           k == tpg::PlaceKind::kJunction;
}

bool pose_valid(double x, double y, const std::string& place_id, bool connector,
                const tpg::PlaceGraphSnapshot& g, const tpg::MapSnapshot& map,
                double min_anchor_clearance_m) {
    const tpg::PlaceRegion* at = g.place_at_world(x, y);
    if (!at) return false;
    if (connector) {
        // The connector must still exist, and the frozen pose must still lie in
        // the connector itself or in one of the rooms it joins (a boundary can
        // shift it by a cell). A pose that drifted into an unrelated region is
        // not a valid doorway anchor.
        const tpg::PlaceRegion* conn = find_place(g, place_id);
        if (!conn) return false;
        bool in_scope = (at->place_id == place_id);
        if (!in_scope) {
            for (const auto& a : conn->adjacent_place_ids)
                if (a == at->place_id) {
                    in_scope = true;
                    break;
                }
        }
        if (!in_scope) return false;
    } else {
        if (at->place_id != place_id) return false;
    }
    // Clearance applies to connectors too: a doorway that is now blocked or
    // wall-adjacent is no longer a usable anchor.
    return clearance_ok(x, y, map, min_anchor_clearance_m);
}

std::vector<PublishedWaypoint> AnchorRegistry::reconcile(
    const std::vector<Candidate>& candidates, const tpg::PlaceGraphSnapshot& g,
    const tpg::MapSnapshot& map) {
    std::vector<PublishedWaypoint> out;
    std::unordered_set<std::string> seen_keys;
    std::uint32_t frontier_id = kFrontierIdBase;

    for (const auto& cand : candidates) {
        bool connector = is_connector(cand.place_kind);

        // Frontier waypoints are volatile: published, never registered.
        if (cand.place_kind == tpg::PlaceKind::kFrontierRegion) {
            if (pose_valid(cand.x, cand.y, cand.place_id, false, g, map,
                           min_anchor_clearance_m)) {
                out.push_back(make_published(frontier_id++, cand.x, cand.y,
                                             cand.yaw, cand, g));
            }
            continue;
        }

        seen_keys.insert(cand.generation_key);
        auto it = records.find(cand.generation_key);
        if (it != records.end()) {
            AnchorRecord& rec = it->second;
            if (pose_valid(rec.x, rec.y, rec.place_id, connector, g, map,
                           min_anchor_clearance_m)) {
                rec.active = true;
                rec.stale = false;
                rec.stale_refresh_count = 0;
                rec.place_id = cand.place_id;  // refresh (split/merge)
                out.push_back(make_published(rec.id, rec.x, rec.y, rec.yaw, cand,
                                             g));
            } else {
                rec.stale = true;
                rec.active = false;
                if (should_retire(rec)) {
                    records.erase(it);
                    mint_if_valid(cand, connector, g, map, seen_keys, out);
                }
                // else: withhold this refresh (degraded but not moved)
            }
        } else {
            mint_if_valid(cand, connector, g, map, seen_keys, out);
        }
    }

    // Keys with active records but no candidate this refresh: go stale.
    for (auto it = records.begin(); it != records.end();) {
        if (seen_keys.count(it->first)) {
            ++it;
            continue;
        }
        AnchorRecord& rec = it->second;
        rec.active = false;
        rec.stale = true;
        if (should_retire(rec)) {
            it = records.erase(it);
            continue;
        }
        if (publish_stale_anchors) {
            const tpg::PlaceRegion* p = find_place(g, rec.place_id);
            bool connector = p && is_connector(p->kind);
            if (pose_valid(rec.x, rec.y, rec.place_id, connector, g, map,
                           min_anchor_clearance_m)) {
                out.push_back(make_published_from_record(rec, g));
            }
        }
        ++it;
    }
    return out;
}

void AnchorRegistry::mint_if_valid(const Candidate& cand, bool connector,
                                   const tpg::PlaceGraphSnapshot& g,
                                   const tpg::MapSnapshot& map,
                                   std::unordered_set<std::string>& seen_keys,
                                   std::vector<PublishedWaypoint>& out) {
    // Provisional gate: don't freeze soft anchors from half-explored places.
    const tpg::PlaceRegion* place = find_place(g, cand.place_id);
    if (is_soft_role(cand.waypoint_role) && place && place->provisional) return;
    if (!pose_valid(cand.x, cand.y, cand.place_id, connector, g, map,
                    min_anchor_clearance_m))
        return;
    AnchorRecord rec;
    rec.id = next_id++;
    rec.generation_key = cand.generation_key;
    rec.place_id = cand.place_id;
    rec.waypoint_role = cand.waypoint_role;
    rec.x = cand.x;
    rec.y = cand.y;
    rec.yaw = cand.yaw;
    rec.active = true;
    rec.stale = false;
    rec.stale_refresh_count = 0;
    records[cand.generation_key] = rec;
    seen_keys.insert(cand.generation_key);
    out.push_back(make_published(rec.id, rec.x, rec.y, rec.yaw, cand, g));
}

PublishedWaypoint AnchorRegistry::make_published(
    std::uint32_t id, double x, double y, double yaw, const Candidate& cand,
    const tpg::PlaceGraphSnapshot& g) const {
    PublishedWaypoint w;
    w.id = id;
    w.x = x;
    w.y = y;
    w.yaw = normalize_angle(yaw);
    w.place_id = cand.place_id;
    w.waypoint_role = cand.waypoint_role;
    w.generation_reason = cand.waypoint_role;
    w.semantic_context = cand.semantic_context;
    refresh_place_info(g, w);
    return w;
}

PublishedWaypoint AnchorRegistry::make_published_from_record(
    const AnchorRecord& rec, const tpg::PlaceGraphSnapshot& g) const {
    PublishedWaypoint w;
    w.id = rec.id;
    w.x = rec.x;
    w.y = rec.y;
    w.yaw = normalize_angle(rec.yaw);
    w.place_id = rec.place_id;
    w.waypoint_role = rec.waypoint_role;
    w.generation_reason = rec.waypoint_role;
    w.semantic_context = rec.waypoint_role + " in " + rec.place_id;
    refresh_place_info(g, w);
    return w;
}

}  // namespace typego_sdk
