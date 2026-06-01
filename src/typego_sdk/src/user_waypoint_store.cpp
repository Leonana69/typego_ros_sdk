#include "typego_sdk/user_waypoint_store.hpp"

#include <cmath>

namespace typego_sdk {

UserWaypoint* UserWaypointStore::find(std::uint32_t id) {
    for (auto& uw : waypoints)
        if (uw.id == id) return &uw;
    return nullptr;
}

PublishedWaypoint UserWaypointStore::make_published(
    const UserWaypoint& uw, const tpg::PlaceGraphSnapshot& g) const {
    PublishedWaypoint w;
    w.id = uw.id;
    w.x = uw.x;
    w.y = uw.y;
    w.yaw = normalize_angle(uw.yaw);
    w.source = "operator";
    w.waypoint_role = "operator";
    w.generation_reason = "operator";
    w.place_id = uw.place_id;
    w.semantic_context = "operator waypoint in " + uw.place_id;
    refresh_place_info(g, w);  // label + clearance from the owning place
    return w;
}

EditResult UserWaypointStore::apply_edit(const std::string& op,
                                         std::uint32_t id, double x, double y,
                                         double yaw, bool has_yaw,
                                         const tpg::PlaceGraphSnapshot& g,
                                         const tpg::MapSnapshot& map) {
    if (op == "add") {
        const tpg::PlaceRegion* at = g.place_at_world(x, y);
        if (!at) return {false, "point is in unknown/obstacle space", 0};
        if (!clearance_ok(x, y, map, min_clearance_m))
            return {false, "too close to a wall/obstacle", 0};
        UserWaypoint uw;
        uw.id = next_id;
        uw.x = x;
        uw.y = y;
        uw.has_yaw = has_yaw;
        uw.yaw = has_yaw ? yaw : std::atan2(at->peak.y - y, at->peak.x - x);
        uw.place_id = at->place_id;
        waypoints.push_back(uw);
        return {true, "", next_id++};
    }
    if (op == "move") {
        if (id < kUserIdBase)
            return {false, "only operator waypoints can be moved", id};
        UserWaypoint* uw = find(id);
        if (!uw) return {false, "waypoint not found", id};
        const tpg::PlaceRegion* at = g.place_at_world(x, y);
        if (!at) return {false, "point is in unknown/obstacle space", id};
        if (!clearance_ok(x, y, map, min_clearance_m))
            return {false, "too close to a wall/obstacle", id};
        uw->x = x;
        uw->y = y;
        uw->has_yaw = has_yaw;
        uw->yaw = has_yaw ? yaw : std::atan2(at->peak.y - y, at->peak.x - x);
        uw->place_id = at->place_id;
        return {true, "", id};
    }
    if (op == "delete") {
        if (id < kUserIdBase)
            return {false,
                    "auto waypoints are geometry-generated; relabel the region "
                    "instead",
                    id};
        for (auto it = waypoints.begin(); it != waypoints.end(); ++it) {
            if (it->id == id) {
                waypoints.erase(it);
                return {true, "", id};
            }
        }
        return {false, "waypoint not found", id};
    }
    return {false, "unknown op: " + op, id};
}

void UserWaypointStore::merge_into(std::vector<PublishedWaypoint>& active,
                                   std::vector<PublishedWaypoint>& parked,
                                   const tpg::PlaceGraphSnapshot& g,
                                   const tpg::MapSnapshot& map) const {
    for (const auto& uw : waypoints) {
        const tpg::PlaceRegion* at = g.place_at_world(uw.x, uw.y);
        bool valid = at && clearance_ok(uw.x, uw.y, map, min_clearance_m);
        // Recompute place_id from the live graph so labels/grouping track
        // split/merge; a parked pin keeps its last-known place_id.
        UserWaypoint resolved = uw;
        if (at) resolved.place_id = at->place_id;
        PublishedWaypoint w = make_published(resolved, g);
        (valid ? active : parked).push_back(w);
    }
}

nlohmann::json UserWaypointStore::to_json() const {
    nlohmann::json arr = nlohmann::json::array();
    for (const auto& uw : waypoints) {
        arr.push_back({
            {"id", uw.id},
            {"x", uw.x},
            {"y", uw.y},
            {"yaw", uw.yaw},
            {"has_yaw", uw.has_yaw},
            {"place_id", uw.place_id},
        });
    }
    nlohmann::json doc;
    doc["user_waypoints"] = arr;
    doc["next_user_waypoint_id"] = next_id;
    return doc;
}

void UserWaypointStore::from_json(const nlohmann::json& j) {
    waypoints.clear();
    next_id = j.value("next_user_waypoint_id", kUserIdBase);
    if (j.contains("user_waypoints")) {
        for (const auto& a : j["user_waypoints"]) {
            UserWaypoint uw;
            uw.id = a.value("id", 0u);
            uw.x = a.value("x", 0.0);
            uw.y = a.value("y", 0.0);
            uw.yaw = a.value("yaw", 0.0);
            uw.has_yaw = a.value("has_yaw", false);
            uw.place_id = a.value("place_id", "");
            waypoints.push_back(uw);
        }
    }
}

}  // namespace typego_sdk
