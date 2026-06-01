// Operator-created (user) waypoint store — ROS-free, like AnchorRegistry.
//
// Auto/geometry waypoints live in AnchorRegistry, which reconcile() retires
// when their generation_key disappears. Operator waypoints are different: they
// are explicitly created/moved/deleted and must never be retired by reconcile.
// This store owns them, validates edits against the live place graph + map, and
// each refresh splits them into the publishable (resolves + clears) and the
// withheld/"parked" (no longer resolves or too close to an obstacle) sets.
//
// Keeping it free of ROS lets it be unit-tested directly (see
// test/test_waypoint_anchor.cpp), exactly like AnchorRegistry.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "place_graph/types.hpp"
#include "typego_sdk/waypoint_anchor.hpp"

namespace typego_sdk {

namespace tpg = place_graph;

struct UserWaypoint {
    std::uint32_t id = 0;
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool has_yaw = false;
    std::string place_id;  // owning region; recomputed each refresh
};

// Outcome of an add/move/delete. `message` carries the rejection reason on
// failure; `id` is the created/affected id.
struct EditResult {
    bool success = false;
    std::string message;
    std::uint32_t id = 0;
};

class UserWaypointStore {
 public:
    // Clearance floor for a hand-placed pin (meters). Set by the node from the
    // user_waypoint_clearance_m param; defaults to the go2 footprint half-length.
    double min_clearance_m = 0.375;

    std::vector<UserWaypoint> waypoints;
    std::uint32_t next_id = kUserIdBase;

    // Apply an op-dispatched edit. add/move hard-reject (success=false) when the
    // pose does not resolve to a place or lacks clearance; delete rejects ids
    // below kUserIdBase (auto/frontier). On success the store is mutated and the
    // affected id returned.
    EditResult apply_edit(const std::string& op, std::uint32_t id, double x,
                          double y, double yaw, bool has_yaw,
                          const tpg::PlaceGraphSnapshot& g,
                          const tpg::MapSnapshot& map);

    // Append a PublishedWaypoint for each pin: into `active` when it still
    // resolves to a place and clears, otherwise into `parked` (withheld from the
    // navigable output but still surfaced/persisted). Recomputes place_id +
    // label from the live graph so they track split/merge.
    void merge_into(std::vector<PublishedWaypoint>& active,
                    std::vector<PublishedWaypoint>& parked,
                    const tpg::PlaceGraphSnapshot& g,
                    const tpg::MapSnapshot& map) const;

    // {"user_waypoints":[{id,x,y,yaw,has_yaw,place_id}...],
    //  "next_user_waypoint_id":N}. No parked flag: validity is recomputed each
    // refresh from the live map.
    nlohmann::json to_json() const;
    void from_json(const nlohmann::json& j);

 private:
    UserWaypoint* find(std::uint32_t id);
    PublishedWaypoint make_published(const UserWaypoint& uw,
                                     const tpg::PlaceGraphSnapshot& g) const;
};

}  // namespace typego_sdk
