// Pure (ROS-free) waypoint anchor model for place_graph_waypoints_node.
//
// The node generates candidate waypoints from a PlaceGraphSnapshot and hands
// them to an AnchorRegistry, which decides which become fixed anchors: an
// anchor keeps its ID *and* its frozen pose across refreshes until that pose
// becomes invalid, at which point it is retired and replaced. Keeping this
// logic free of ROS lets it be unit-tested directly (see
// test/test_waypoint_anchor.cpp).

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "place_graph/types.hpp"

namespace typego_sdk {

namespace tpg = place_graph;

// Volatile (frontier) waypoint IDs live in a reserved high range so they never
// advance the persistent next_id counter.
constexpr std::uint32_t kFrontierIdBase = 1000000000u;

// Operator-created (user) waypoint IDs live above the frontier range so an id
// alone disambiguates origin: auto [0, 1e9), frontier [1e9, 2e9), user [2e9, …).
constexpr std::uint32_t kUserIdBase = 2000000000u;

// A waypoint as published / persisted.
struct PublishedWaypoint {
    std::uint32_t id = 0;
    double x = 0.0, y = 0.0, yaw = 0.0;
    std::string label;
    std::string semantic_context;
    std::string source = "place_graph";
    std::string generation_reason;  // == waypoint_role
    std::string place_id;
    std::string waypoint_role;
    double clearance_m = 0.0;
    double confidence = 0.0;
};

// A desired waypoint proposed from the graph (before reconciliation).
struct Candidate {
    std::string generation_key;
    std::string place_id;
    tpg::PlaceKind place_kind = tpg::PlaceKind::kRoom;
    std::string waypoint_role;
    double x = 0.0, y = 0.0, yaw = 0.0;
    std::string semantic_context;
};

// A persisted fixed anchor.
struct AnchorRecord {
    std::uint32_t id = 0;
    std::string generation_key;
    std::string place_id;
    std::string waypoint_role;
    double x = 0.0, y = 0.0, yaw = 0.0;
    bool active = false;
    bool stale = false;
    std::uint32_t stale_refresh_count = 0;
};

bool is_connector(tpg::PlaceKind k);

// Wrap an angle to (-pi, pi].
double normalize_angle(double a);

// The denormalized display label derived from a place:
// operator_label (if locked) > instance_label > semantic_label > place_id.
std::string effective_label(const tpg::PlaceRegion& p);

// True if every cell whose center is within `min_clear` *meters* of (x,y) is
// in-bounds and free in `map`. The metric (not inflated-integer) threshold is
// exact. This is the near-wall safety check `pose_valid` layers on top of
// place resolution, and the gate operator pins reuse.
bool clearance_ok(double x, double y, const tpg::MapSnapshot& map,
                  double min_clear);

// Re-derive the place-owned fields (label, clearance_m) on `w` from its
// `place_id` against graph `g`. Single definition of the denormalization,
// shared by reconcile, user-waypoint publishing, and the label-edit refresh.
void refresh_place_info(const tpg::PlaceGraphSnapshot& g, PublishedWaypoint& w);

// True if the (frozen) pose is still a usable anchor for `place_id` in graph
// `g`: it resolves to that place (for connectors, to the connector or one of
// its adjacent places) and has at least `min_anchor_clearance_m` of free space
// around it in the input map.
bool pose_valid(double x, double y, const std::string& place_id, bool connector,
                const tpg::PlaceGraphSnapshot& g, const tpg::MapSnapshot& map,
                double min_anchor_clearance_m);

// Holds the persistent fixed-anchor state and reconciles a fresh set of
// candidates against it each refresh.
class AnchorRegistry {
 public:
    // Tunables (set by the node from ROS params; defaults match the node).
    double min_anchor_clearance_m = 0.25;
    int stale_anchor_retire_refreshes = 3;
    bool publish_stale_anchors = false;
    // Minimum spacing (m) enforced on the poses actually published. Needed
    // because the candidate-time dedup tests fresh candidate poses while
    // reconcile publishes frozen record poses, which can drift together. <= 0
    // disables the pass.
    double min_separation_m = 0.0;

    // Persistent state (serialized by the node).
    std::unordered_map<std::string, AnchorRecord> records;
    std::uint32_t next_id = 0;

    // Reconcile candidates against the registry and return the waypoints to
    // publish. Soft anchors (center) are gated while their place is
    // provisional; frontier candidates are volatile (published, never stored);
    // a known anchor keeps its frozen pose while valid, and is retired +
    // replaced only once a replacement can actually be minted. The published
    // set is finally thinned to min_separation_m.
    std::vector<PublishedWaypoint> reconcile(
        const std::vector<Candidate>& candidates,
        const tpg::PlaceGraphSnapshot& g, const tpg::MapSnapshot& map);

 private:
    // Increment a record's stale counter and report whether it has now been
    // unmatched/invalid for `stale_anchor_retire_refreshes` refreshes.
    bool should_retire(AnchorRecord& rec) const {
        return ++rec.stale_refresh_count >=
               static_cast<std::uint32_t>(stale_anchor_retire_refreshes);
    }
    void mint_if_valid(const Candidate& cand, bool connector,
                       const tpg::PlaceGraphSnapshot& g,
                       const tpg::MapSnapshot& map,
                       std::unordered_set<std::string>& seen_keys,
                       std::vector<PublishedWaypoint>& out);
    PublishedWaypoint make_published(std::uint32_t id, double x, double y,
                                     double yaw, const Candidate& cand,
                                     const tpg::PlaceGraphSnapshot& g) const;
    PublishedWaypoint make_published_from_record(
        const AnchorRecord& rec, const tpg::PlaceGraphSnapshot& g) const;
    // Withhold (do not retire) anchors closer than min_separation_m to a
    // higher-priority one. Frontier waypoints are exempt.
    void suppress_clustered(std::vector<PublishedWaypoint>& wps) const;
};

}  // namespace typego_sdk
