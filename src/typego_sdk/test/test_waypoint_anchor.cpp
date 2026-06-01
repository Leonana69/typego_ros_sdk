#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "place_graph/place_graph.hpp"
#include "typego_sdk/user_waypoint_store.hpp"
#include "typego_sdk/waypoint_anchor.hpp"

namespace tpg = place_graph;
using typego_sdk::AnchorRegistry;
using typego_sdk::Candidate;
using typego_sdk::EditResult;
using typego_sdk::kFrontierIdBase;
using typego_sdk::kUserIdBase;
using typego_sdk::PublishedWaypoint;
using typego_sdk::UserWaypoint;
using typego_sdk::UserWaypointStore;

namespace {

constexpr int kW = 30, kH = 30;
constexpr double kRes = 0.1;

// Cell center in world meters.
double wc(int cell) { return (cell + 0.5) * kRes; }

// Map that is free everywhere except a 1-cell border, so any interior pose has
// ample clearance; pose invalidity in these tests comes from the place graph,
// not from occupancy.
tpg::MapSnapshot make_free_map() {
    tpg::MapSnapshot m;
    m.width = kW;
    m.height = kH;
    m.resolution_m = kRes;
    m.cells.assign(kW * kH, tpg::CellState::kFree);
    for (int x = 0; x < kW; ++x) {
        m.cells[x] = tpg::CellState::kOccupied;
        m.cells[(kH - 1) * kW + x] = tpg::CellState::kOccupied;
    }
    for (int y = 0; y < kH; ++y) {
        m.cells[y * kW] = tpg::CellState::kOccupied;
        m.cells[y * kW + (kW - 1)] = tpg::CellState::kOccupied;
    }
    return m;
}

class GraphBuilder {
 public:
    GraphBuilder() {
        g.width = kW;
        g.height = kH;
        g.resolution_m = kRes;
        g.place_assignment.assign(kW * kH, tpg::kNoPlace);
    }
    // Add a place occupying cells [x0,x1) x [y0,y1).
    void add(const std::string& id, tpg::PlaceKind kind, int x0, int y0, int x1,
             int y1, bool provisional = false,
             std::vector<std::string> adjacent = {}) {
        tpg::PlaceRegion p;
        p.place_id = id;
        p.kind = kind;
        p.provisional = provisional;
        p.adjacent_place_ids = std::move(adjacent);
        p.clearance_m = 1.0;
        std::uint16_t idx = static_cast<std::uint16_t>(g.places.size());
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                g.place_assignment[y * kW + x] = idx;
                p.cell_region.push_back({x, y});
            }
        p.centroid.x = wc((x0 + x1) / 2);
        p.centroid.y = wc((y0 + y1) / 2);
        p.peak = p.centroid;
        g.places.push_back(p);
    }
    tpg::PlaceGraphSnapshot g;
};

Candidate center_cand(const std::string& place_id, double x, double y) {
    Candidate c;
    c.generation_key = place_id + "|center";
    c.place_id = place_id;
    c.place_kind = tpg::PlaceKind::kRoom;
    c.waypoint_role = "center";
    c.x = x;
    c.y = y;
    return c;
}

}  // namespace

TEST(AnchorRegistry, SameKeyKeepsFrozenPose) {
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 20, 20);
    auto map = make_free_map();
    AnchorRegistry reg;

    auto out1 = reg.reconcile({center_cand("room_0", wc(10), wc(10))}, gb.g, map);
    ASSERT_EQ(out1.size(), 1u);
    std::uint32_t id = out1[0].id;
    EXPECT_DOUBLE_EQ(out1[0].x, wc(10));

    // The candidate "wants" to move, but the anchor keeps its frozen pose+id.
    auto out2 = reg.reconcile({center_cand("room_0", wc(14), wc(14))}, gb.g, map);
    ASSERT_EQ(out2.size(), 1u);
    EXPECT_EQ(out2[0].id, id);
    EXPECT_DOUBLE_EQ(out2[0].x, wc(10));
    EXPECT_DOUBLE_EQ(out2[0].y, wc(10));
}

TEST(AnchorRegistry, InvalidPoseRetiresInsteadOfMoving) {
    auto map = make_free_map();
    AnchorRegistry reg;
    reg.stale_anchor_retire_refreshes = 3;

    // Mint an anchor at cell (10,10).
    GraphBuilder g1;
    g1.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 20, 20);
    auto out0 = reg.reconcile({center_cand("room_0", wc(10), wc(10))}, g1.g, map);
    ASSERT_EQ(out0.size(), 1u);
    std::uint32_t old_id = out0[0].id;

    // The room shifts so the frozen pose (10,10) is no longer in any place,
    // while a fresh candidate at (22,22) is valid.
    GraphBuilder g2;
    g2.add("room_0", tpg::PlaceKind::kRoom, 18, 18, 27, 27);
    Candidate moved = center_cand("room_0", wc(22), wc(22));

    // Grace period: pose is invalid, so the anchor is withheld but not moved.
    auto r1 = reg.reconcile({moved}, g2.g, map);
    EXPECT_TRUE(r1.empty());
    auto r2 = reg.reconcile({moved}, g2.g, map);
    EXPECT_TRUE(r2.empty());
    // Third refresh: retire and mint a replacement with a NEW id at the new pose.
    auto r3 = reg.reconcile({moved}, g2.g, map);
    ASSERT_EQ(r3.size(), 1u);
    EXPECT_NE(r3[0].id, old_id);
    EXPECT_DOUBLE_EQ(r3[0].x, wc(22));
}

TEST(AnchorRegistry, ProvisionalSoftAnchorsGated) {
    auto map = make_free_map();
    AnchorRegistry reg;

    // Provisional room: the soft (center) anchor is withheld.
    GraphBuilder prov;
    prov.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 20, 20, /*provisional=*/true);
    auto r1 = reg.reconcile({center_cand("room_0", wc(10), wc(10))}, prov.g, map);
    EXPECT_TRUE(r1.empty());
    EXPECT_TRUE(reg.records.empty());

    // Once the place closes, the anchor is minted.
    GraphBuilder closed;
    closed.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 20, 20,
               /*provisional=*/false);
    auto r2 = reg.reconcile({center_cand("room_0", wc(10), wc(10))}, closed.g,
                            map);
    ASSERT_EQ(r2.size(), 1u);
    EXPECT_EQ(r2[0].waypoint_role, "center");
}

TEST(AnchorRegistry, RestartRecoveryPreservesIdAndPose) {
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 20, 20);
    auto map = make_free_map();

    AnchorRegistry reg1;
    auto out1 = reg1.reconcile({center_cand("room_0", wc(10), wc(10))}, gb.g,
                               map);
    ASSERT_EQ(out1.size(), 1u);

    // Simulate a restart: a fresh registry restored from persisted state.
    AnchorRegistry reg2;
    reg2.records = reg1.records;
    reg2.next_id = reg1.next_id;

    auto out2 = reg2.reconcile({center_cand("room_0", wc(13), wc(13))}, gb.g,
                               map);
    ASSERT_EQ(out2.size(), 1u);
    EXPECT_EQ(out2[0].id, out1[0].id);
    EXPECT_DOUBLE_EQ(out2[0].x, wc(10));
    EXPECT_DOUBLE_EQ(out2[0].y, wc(10));
}

// The High-priority fix: a connector anchor's frozen pose must still lie in the
// connector or one of its adjacent places, not merely "somewhere".
TEST(WaypointAnchor, ConnectorPoseValidationScope) {
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 2, 2, 9, 28);
    gb.add("room_1", tpg::PlaceKind::kRoom, 11, 2, 18, 28);
    gb.add("portal_0", tpg::PlaceKind::kPortal, 9, 13, 11, 17,
           /*provisional=*/false, {"room_0", "room_1"});
    gb.add("room_2", tpg::PlaceKind::kRoom, 20, 2, 27, 28);  // not adjacent
    auto map = make_free_map();

    // In the connector itself: valid.
    EXPECT_TRUE(typego_sdk::pose_valid(wc(10), wc(15), "portal_0", true, gb.g,
                                       map, 0.25));
    // In an adjacent room (boundary drifted by a cell): still valid.
    EXPECT_TRUE(typego_sdk::pose_valid(wc(5), wc(15), "portal_0", true, gb.g,
                                       map, 0.25));
    // Drifted into an unrelated, non-adjacent region: INVALID (the fix).
    EXPECT_FALSE(typego_sdk::pose_valid(wc(23), wc(15), "portal_0", true, gb.g,
                                        map, 0.25));
    // Resolves to no place at all: invalid.
    EXPECT_FALSE(typego_sdk::pose_valid(wc(0), wc(0), "portal_0", true, gb.g,
                                        map, 0.25));
}

// ---------------- UserWaypointStore (operator waypoints) ----------------

TEST(UserWaypointStore, SchemaV4RoundTrip) {
    UserWaypointStore s;
    s.next_id = kUserIdBase + 7;
    s.waypoints.push_back({kUserIdBase, 1.0, 2.0, 0.5, true, "room_0"});
    s.waypoints.push_back({kUserIdBase + 3, 3.0, 4.0, 0.0, false, "room_1"});

    UserWaypointStore t;
    t.from_json(s.to_json());
    ASSERT_EQ(t.waypoints.size(), 2u);
    EXPECT_EQ(t.next_id, kUserIdBase + 7);
    EXPECT_EQ(t.waypoints[0].id, kUserIdBase);
    EXPECT_DOUBLE_EQ(t.waypoints[0].x, 1.0);
    EXPECT_DOUBLE_EQ(t.waypoints[0].y, 2.0);
    EXPECT_DOUBLE_EQ(t.waypoints[0].yaw, 0.5);
    EXPECT_TRUE(t.waypoints[0].has_yaw);
    EXPECT_EQ(t.waypoints[0].place_id, "room_0");
    EXPECT_EQ(t.waypoints[1].id, kUserIdBase + 3);
    EXPECT_FALSE(t.waypoints[1].has_yaw);
    EXPECT_EQ(t.waypoints[1].place_id, "room_1");
}

TEST(UserWaypointStore, IdAllocationMonotonic) {
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 25, 25);
    auto map = make_free_map();
    UserWaypointStore s;

    auto r1 = s.apply_edit("add", 0, wc(10), wc(10), 0, false, gb.g, map);
    ASSERT_TRUE(r1.success);
    EXPECT_EQ(r1.id, kUserIdBase);
    auto r2 = s.apply_edit("add", 0, wc(12), wc(12), 0, false, gb.g, map);
    ASSERT_TRUE(r2.success);
    EXPECT_EQ(r2.id, kUserIdBase + 1);
    // Deleting never recycles ids.
    auto rd = s.apply_edit("delete", kUserIdBase, 0, 0, 0, false, gb.g, map);
    ASSERT_TRUE(rd.success);
    auto r3 = s.apply_edit("add", 0, wc(14), wc(14), 0, false, gb.g, map);
    ASSERT_TRUE(r3.success);
    EXPECT_EQ(r3.id, kUserIdBase + 2);
    EXPECT_EQ(s.waypoints.size(), 2u);
}

TEST(UserWaypointStore, RejectMoveDeleteOnNonUserIds) {
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 25, 25);
    auto map = make_free_map();
    UserWaypointStore s;

    // Auto id (< kFrontierIdBase): move rejected.
    auto m_auto = s.apply_edit("move", 42, wc(10), wc(10), 0, false, gb.g, map);
    EXPECT_FALSE(m_auto.success);
    EXPECT_EQ(m_auto.message, "only operator waypoints can be moved");
    // Frontier id (in [kFrontierIdBase, kUserIdBase)): delete rejected.
    auto d_frontier =
        s.apply_edit("delete", kFrontierIdBase + 1, 0, 0, 0, false, gb.g, map);
    EXPECT_FALSE(d_frontier.success);
    EXPECT_NE(d_frontier.message.find("relabel"), std::string::npos);
    // A user id that doesn't exist: move fails as not-found.
    auto m_missing =
        s.apply_edit("move", kUserIdBase, wc(10), wc(10), 0, false, gb.g, map);
    EXPECT_FALSE(m_missing.success);
    EXPECT_EQ(m_missing.message, "waypoint not found");
}

TEST(UserWaypointStore, HardRejectUnsafeAdd) {
    auto map = make_free_map();
    UserWaypointStore s;
    // Room hugging the border so an in-room cell can still be wall-adjacent.
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 1, 1, 15, 15);

    // Free cell outside any region resolves to no place: rejected.
    auto off = s.apply_edit("add", 0, wc(20), wc(20), 0, false, gb.g, map);
    EXPECT_FALSE(off.success);
    EXPECT_EQ(off.message, "point is in unknown/obstacle space");

    // In the room but within 0.375 m of the occupied border: clearance rejects.
    auto near = s.apply_edit("add", 0, wc(2), wc(2), 0, false, gb.g, map);
    EXPECT_FALSE(near.success);
    EXPECT_EQ(near.message, "too close to a wall/obstacle");

    // Interior with clearance: accepted, place_id set.
    auto ok = s.apply_edit("add", 0, wc(8), wc(8), 0, false, gb.g, map);
    ASSERT_TRUE(ok.success);
    ASSERT_EQ(s.waypoints.size(), 1u);
    EXPECT_EQ(s.waypoints[0].place_id, "room_0");
}

TEST(UserWaypointStore, MergeRefreshesLabelFromPlace) {
    auto map = make_free_map();
    UserWaypointStore s;
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 25, 25);
    ASSERT_TRUE(s.apply_edit("add", 0, wc(15), wc(15), 0, false, gb.g, map)
                    .success);

    std::vector<PublishedWaypoint> active, parked;
    s.merge_into(active, parked, gb.g, map);
    ASSERT_EQ(active.size(), 1u);
    EXPECT_TRUE(parked.empty());
    EXPECT_EQ(active[0].label, "room_0");  // falls back to place_id
    EXPECT_EQ(active[0].place_id, "room_0");
    EXPECT_EQ(active[0].source, "operator");

    // Operator relabels the owning region; merge picks up effective_label.
    gb.g.places[0].operator_label = "kitchen";
    gb.g.places[0].label_locked = true;
    active.clear();
    parked.clear();
    s.merge_into(active, parked, gb.g, map);
    ASSERT_EQ(active.size(), 1u);
    EXPECT_EQ(active[0].label, "kitchen");
}

TEST(UserWaypointStore, WithholdsParkedPinButKeepsIt) {
    auto map = make_free_map();
    UserWaypointStore s;
    GraphBuilder gb;
    gb.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 25, 25);
    ASSERT_TRUE(s.apply_edit("add", 0, wc(15), wc(15), 0, false, gb.g, map)
                    .success);

    // A refresh whose graph no longer covers the pin's cell: parked, not dropped.
    GraphBuilder shrunk;
    shrunk.add("room_0", tpg::PlaceKind::kRoom, 5, 5, 10, 10);  // excludes (15,15)
    std::vector<PublishedWaypoint> active, parked;
    s.merge_into(active, parked, shrunk.g, map);
    EXPECT_TRUE(active.empty());
    ASSERT_EQ(parked.size(), 1u);
    EXPECT_EQ(parked[0].id, kUserIdBase);
    EXPECT_EQ(s.waypoints.size(), 1u);  // still held + serialized

    // When the region grows back over the pin, it re-activates automatically.
    active.clear();
    parked.clear();
    s.merge_into(active, parked, gb.g, map);
    EXPECT_EQ(active.size(), 1u);
    EXPECT_TRUE(parked.empty());
}
