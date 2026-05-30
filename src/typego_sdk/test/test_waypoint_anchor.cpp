#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "place_graph/place_graph.hpp"
#include "typego_sdk/waypoint_anchor.hpp"

namespace tpg = place_graph;
using typego_sdk::AnchorRegistry;
using typego_sdk::Candidate;
using typego_sdk::PublishedWaypoint;

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
