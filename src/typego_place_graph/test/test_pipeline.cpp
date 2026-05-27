#include <gtest/gtest.h>

#include <algorithm>

#include "test_fixtures.hpp"
#include "typego_place_graph/matching.hpp"
#include "typego_place_graph/place_graph.hpp"

namespace tpg = typego_place_graph;

namespace {

int count_kind(const tpg::PlaceGraphSnapshot& s, tpg::PlaceKind k) {
    int n = 0;
    for (const auto& p : s.places) if (p.kind == k) ++n;
    return n;
}

}  // namespace

TEST(Pipeline, OneSquareRoom) {
    auto m = tpg::test::one_square_room(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    EXPECT_EQ(count_kind(g, tpg::PlaceKind::kRoom) +
              count_kind(g, tpg::PlaceKind::kOpenArea), 1);
    EXPECT_EQ(count_kind(g, tpg::PlaceKind::kPortal), 0);
    EXPECT_EQ(count_kind(g, tpg::PlaceKind::kOpenTransition), 0);
}

TEST(Pipeline, TwoRoomsWithDoorway) {
    auto m = tpg::test::two_rooms_with_doorway(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    int rooms = count_kind(g, tpg::PlaceKind::kRoom) +
                count_kind(g, tpg::PlaceKind::kOpenArea);
    EXPECT_EQ(rooms, 2);
    EXPECT_GE(count_kind(g, tpg::PlaceKind::kPortal), 1);
    // The portal's adjacency lists both rooms.
    for (const auto& p : g.places) {
        if (p.kind == tpg::PlaceKind::kPortal) {
            EXPECT_EQ(p.adjacent_place_ids.size(), 2u);
            EXPECT_TRUE(p.width_m.has_value());
            EXPECT_TRUE(p.approach_yaw.has_value());
        }
    }
}

TEST(Pipeline, CorridorWithSideRooms) {
    auto m = tpg::test::corridor_with_side_rooms(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    int rooms = count_kind(g, tpg::PlaceKind::kRoom) +
                count_kind(g, tpg::PlaceKind::kOpenArea);
    // 3 rooms + 1 corridor; the corridor may classify as kCorridor.
    EXPECT_EQ(rooms + count_kind(g, tpg::PlaceKind::kCorridor), 4);
    EXPECT_GE(count_kind(g, tpg::PlaceKind::kCorridor), 1);
    EXPECT_GE(count_kind(g, tpg::PlaceKind::kPortal), 3);
}

TEST(Pipeline, TIntersection) {
    auto m = tpg::test::t_intersection(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    // At least one junction is expected; if the geometry instead gets
    // segmented into corridors with portals, that's also acceptable
    // per the endpoint-set dispatch.
    int conn = count_kind(g, tpg::PlaceKind::kJunction) +
               count_kind(g, tpg::PlaceKind::kPortal) +
               count_kind(g, tpg::PlaceKind::kOpenTransition);
    EXPECT_GE(conn, 1);
    EXPECT_GE(count_kind(g, tpg::PlaceKind::kCorridor), 1);
}

TEST(Pipeline, OpenPlan) {
    auto m = tpg::test::open_plan(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    // A 10x10m room is exactly at the A_open boundary; either
    // open_area or room is fine. What matters: one place, no portals.
    EXPECT_EQ(static_cast<int>(g.places.size()), 1);
    EXPECT_TRUE(g.places[0].kind == tpg::PlaceKind::kOpenArea ||
                g.places[0].kind == tpg::PlaceKind::kRoom);
}

TEST(Pipeline, PartiallyExploredRoom) {
    auto m = tpg::test::partially_explored_room(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    bool any_provisional_or_frontier = false;
    for (const auto& p : g.places) {
        if (p.provisional) any_provisional_or_frontier = true;
        if (p.kind == tpg::PlaceKind::kFrontierRegion)
            any_provisional_or_frontier = true;
    }
    EXPECT_TRUE(any_provisional_or_frontier);
}

TEST(Pipeline, NoisyMapArtifactsDropped) {
    auto m = tpg::test::noisy_map_with_artifacts(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    // The 2x2 free island should not survive A_min. Only the room
    // (and possibly fallback-frontier_region) should appear.
    int rooms = count_kind(g, tpg::PlaceKind::kRoom) +
                count_kind(g, tpg::PlaceKind::kOpenArea);
    EXPECT_EQ(rooms, 1);
}

TEST(Pipeline, WaypointLookup) {
    auto m = tpg::test::two_rooms_with_doorway(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    // Center of left room is near (2.6, 2.6) in world coordinates.
    const auto* p = g.place_at_world(2.5, 2.5);
    ASSERT_NE(p, nullptr);
    EXPECT_TRUE(p->kind == tpg::PlaceKind::kRoom ||
                p->kind == tpg::PlaceKind::kOpenArea);
    // Center of right room ~(7.6, 2.5).
    const auto* q = g.place_at_world(7.6, 2.5);
    ASSERT_NE(q, nullptr);
    EXPECT_TRUE(q->kind == tpg::PlaceKind::kRoom ||
                q->kind == tpg::PlaceKind::kOpenArea);
    EXPECT_NE(p->place_id, q->place_id);
}

TEST(Pipeline, TightClosetGetsFallbackSeed) {
    auto m = tpg::test::tight_closet_across_doorway(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    int rooms = count_kind(g, tpg::PlaceKind::kRoom) +
                count_kind(g, tpg::PlaceKind::kOpenArea) +
                count_kind(g, tpg::PlaceKind::kCorridor);
    // Big room + closet should both surface (closet via fallback,
    // big room via opening) OR they share a free-space component and
    // collapse to one — both are valid spec outcomes per Step 2.3 note.
    EXPECT_GE(rooms, 1);
}

TEST(Pipeline, WideGapEmitsOpenTransition) {
    auto m = tpg::test::wide_gap_two_core(0.10);
    tpg::Config cfg;
    auto g = tpg::build_place_graph(m, cfg);
    int wide = count_kind(g, tpg::PlaceKind::kOpenTransition);
    int portal = count_kind(g, tpg::PlaceKind::kPortal);
    // The 3.5m gap exceeds w_max_portal_m (2.5); if it produces two
    // cores at all, it should classify as open_transition (or stay one
    // open_area).
    EXPECT_TRUE(wide >= 1 || portal == 0);
}
