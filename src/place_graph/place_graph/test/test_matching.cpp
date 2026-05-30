#include <gtest/gtest.h>

#include "test_fixtures.hpp"
#include "place_graph/matching.hpp"
#include "place_graph/place_graph.hpp"

namespace tpg = place_graph;

namespace {

const tpg::PlaceRegion* find_room_around(const tpg::PlaceGraphSnapshot& g,
                                         double wx, double wy) {
    return g.place_at_world(wx, wy);
}

}  // namespace

TEST(Matching, RoomIdSurvivesIncrementalDetail) {
    auto m1 = tpg::test::two_rooms_with_doorway(0.10);
    tpg::Config cfg;
    auto g1 = tpg::build_place_graph(m1, cfg);
    // Match against an empty prev for the initial cold-start.
    tpg::PlaceGraphSnapshot empty;
    auto mr1 = tpg::match_place_graphs(empty, g1, cfg);

    // Refine: add a small obstacle inside the left room. IDs should
    // survive.
    auto m2 = m1;
    tpg::test::fill_rect(m2, 10, 10, 14, 14, tpg::CellState::kOccupied);
    auto g2 = tpg::build_place_graph(m2, cfg);
    auto mr2 = tpg::match_place_graphs(mr1.snapshot, g2, cfg);

    const auto* left1 = find_room_around(mr1.snapshot, 2.5, 2.5);
    const auto* left2 = find_room_around(mr2.snapshot, 2.5, 2.5);
    ASSERT_NE(left1, nullptr);
    ASSERT_NE(left2, nullptr);
    EXPECT_EQ(left1->place_id, left2->place_id);
}

TEST(Matching, AddedWallSplitsRoomLargestKeepsId) {
    // Start with a single big rectangular room. Then add an internal
    // wall that splits it; the larger half should keep the old ID.
    int w = 142, h = 52;
    auto m1 = tpg::test::make_blank(w, h, 0.10, tpg::CellState::kFree);
    tpg::test::rect_walls(m1, 0, 0, w, h);
    tpg::Config cfg;
    auto g1 = tpg::build_place_graph(m1, cfg);
    auto mr1 = tpg::match_place_graphs({}, g1, cfg);
    ASSERT_GE(static_cast<int>(mr1.snapshot.places.size()), 1);
    std::string original_id = mr1.snapshot.places[0].place_id;

    // Add a wall at x=40 with a 0.6 m doorway in the middle.
    auto m2 = m1;
    for (int y = 1; y < h - 1; ++y) {
        if (y >= 23 && y < 29) continue;
        m2.cells[y * w + 40] = tpg::CellState::kOccupied;
    }
    auto g2 = tpg::build_place_graph(m2, cfg);
    auto mr2 = tpg::match_place_graphs(mr1.snapshot, g2, cfg);

    // Find which fresh place is the larger half (right side, ~ 100 *
    // 50 free cells vs ~ 40 * 50).
    const tpg::PlaceRegion* right = find_room_around(mr2.snapshot, 9.0, 2.5);
    ASSERT_NE(right, nullptr);
    EXPECT_EQ(right->place_id, original_id);

    bool split_seen = false;
    for (const auto& e : mr2.events) {
        if (e.kind == tpg::PlaceEventKind::kSplit) split_seen = true;
    }
    EXPECT_TRUE(split_seen);
}

TEST(Matching, ConnectorIdSurvivesAcrossRefresh) {
    auto m1 = tpg::test::two_rooms_with_doorway(0.10);
    tpg::Config cfg;
    auto g1 = tpg::build_place_graph(m1, cfg);
    auto mr1 = tpg::match_place_graphs({}, g1, cfg);
    std::string portal_id;
    for (const auto& p : mr1.snapshot.places) {
        if (p.kind == tpg::PlaceKind::kPortal) { portal_id = p.place_id; break; }
    }
    ASSERT_FALSE(portal_id.empty());

    // Now slightly modify the map (some interior detail) and refresh.
    auto m2 = m1;
    tpg::test::fill_rect(m2, 80, 10, 84, 14, tpg::CellState::kOccupied);
    auto g2 = tpg::build_place_graph(m2, cfg);
    auto mr2 = tpg::match_place_graphs(mr1.snapshot, g2, cfg);
    std::string new_portal_id;
    for (const auto& p : mr2.snapshot.places) {
        if (p.kind == tpg::PlaceKind::kPortal) { new_portal_id = p.place_id; break; }
    }
    EXPECT_EQ(portal_id, new_portal_id);
}
