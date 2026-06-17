#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <set>
#include <string>
#include <utility>

#include "place_graph/place_graph.hpp"
#include "place_graph/sampling.hpp"

namespace tpg = place_graph;

namespace {

// Hand-built snapshot builder. Unlike the GraphBuilder in test_waypoint_anchor,
// rect() merges into an existing place_id, so a single region can span several
// rectangles (needed for the U-shaped geodesic test).
struct TGraph {
    static constexpr int W = 40;
    static constexpr int H = 40;
    static constexpr double RES = 0.1;
    tpg::PlaceGraphSnapshot g;

    TGraph() {
        g.width = W;
        g.height = H;
        g.resolution_m = RES;
        g.origin_x = 0.0;
        g.origin_y = 0.0;
        g.place_assignment.assign(W * H, tpg::kNoPlace);
    }

    int ensure(const std::string& id, tpg::PlaceKind kind, double clearance) {
        for (std::size_t i = 0; i < g.places.size(); ++i)
            if (g.places[i].place_id == id) return static_cast<int>(i);
        tpg::PlaceRegion p;
        p.place_id = id;
        p.kind = kind;
        p.clearance_m = clearance;
        g.places.push_back(p);
        return static_cast<int>(g.places.size()) - 1;
    }

    // Add cells [x0,x1) x [y0,y1) to place `id`, creating it if needed.
    void rect(const std::string& id, tpg::PlaceKind kind, int x0, int y0,
              int x1, int y1, double clearance = 1.0) {
        int idx = ensure(id, kind, clearance);
        for (int y = y0; y < y1; ++y)
            for (int x = x0; x < x1; ++x) {
                g.place_assignment[y * W + x] = static_cast<std::uint16_t>(idx);
                g.places[idx].cell_region.push_back({x, y});
                g.places[idx].centroid.x = (x + 0.5) * RES;
                g.places[idx].centroid.y = (y + 0.5) * RES;
                g.places[idx].peak = g.places[idx].centroid;
            }
    }
};

// World coordinate of a cell center (origin 0).
double wc(int cell) { return (cell + 0.5) * TGraph::RES; }
// Integer cell owning a world coordinate (origin 0).
int cell_of(double w) { return static_cast<int>(w / TGraph::RES); }

}  // namespace

TEST(Sampling, PointInsideCurrentRegion) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 13, 13);
    std::mt19937 rng(12345);
    const double qx = wc(7), qy = wc(7), range = 0.5;

    for (int i = 0; i < 200; ++i) {
        auto out = tpg::sample_in_region(t.g, qx, qy, range, rng);
        ASSERT_TRUE(out.has_value());
        EXPECT_EQ(out->place_id, "room_a");
        const auto* p = t.g.place_at_world(out->point.x, out->point.y);
        ASSERT_NE(p, nullptr);
        EXPECT_EQ(p->place_id, "room_a");
        // Geodesic dist >= euclidean dist, so euclidean must be within range
        // (plus up to one cell of sub-cell jitter).
        double d = std::hypot(out->point.x - qx, out->point.y - qy);
        EXPECT_LE(d, range + std::sqrt(2.0) * TGraph::RES + 1e-9);
    }
}

TEST(Sampling, GeodesicStaysInRegionVsEuclidean) {
    // U-shape, one region: two tall arms joined only at the bottom; the middle
    // is a wall (kNoPlace). Robot sits at the top of the left arm.
    TGraph t;
    t.rect("room_u", tpg::PlaceKind::kRoom, 3, 3, 6, 25);    // left arm
    t.rect("room_u", tpg::PlaceKind::kRoom, 10, 3, 13, 25);  // right arm
    t.rect("room_u", tpg::PlaceKind::kRoom, 3, 3, 13, 6);    // bottom connector

    const double qx = wc(4), qy = wc(23), range = 1.0;
    ASSERT_EQ(t.g.place_at_world(qx, qy)->place_id, "room_u");

    // Geodesic: every sample is in-region AND geodesically reachable within
    // range, which from the top of the left arm can never reach the right arm
    // (that needs ~4 m of travel down-across-up). So x stays in the left arm.
    std::mt19937 rng_g(7);
    for (int i = 0; i < 500; ++i) {
        auto out = tpg::sample_in_region(t.g, qx, qy, range, rng_g,
                                         tpg::SampleMode::kGeodesic);
        ASSERT_TRUE(out.has_value());
        EXPECT_EQ(out->place_id, "room_u");
        int cx = cell_of(out->point.x);
        EXPECT_GE(cx, 3);
        EXPECT_LT(cx, 6) << "geodesic sample leaked out of the left arm";
    }

    // Euclidean: the right arm is within straight-line range, so over many
    // draws it WILL pick a point there (around the corner) -- the trade-off.
    std::mt19937 rng_e(7);
    bool reached_right_arm = false;
    for (int i = 0; i < 500; ++i) {
        auto out = tpg::sample_in_region(t.g, qx, qy, range, rng_e,
                                         tpg::SampleMode::kEuclidean);
        ASSERT_TRUE(out.has_value());
        EXPECT_EQ(out->place_id, "room_u");
        if (cell_of(out->point.x) >= 10) reached_right_arm = true;
    }
    EXPECT_TRUE(reached_right_arm)
        << "euclidean mode should reach the far arm across the gap";
}

TEST(Sampling, RangeLargerThanRegionStaysInRegion) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 8, 8);
    std::mt19937 rng(99);
    auto out = tpg::sample_in_region(t.g, wc(5), wc(5), 1e6, rng);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->place_id, "room_a");
    const auto* p = t.g.place_at_world(out->point.x, out->point.y);
    ASSERT_NE(p, nullptr);
    EXPECT_EQ(p->place_id, "room_a");
}

TEST(Sampling, RobotNotInAnyRegion) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 8, 8);
    std::mt19937 rng(1);
    // (1.5, 1.5) -> cell (15,15), outside the room -> kNoPlace.
    auto out = tpg::sample_in_region(t.g, 1.5, 1.5, 1.0, rng);
    EXPECT_FALSE(out.has_value());
}

TEST(Sampling, DeterminismWithFixedSeed) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 13, 13);
    const double qx = wc(7), qy = wc(7), range = 0.6;

    std::mt19937 a(42), b(42);
    auto ra = tpg::sample_in_region(t.g, qx, qy, range, a);
    auto rb = tpg::sample_in_region(t.g, qx, qy, range, b);
    ASSERT_TRUE(ra.has_value());
    ASSERT_TRUE(rb.has_value());
    EXPECT_EQ(ra->point.x, rb->point.x);
    EXPECT_EQ(ra->point.y, rb->point.y);
    EXPECT_EQ(ra->place_id, rb->place_id);

    // Different seeds should not all collapse to the same point.
    std::set<std::pair<double, double>> distinct;
    for (unsigned s = 1; s <= 8; ++s) {
        std::mt19937 rng(s);
        auto out = tpg::sample_in_region(t.g, qx, qy, range, rng);
        ASSERT_TRUE(out.has_value());
        distinct.insert({out->point.x, out->point.y});
    }
    EXPECT_GE(distinct.size(), 2u);
}

TEST(Sampling, RangeSmallerThanOneCell) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 13, 13);
    std::mt19937 rng(3);
    const double qx = wc(7), qy = wc(7);
    // Half a cell: only the robot's own cell qualifies (neighbors are 1 cell
    // = 0.1 m away, beyond range).
    auto out = tpg::sample_in_region(t.g, qx, qy, 0.5 * TGraph::RES, rng);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(cell_of(out->point.x), 7);
    EXPECT_EQ(cell_of(out->point.y), 7);
    EXPECT_EQ(out->place_id, "room_a");
}

TEST(Sampling, EmptySnapshot) {
    tpg::PlaceGraphSnapshot empty;  // width=height=0, resolution_m=0
    std::mt19937 rng(5);
    EXPECT_FALSE(tpg::sample_in_region(empty, 1.0, 1.0, 1.0, rng).has_value());
}

TEST(Sampling, RobotOnPortalCell) {
    // Two rooms separated by a 2-cell portal place; query lands on the portal.
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 9, 13);
    t.rect("portal_1", tpg::PlaceKind::kPortal, 9, 7, 11, 9);
    t.rect("room_b", tpg::PlaceKind::kRoom, 11, 3, 17, 13);
    std::mt19937 rng(8);
    // (wc(9), wc(7)) -> cell (9,7), owned by portal_1.
    ASSERT_EQ(t.g.place_at_world(wc(9), wc(7))->place_id, "portal_1");
    auto out = tpg::sample_in_region(t.g, wc(9), wc(7), 0.5, rng);
    ASSERT_TRUE(out.has_value());
    EXPECT_EQ(out->place_id, "portal_1");
}

TEST(Sampling, RegionClearanceGate) {
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 13, 13, /*clearance=*/1.0);
    std::mt19937 rng(2);
    const double qx = wc(7), qy = wc(7), range = 0.6;
    // Region clearance (1.0 m) below the floor -> reject.
    EXPECT_FALSE(
        tpg::sample_in_region(t.g, qx, qy, range, rng, tpg::SampleMode::kGeodesic,
                              /*min_clearance=*/2.0)
            .has_value());
    // Floor satisfied -> accept.
    EXPECT_TRUE(
        tpg::sample_in_region(t.g, qx, qy, range, rng, tpg::SampleMode::kGeodesic,
                              /*min_clearance=*/0.5)
            .has_value());
}

TEST(Sampling, MinClearanceKeepsAwayFromWalls) {
    // Solid room over cells [3,18) x [3,18); everything around it is kNoPlace
    // (walls). res = 0.1 m, so a 0.5 m margin is 5 cells. The only cells >= 0.5 m
    // from every wall are [7,13] x [7,13] (e.g. cell 7 is exactly 5 cells from
    // the wall column at x=2; cell 13 is 5 cells from the wall column at x=18).
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 18, 18, /*clearance=*/2.0);
    std::mt19937 rng(11);
    const double qx = wc(10), qy = wc(10), range = 5.0, margin = 0.5;
    for (int i = 0; i < 500; ++i) {
        auto out = tpg::sample_in_region(t.g, qx, qy, range, rng,
                                         tpg::SampleMode::kGeodesic, margin);
        ASSERT_TRUE(out.has_value());
        EXPECT_EQ(out->place_id, "room_a");
        int cx = cell_of(out->point.x), cy = cell_of(out->point.y);
        EXPECT_GE(cx, 7);
        EXPECT_LE(cx, 13);
        EXPECT_GE(cy, 7);
        EXPECT_LE(cy, 13);
    }
}

TEST(Sampling, MinClearanceTooTightForRegionYieldsNone) {
    // A 4-cell-wide room: no interior cell can be 0.5 m from both walls, so the
    // per-cell margin filter empties the candidate set even though the region's
    // (synthetic) clearance_m passes the fast-path gate.
    TGraph t;
    t.rect("room_a", tpg::PlaceKind::kRoom, 3, 3, 7, 7, /*clearance=*/1.0);
    std::mt19937 rng(4);
    auto out = tpg::sample_in_region(t.g, wc(4), wc(4), 5.0, rng,
                                     tpg::SampleMode::kGeodesic, 0.5);
    EXPECT_FALSE(out.has_value());
}
