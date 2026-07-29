#include <gtest/gtest.h>

#include <cmath>
#include <string>
#include <vector>

#include "place_graph/coverage.hpp"

namespace tpg = place_graph;

namespace {

constexpr double kRes = 0.1;

struct Scene {
    tpg::PlaceGraphSnapshot snap;
    tpg::MapSnapshot map;
    tpg::PlaceRegion region;
};

// Build a scene whose region is every cell for which `inside(x, y)` (world
// meters) holds. Everything else is an occupied wall.
template <typename F>
Scene make_scene(int w, int h, F inside) {
    Scene s;
    s.snap.width = w;
    s.snap.height = h;
    s.snap.resolution_m = kRes;
    s.snap.origin_x = 0.0;
    s.snap.origin_y = 0.0;
    s.snap.place_assignment.assign(static_cast<std::size_t>(w) * h,
                                   tpg::kNoPlace);
    s.map.width = w;
    s.map.height = h;
    s.map.resolution_m = kRes;
    s.map.origin_x = 0.0;
    s.map.origin_y = 0.0;
    s.map.cells.assign(static_cast<std::size_t>(w) * h,
                       tpg::CellState::kOccupied);

    s.region.place_id = "room_0";
    // kOpenArea, not kCorridor: a long hallway exceeds Config::a_open_m2 and is
    // classified open_area, so elongation is what has to catch it. Tests that
    // want corridor treatment raise `elongation` explicitly.
    s.region.kind = tpg::PlaceKind::kOpenArea;
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            if (!inside((x + 0.5) * kRes, (y + 0.5) * kRes)) continue;
            s.region.cell_region.push_back({x, y});
            s.snap.place_assignment[static_cast<std::size_t>(y) * w + x] = 0;
            s.map.cells[static_cast<std::size_t>(y) * w + x] =
                tpg::CellState::kFree;
        }
    }
    s.snap.places.push_back(s.region);
    return s;
}

// Worst geodesic-free proxy: straight-line distance from every region cell to
// the nearest chosen point. For the convex/rectilinear shapes used here it is a
// lower bound on the geodesic gap, so asserting on it is conservative.
double max_gap(const Scene& s, const std::vector<tpg::CoveragePoint>& pts) {
    double worst = 0.0;
    for (const auto& c : s.region.cell_region) {
        const double wx = (c.x + 0.5) * kRes;
        const double wy = (c.y + 0.5) * kRes;
        double best = 1e18;
        for (const auto& p : pts)
            best = std::min(best, std::hypot(wx - p.point.x, wy - p.point.y));
        worst = std::max(worst, best);
    }
    return worst;
}

double min_separation(const std::vector<tpg::CoveragePoint>& pts) {
    double best = 1e18;
    for (std::size_t i = 0; i < pts.size(); ++i)
        for (std::size_t j = i + 1; j < pts.size(); ++j)
            best = std::min(best, std::hypot(pts[i].point.x - pts[j].point.x,
                                             pts[i].point.y - pts[j].point.y));
    return pts.size() < 2 ? 0.0 : best;
}

}  // namespace

// The defining property: nothing reachable is left farther than `spacing_m`.
TEST(Coverage, BoundsTheGapInAStraightCorridor) {
    // 6.0 m x 1.0 m corridor, deliberately NOT aligned to a 2 m world lattice.
    Scene s = make_scene(80, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_clearance_m = 0.25;
    cfg.min_separation_m = 1.0;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_FALSE(pts.empty());
    EXPECT_LE(max_gap(s, pts), cfg.spacing_m);
    EXPECT_GE(min_separation(pts), cfg.min_separation_m - 1e-9);
}

// The case the lattice sampler could not handle: the L's cell-average centroid
// lies outside the region, so a centroid-directed recovery aims into the notch.
TEST(Coverage, CoversAnLBendIncludingTheElbow) {
    // Legs 5 m x 1 m meeting at a right angle.
    Scene s = make_scene(80, 80, [](double x, double y) {
        const bool leg_a = x >= 0.5 && x <= 5.5 && y >= 0.5 && y <= 1.5;
        const bool leg_b = x >= 4.5 && x <= 5.5 && y >= 0.5 && y <= 5.5;
        return leg_a || leg_b;
    });
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_clearance_m = 0.25;
    cfg.min_separation_m = 1.0;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_FALSE(pts.empty());
    EXPECT_LE(max_gap(s, pts), cfg.spacing_m);

    // The elbow itself must be covered, not just the two legs.
    double elbow_best = 1e18;
    for (const auto& p : pts)
        elbow_best =
            std::min(elbow_best, std::hypot(p.point.x - 5.0, p.point.y - 1.0));
    EXPECT_LE(elbow_best, cfg.spacing_m);
}

// Seeds represent coverage the caller already committed to. The sampler must
// fill only the gaps, which is what keeps anchors stable as a map grows.
TEST(Coverage, SeedsSuppressNearbyPointsAndFillOnlyGaps) {
    Scene s = make_scene(80, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_separation_m = 1.0;

    auto none = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    // Seed with the points the sampler would have chosen: nothing is left to do.
    std::vector<tpg::Point2d> seeds;
    for (const auto& p : none) seeds.push_back(p.point);
    auto again = tpg::sample_coverage(s.snap, s.map, s.region, seeds, cfg);
    EXPECT_TRUE(again.empty());

    // Seeding only one end still leaves the far end to be filled.
    auto partial = tpg::sample_coverage(s.snap, s.map, s.region,
                                        {{1.0, 1.4}}, cfg);
    EXPECT_FALSE(partial.empty());
    for (const auto& p : partial)
        EXPECT_GE(std::hypot(p.point.x - 1.0, p.point.y - 1.4),
                  cfg.spacing_m - 1e-9);
}

// A pose_ok predicate must be honoured, so the caller never has to reject a
// proposal (which would silently reopen a hole).
TEST(Coverage, RespectsThePoseOkPredicate) {
    Scene s = make_scene(80, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 1.5;
    cfg.min_separation_m = 0.5;

    // Forbid the middle third of the corridor entirely.
    auto ok = [](double x, double) { return x < 2.5 || x > 4.5; };
    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg, ok);
    ASSERT_FALSE(pts.empty());
    for (const auto& p : pts) EXPECT_TRUE(ok(p.point.x, p.point.y));
}

// Geodesic, not euclidean: a thin wall stub makes two cells that are 0.2 m
// apart in a straight line genuinely far apart to walk between.
TEST(Coverage, UsesGeodesicDistanceAroundAWallStub) {
    // A 4 m x 2.2 m room split down the middle by a stub that leaves a gap only
    // at the top, so the two halves are ~5 m apart geodesically.
    Scene s = make_scene(60, 40, [](double x, double y) {
        if (x < 0.5 || x > 4.5 || y < 0.5 || y > 2.7) return false;
        const bool in_stub = x > 2.4 && x < 2.6 && y < 2.2;
        return !in_stub;
    });
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_separation_m = 0.3;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_GE(pts.size(), 2u);
    // Both sides of the stub must get coverage; a euclidean sampler would have
    // considered one side's point as covering the other.
    bool left = false, right = false;
    for (const auto& p : pts) {
        if (p.point.y < 2.2 && p.point.x < 2.4) left = true;
        if (p.point.y < 2.2 && p.point.x > 2.6) right = true;
    }
    EXPECT_TRUE(left);
    EXPECT_TRUE(right);
}

// Same input must give the same output: the anchor registry keys coverage on
// the resulting pose, so a nondeterministic sampler would churn waypoint ids.
TEST(Coverage, IsDeterministic) {
    Scene s = make_scene(80, 80, [](double x, double y) {
        const bool leg_a = x >= 0.5 && x <= 5.5 && y >= 0.5 && y <= 1.5;
        const bool leg_b = x >= 4.5 && x <= 5.5 && y >= 0.5 && y <= 5.5;
        return leg_a || leg_b;
    });
    tpg::CoverageConfig cfg;
    auto a = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    auto b = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_EQ(a.size(), b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        EXPECT_DOUBLE_EQ(a[i].point.x, b[i].point.x);
        EXPECT_DOUBLE_EQ(a[i].point.y, b[i].point.y);
    }
}

// The point of the corridor path: waypoints land on the centerline, so driving
// the published list is a straight sweep rather than a zig-zag between walls.
TEST(Coverage, CentersPointsOnTheCorridorAxis) {
    // 6.0 m x 1.0 m corridor; centerline is y = 1.4.
    Scene s = make_scene(90, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    s.region.elongation = 6.0;
    s.region.clearance_m = 0.5;
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_separation_m = 1.0;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_GE(pts.size(), 3u);
    for (const auto& p : pts)
        EXPECT_NEAR(p.point.y, 1.4, 0.11) << "point off the corridor axis";
    // Still covers what it is supposed to.
    EXPECT_LE(max_gap(s, pts), cfg.spacing_m);
}

// The returned sequence must BE the patrol route: monotonically along the
// place, so the caller can drive it as-is.
TEST(Coverage, ReturnsPointsInTraversalOrder) {
    Scene s = make_scene(90, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    s.region.elongation = 6.0;
    s.region.clearance_m = 0.5;
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_GE(pts.size(), 3u);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        EXPECT_GT(pts[i].route_position_m, pts[i - 1].route_position_m);
        // A straight corridor along x: traversal order must also be x order.
        EXPECT_GT(pts[i].point.x, pts[i - 1].point.x);
    }
}

// Ordering has to follow the bend, which is why it is geodesic and not a PCA
// axis: sorting an L by x or y interleaves the two legs.
TEST(Coverage, OrdersAroundAnLBendWithoutBacktracking) {
    Scene s = make_scene(80, 80, [](double x, double y) {
        const bool leg_a = x >= 0.5 && x <= 5.5 && y >= 0.5 && y <= 1.5;
        const bool leg_b = x >= 4.5 && x <= 5.5 && y >= 0.5 && y <= 5.5;
        return leg_a || leg_b;
    });
    s.region.elongation = 9.0;
    s.region.clearance_m = 0.5;
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;

    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_GE(pts.size(), 4u);
    for (std::size_t i = 1; i < pts.size(); ++i) {
        EXPECT_GT(pts[i].route_position_m, pts[i - 1].route_position_m);
        // Consecutive hops stay local; a sort that interleaved the legs would
        // jump the width of the L here.
        EXPECT_LT(std::hypot(pts[i].point.x - pts[i - 1].point.x,
                             pts[i].point.y - pts[i - 1].point.y),
                  2.0 * cfg.spacing_m);
    }
}

// If the traversal direction flipped when the map grew, a patrol run would
// reverse mid-mission.
TEST(Coverage, TraversalDirectionSurvivesTheMapGrowing) {
    tpg::CoverageConfig cfg;
    auto build = [](double far_end) {
        Scene s = make_scene(120, 30, [far_end](double x, double y) {
            return x >= 0.7 && x <= far_end && y >= 0.9 && y <= 1.9;
        });
        s.region.elongation = 6.0;
        s.region.clearance_m = 0.5;
        return s;
    };
    Scene small = build(6.7);
    Scene grown = build(10.7);

    auto r1 = tpg::compute_place_route(small.snap, small.region, cfg);
    auto r2 = tpg::compute_place_route(grown.snap, grown.region, cfg);
    ASSERT_TRUE(r1.corridor_like);
    ASSERT_TRUE(r2.corridor_like);
    // The near end is unchanged, so the origin of the coordinate is unchanged
    // and every existing point keeps its along-value.
    EXPECT_NEAR(r1.terminal_a_world.x, r2.terminal_a_world.x, 0.11);
    EXPECT_LT(r1.length_m, r2.length_m);
}

// A blobby place has no meaningful end-to-end direction; it must be left in
// selection order rather than collapsed onto a spurious axis.
TEST(Coverage, BlobbyPlaceIsNotTreatedAsACorridor) {
    Scene s = make_scene(60, 60, [](double x, double y) {
        return x >= 0.5 && x <= 4.5 && y >= 0.5 && y <= 4.5;
    });
    s.region.elongation = 1.1;
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;

    auto route = tpg::compute_place_route(s.snap, s.region, cfg);
    EXPECT_FALSE(route.corridor_like);
    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    ASSERT_FALSE(pts.empty());
    // Still route-ordered -- ordering works on any shape; only the centerline
    // treatment is gated on being corridor-like.
    for (std::size_t i = 1; i < pts.size(); ++i)
        EXPECT_GT(pts[i].route_position_m, pts[i - 1].route_position_m);
    EXPECT_LE(max_gap(s, pts), cfg.spacing_m);
}

// Centering is only free while the corridor is narrow enough for a centerline
// point to reach its walls; past that the guard must disable it rather than
// leave the edges uncovered.
TEST(Coverage, WideCorridorDisablesCenteringToKeepCoverage) {
    // 4.0 m wide: half-width 2.0 > 0.866 * spacing, so centerline-only sampling
    // would strand the sides.
    Scene s = make_scene(120, 70, [](double x, double y) {
        return x >= 0.5 && x <= 10.5 && y >= 0.5 && y <= 4.5;
    });
    s.region.elongation = 4.0;
    s.region.clearance_m = 2.0;
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    cfg.min_separation_m = 1.0;

    auto route = tpg::compute_place_route(s.snap, s.region, cfg);
    EXPECT_TRUE(route.corridor_like);
    EXPECT_FALSE(route.center_on_axis);
    auto pts = tpg::sample_coverage(s.snap, s.map, s.region, {}, cfg);
    EXPECT_LE(max_gap(s, pts), cfg.spacing_m);
}

// On a place that LOOPS the geodesic diameter is not unique, so both sweep
// endpoints migrate together as the map grows and the world-order tie-break
// lands on the other arm: the published list comes back reversed mid-patrol.
// Pinning the previous terminal is what actually holds the direction.
TEST(Coverage, PinnedTerminalHoldsDirectionOnARing) {
    tpg::CoverageConfig cfg;
    cfg.spacing_m = 2.0;
    // Ring corridor ~1.0 m wide around a solid core, with a stub whose length
    // stands in for further exploration.
    auto build = [](double stub_len) {
        Scene s = make_scene(110, 110, [stub_len](double x, double y) {
            const double dx = x - 4.5, dy = y - 4.5;
            const double rr = std::hypot(dx, dy);
            const bool ring = rr >= 2.5 && rr <= 3.5;
            const bool stub =
                x >= 7.9 && x <= 7.9 + stub_len && y >= 4.0 && y <= 5.0;
            return ring || stub;
        });
        s.region.elongation = 8.0;
        s.region.clearance_m = 0.5;
        return s;
    };

    Scene before = build(0.4);
    Scene after = build(2.4);
    const auto r_before = tpg::compute_place_route(before.snap, before.region, cfg);
    ASSERT_TRUE(r_before.corridor_like);

    // Free re-derivation on the grown map may or may not agree; pinning must.
    const tpg::Point2d pin = r_before.terminal_a_world;
    const auto r_pinned =
        tpg::compute_place_route(after.snap, after.region, cfg, &pin);
    ASSERT_TRUE(r_pinned.corridor_like);
    EXPECT_DOUBLE_EQ(r_pinned.terminal_a_world.x, pin.x);
    EXPECT_DOUBLE_EQ(r_pinned.terminal_a_world.y, pin.y);

    // With the terminal held, a fixed set of waypoints keeps its order.
    std::vector<tpg::Point2d> pts;
    for (const auto& p :
         tpg::sample_coverage(before.snap, before.map, before.region, {}, cfg,
                              {}, &r_before))
        pts.push_back(p.point);
    ASSERT_GE(pts.size(), 4u);
    const auto ord_before =
        tpg::order_route(before.snap, before.region, pts, &r_before);
    const auto ord_pinned =
        tpg::order_route(after.snap, after.region, pts, &r_pinned);
    EXPECT_EQ(ord_before, ord_pinned);
}

// A pin that has drifted off the place (or onto a detached fragment) must be
// ignored rather than leaving the region unreachable.
TEST(Coverage, StalePinIsIgnored) {
    Scene s = make_scene(90, 30, [](double x, double y) {
        return x >= 0.7 && x <= 6.7 && y >= 0.9 && y <= 1.9;
    });
    s.region.elongation = 6.0;
    s.region.clearance_m = 0.5;
    tpg::CoverageConfig cfg;

    const tpg::Point2d off_map{50.0, 50.0};
    const auto pinned =
        tpg::compute_place_route(s.snap, s.region, cfg, &off_map);
    const auto fresh = tpg::compute_place_route(s.snap, s.region, cfg);
    ASSERT_TRUE(pinned.corridor_like);
    EXPECT_DOUBLE_EQ(pinned.terminal_a_world.x, fresh.terminal_a_world.x);
    EXPECT_DOUBLE_EQ(pinned.terminal_a_world.y, fresh.terminal_a_world.y);
    EXPECT_GT(pinned.length_m, 0.0);
}

TEST(Coverage, DegenerateInputsReturnEmpty) {
    Scene s = make_scene(40, 40, [](double x, double y) {
        return x >= 0.5 && x <= 2.5 && y >= 0.5 && y <= 2.5;
    });
    tpg::CoverageConfig cfg;

    tpg::PlaceRegion empty_region;
    empty_region.place_id = "room_0";
    EXPECT_TRUE(
        tpg::sample_coverage(s.snap, s.map, empty_region, {}, cfg).empty());

    tpg::CoverageConfig bad = cfg;
    bad.spacing_m = 0.0;
    EXPECT_TRUE(tpg::sample_coverage(s.snap, s.map, s.region, {}, bad).empty());

    // A region where every cell is too close to a wall yields nothing rather
    // than looping.
    tpg::CoverageConfig tight = cfg;
    tight.min_clearance_m = 5.0;
    EXPECT_TRUE(
        tpg::sample_coverage(s.snap, s.map, s.region, {}, tight).empty());
}
