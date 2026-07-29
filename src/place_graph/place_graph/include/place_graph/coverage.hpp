// place_graph: geodesic coverage sampling.
//
// Choose points inside a place so that EVERY reachable cell of that place lies
// within `spacing_m` of in-region travel from some point. This is a
// measurement-driven sampler rather than a placement heuristic: each step finds
// the worst-covered cell and puts the next point there, so the bound on the
// coverage hole holds by construction. When the loop exits, no reachable cell
// is farther than `spacing_m` from a point -- that is the postcondition.
//
// Contrast with sampling a fixed world lattice, which controls where points ARE
// and says nothing about what it drops: on a corridor narrower than the lattice
// pitch and not aligned to it, whole rows of nodes fall outside the region and
// the resulting hole spans the full corridor width.
//
// Distances are geodesic (8-connected, no corner-cutting, never leaving the
// region), matching sampling.cpp. Straight-line distance is wrong here: it cuts
// the corner at a 90-degree bend, and calls two cells either side of a thin wall
// stub "close" when the robot has to walk around.
//
// Pure, ROS-free, no globals, no I/O -- consistent with the rest of the
// place_graph library.

#pragma once

#include <functional>
#include <vector>

#include "place_graph/types.hpp"

namespace place_graph {

struct CoveragePoint {
    Point2d point;             // cell center, world meters
    CellCoord cell;            // cell in the snapshot grid
    double clearance_m = 0.0;  // distance to the nearest wall / unknown cell
    // Distance walked along the route to reach this point from its start.
    // Points are returned in route order, so the sequence IS a patrol: drive it
    // as given and you sweep the place.
    double route_position_m = -1.0;
};

struct CoverageConfig {
    // Bound on the geodesic distance from any region cell to its nearest
    // point. The sampler stops once nothing exceeds it.
    double spacing_m = 2.0;
    // Fallback wall-clearance floor, used only when no `pose_ok` predicate is
    // supplied. A cell passes iff the nearest blocked cell is strictly farther
    // than this.
    double min_clearance_m = 0.25;
    // After choosing the worst-covered cell, shift up to this far onto the
    // local clearance ridge so the point is not left hard against a wall.
    double ridge_nudge_m = 0.3;
    // Euclidean floor between chosen points. Required *in addition to*
    // spacing_m: two cells can be geodesically far yet euclidean-close across a
    // thin wall stub, which reads as a cluster even though coverage is correct.
    double min_separation_m = 1.0;
    // A place counts as corridor-like at or above this elongation (or when its
    // kind is kCorridor). Mirrors Config::corridor_aspect. Note that a long
    // hallway often classifies as kOpenArea once its area exceeds
    // Config::a_open_m2, so the elongation test -- not the kind -- is usually
    // what catches it.
    double corridor_aspect = 3.0;
    // Half-thickness of the "same cross-section" band used for lateral
    // centering: cells whose along-corridor coordinate is within this much of
    // the chosen point's are treated as the same slice through the corridor.
    double cross_section_tol_m = 0.15;
    // Safety valve so a pathological region cannot spin.
    int max_points = 512;
};

// A 1-D traversal coordinate for an elongated place: the two endpoints of its
// geodesic diameter, and the in-region distance from the first of them to every
// cell. Sorting points by `along` orders them end to end and follows bends,
// which a PCA axis cannot do.
struct PlaceRoute {
    // False when the place is too blobby for a traversal to mean anything; the
    // caller should then leave points in selection order.
    bool corridor_like = false;
    // True when the place is narrow enough that points on its centerline still
    // cover its full width, so lateral centering costs no coverage.
    bool center_on_axis = false;

    CellCoord terminal_a{};
    CellCoord terminal_b{};
    Point2d terminal_a_world{};
    Point2d terminal_b_world{};
    double length_m = 0.0;  // geodesic distance terminal_a -> terminal_b

    // Geodesic distance from terminal_a for every cell of the snapshot grid;
    // infinity outside the region, and in any component unreachable from it.
    std::vector<double> along;

    // Along-coordinate at a world point, or infinity when it is not in the
    // region (or the route is invalid).
    double along_at(const PlaceGraphSnapshot& snap, double wx, double wy) const;
};

// Compute the traversal coordinate for `region`. Returns a route with
// corridor_like == false (and an empty field) when the place is not elongated
// enough to have a meaningful end-to-end direction.
//
// `preferred_start` pins terminal_a. Pass the terminal from a previous refresh
// to hold the traversal direction as the map grows: without it the double sweep
// picks whichever endpoint the current geometry gives, and on a place that
// loops -- where the geodesic diameter is not unique -- both endpoints migrate
// together, so the route can come back reversed. Ignored when it does not
// resolve into the region, or when starting there would leave most of the place
// unreachable (i.e. it has fallen onto a detached fragment); either way the
// caller should then persist the freshly swept terminal_a_world in its place.
PlaceRoute compute_place_route(const PlaceGraphSnapshot& snap,
                               const PlaceRegion& region,
                               const CoverageConfig& cfg,
                               const Point2d* preferred_start = nullptr);

// Order in-region points into a route the caller can drive as a plain list.
//
// Sorting by distance-from-one-end only works when the place is a simple path.
// Real hallways are not: a captured 64 m2 "hallway" here is a network of
// parallel corridors with cross-connections and stubs, and sorting that by
// distance from one end interleaves the branches, so consecutive entries in the
// list teleport between them.
//
// Instead this builds a minimum spanning tree over the GEODESIC distances
// between the points and returns its DFS preorder. On a simple corridor the
// tree is a path and the result is the end-to-end order; on a branching network
// it walks one branch at a time, which is what a patrol should do. Rooted at
// the place's first terminal when `route` says it has one, so a corridor is
// walked end to end rather than started from the middle.
//
// Returns indices into `points`. When `route_position_m` is non-null it is
// filled (indexed like `points`) with the distance walked to reach each point.
// Points outside the region cannot be routed; they keep their relative order
// and are appended at the end with position -1.
std::vector<std::size_t> order_route(
    const PlaceGraphSnapshot& snap, const PlaceRegion& region,
    const std::vector<Point2d>& points, const PlaceRoute* route = nullptr,
    std::vector<double>* route_position_m = nullptr);

// Sample coverage points inside `region`.
//
// `seeds` are poses that already provide coverage (entrances, doorways, and any
// coverage anchors the caller has already committed to). They are NOT returned;
// they simply start the distance field, so the sampler fills genuine gaps
// instead of re-deriving the whole set. Seeds outside the region are ignored.
//
// `pose_ok` decides whether a cell center may host a point. Pass the caller's
// own pose validation so the sampler never proposes something the caller will
// reject; it is evaluated once per region cell, not per iteration. When empty,
// `cfg.min_clearance_m` is used instead.
//
// `route` is the place's traversal coordinate. Pass one you already computed to
// avoid recomputing it; pass nullptr to have it computed internally. When the
// route says the place is corridor-like the returned points are sorted along
// it, and when it also says center_on_axis they are centered across the
// corridor width instead of being dispersed toward the walls.
//
// Returns points sorted by route_position_m for a corridor-like place, and
// otherwise in the order they were chosen (worst-covered first). Either way the
// result is deterministic for a given input.
std::vector<CoveragePoint> sample_coverage(
    const PlaceGraphSnapshot& snap, const MapSnapshot& map,
    const PlaceRegion& region, const std::vector<Point2d>& seeds,
    const CoverageConfig& cfg,
    const std::function<bool(double, double)>& pose_ok = {},
    const PlaceRoute* route = nullptr);

}  // namespace place_graph
