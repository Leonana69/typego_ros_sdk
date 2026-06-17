// place_graph: random in-region sampling.
//
// Given a PlaceGraphSnapshot and a world query point (typically the robot's
// pose), sample a random reachable point inside the SAME place that owns the
// query point, within `range` meters of movement. Used by "wander around a
// room" behaviors: repeatedly sample + navigate.
//
// Pure, ROS-free, no globals, no I/O -- consistent with the rest of the
// place_graph library. The RNG is caller-owned so callers (and tests) control
// determinism.

#pragma once

#include <optional>
#include <random>
#include <string>

#include "place_graph/types.hpp"

namespace place_graph {

struct SampleResult {
    Point2d point;         // sampled point, world meters
    std::string place_id;  // place that owns the point (== place at the query)
};

enum class SampleMode {
    // DEFAULT. Flood the query region's own cells outward from the query cell
    // (8-connected, no corner-cutting) and keep cells whose in-region travel
    // distance is <= range. The path never leaves the region, so every sample
    // is reachable without exiting the room -- correct for concave (L/U) rooms.
    kGeodesic,
    // Straight-line filter over the region's cells. Cheaper, but in a concave
    // room can pick a point that is within `range` as the crow flies yet lies
    // around a corner. Retained for cheap callers and the contrast unit test.
    kEuclidean,
};

// Sample a random point inside the place that owns (wx, wy), within `range_m`.
// Returns std::nullopt when: the snapshot is degenerate, the query point is not
// owned by any place, the region is empty, or no cell within range also clears
// the requested wall margin.
//
// `min_clearance_m` is a per-point margin: the sampled point is kept at least
// this far (meters) from the nearest wall / obstacle / unknown cell. Doorways
// to adjacent rooms do not count as obstacles. Pass <= 0 to disable.
std::optional<SampleResult> sample_in_region(
    const PlaceGraphSnapshot& snap, double wx, double wy, double range_m,
    std::mt19937& rng, SampleMode mode = SampleMode::kGeodesic,
    double min_clearance_m = 0.0);

}  // namespace place_graph
