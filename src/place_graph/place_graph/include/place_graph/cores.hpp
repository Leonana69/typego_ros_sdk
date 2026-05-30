#pragma once

#include "place_graph/preprocessing.hpp"

namespace place_graph {

// Phase 2 output. A `Core` is either:
//   - a survivor of morphological opening (`source = kOpened`)
//   - a fallback seed inside an unopened free-space component that
//     had no opened core (`source = kFallback`)
//   - a marker for a frontier_region (no real core; the whole
//     component is treated as an exploration target; `source =
//     kFrontierRegion`)
//
// Each Core carries enough geometry to be used as a watershed seed and
// to classify the eventual room as room / corridor / open_area in
// Phase 4 of the spec (here folded into Phase 2.4 just so the
// PlaceRegion has its final kind before watershed runs).
struct Core {
    enum class Source : std::uint8_t {
        kOpened,
        kFallback,
        kFrontierRegion,
    };

    Source source = Source::kOpened;
    PlaceKind kind = PlaceKind::kRoom;
    int seed_x = 0;
    int seed_y = 0;
    double area_m2 = 0.0;
    double clearance_m = 0.0;
    // All cells of the opening survivor for this core (or the single
    // fallback seed for kFallback). The watershed seeds the BFS from
    // every cell in this set at distance 0; using a single seed per
    // core gives elongated cores (corridors, open_areas) an unfair
    // disadvantage against any neighbor that reaches the boundary
    // through a shorter geodesic.
    std::vector<CellCoord> seed_cells;
    // For frontier_region we keep the full set of component cells so
    // Step 3's watershed can paint the whole component to this place
    // without competing with real room cores (those don't enter the
    // BFS for this component at all).
    std::vector<CellCoord> frontier_region_cells;
};

struct CoresResult {
    std::vector<Core> cores;
    // For each free cell of the segmentation grid, the index of the
    // free-space component (in the *unopened* mask) it belongs to.
    // -1 for non-free cells. Used by Step 3 to scope frontier_region
    // components.
    std::vector<int> component_of;
    // Distance transform in cells, used downstream by the connector
    // step. Stored radius-to-nearest-obstacle.
    std::vector<float> distance_cells;
};

CoresResult extract_cores(const SegmentationGrid& seg, const Config& cfg);

}  // namespace place_graph
