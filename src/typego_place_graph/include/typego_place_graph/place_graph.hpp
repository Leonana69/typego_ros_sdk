#pragma once

#include "typego_place_graph/connectors.hpp"

namespace typego_place_graph {

// Phases 1-5: pure pipeline from a MapSnapshot to a place graph with
// fresh (non-stable) IDs. To preserve IDs across refreshes call
// `match_place_graphs` on (prev, current) afterward (see matching.hpp).
PlaceGraphSnapshot build_place_graph(const MapSnapshot& map,
                                     const Config& cfg);

}  // namespace typego_place_graph
