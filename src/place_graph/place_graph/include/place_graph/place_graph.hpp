#pragma once

#include "place_graph/connectors.hpp"

namespace place_graph {

// Phases 1-5: pure pipeline from a MapSnapshot to a place graph with
// fresh (non-stable) IDs. To preserve IDs across refreshes call
// `match_place_graphs` on (prev, current) afterward (see matching.hpp).
PlaceGraphSnapshot build_place_graph(const MapSnapshot& map,
                                     const Config& cfg);

}  // namespace place_graph
