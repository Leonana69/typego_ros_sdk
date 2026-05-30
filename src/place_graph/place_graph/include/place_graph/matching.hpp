#pragma once

#include "place_graph/place_graph.hpp"

namespace place_graph {

struct MatchResult {
    PlaceGraphSnapshot snapshot;
    std::vector<PlaceEvent> events;
};

// Phase 6: stable-ID matching. Takes the previous published snapshot
// (may be empty) and a fresh snapshot returned by build_place_graph,
// and returns a new snapshot whose IDs survive across map updates.
// Pure: no mutation of inputs.
MatchResult match_place_graphs(const PlaceGraphSnapshot& prev,
                               const PlaceGraphSnapshot& fresh,
                               const Config& cfg);

}  // namespace place_graph
