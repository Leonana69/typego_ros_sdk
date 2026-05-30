#pragma once

#include "place_graph/cores.hpp"

namespace place_graph {

// Phase 3 output. `core_assignment[i] == -1` for non-free cells, else
// an index into `cores` (0-based, matching the input order). `boundary`
// is 1 where at least one 4-neighbor has a different core_id.
//
// For frontier_region cores, the watershed BFS does not enter the
// containing free-space component; we paint the whole component to
// that core directly. This way the frontier_region's geometry is
// exactly the free-space pocket that was too unsettled to commit to a
// room, and a real room core in some other component doesn't fight
// over those cells.
struct WatershedResult {
    std::vector<int> core_assignment;
    std::vector<std::uint8_t> boundary;
};

WatershedResult watershed_assign(const SegmentationGrid& seg,
                                 const CoresResult& cores);

}  // namespace place_graph
