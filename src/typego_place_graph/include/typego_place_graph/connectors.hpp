#pragma once

#include "typego_place_graph/watershed.hpp"

namespace typego_place_graph {

// A materialized connector before stable IDs are minted. The pipeline
// turns each of these into a PlaceRegion in the final snapshot.
struct ConnectorCandidate {
    PlaceKind kind = PlaceKind::kPortal;
    // Endpoint core indices (into CoresResult::cores).
    std::vector<int> endpoint_cores;
    std::vector<CellCoord> cells;
    Point2d centroid;
    double width_m = 0.0;
    double clearance_m = 0.0;
    // Principal axis of the boundary band, in radians (atan2-style,
    // x-axis = 0). For portals/open_transitions this is the centerline
    // direction; approach_yaw is this + pi/2, then flipped to point
    // toward the larger adjacent room (orientation handled in
    // place_graph.cpp where centroids are available). For junctions
    // axis_angle has no useful meaning and approach_yaw is left unset.
    double axis_angle = 0.0;
};

std::vector<ConnectorCandidate> extract_connectors(
    const SegmentationGrid& seg,
    const CoresResult& cores,
    const WatershedResult& ws,
    const Config& cfg);

}  // namespace typego_place_graph
