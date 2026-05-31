// place_graph: geometry-only place graph for room-aware semantic
// waypoints. Implements Phases 1-6 of doc/waypoint_group_v1.md.
//
// This header defines the immutable data types that flow through the
// pipeline. The pipeline itself is a pure function:
//
//   MapSnapshot -> PlaceGraphSnapshot
//
// No ROS includes, no globals, no I/O. The package is split out from
// typego_sdk so the segmentation pipeline is independently testable
// and a future ROS node can wire occupancy-grid messages into
// MapSnapshot without dragging the rest of typego_sdk in.

#pragma once

#include <array>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace place_graph {

// Occupancy values. The pipeline does not care about the ROS sign
// conventions; the caller maps its grid into these three states.
enum class CellState : std::uint8_t {
    kFree = 0,
    kOccupied = 1,
    kUnknown = 2,
};

// Input snapshot. Owned by the caller for the duration of the call;
// the pipeline does not retain references.
struct MapSnapshot {
    int width = 0;
    int height = 0;
    double resolution_m = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;
    // Row-major, length width * height.
    std::vector<CellState> cells;

    // Caller-supplied identifier; carried into PlaceGraphSnapshot for
    // debugging and persistence keying.
    std::string map_version;

    int index(int x, int y) const { return y * width + x; }
    bool in_bounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
    CellState at(int x, int y) const { return cells[index(x, y)]; }
};

// Sentinel value used in the cached uint16 place_assignment grid for
// "no place at this cell" (e.g., occupied / unknown / outside any
// region). Any place_id_index >= 0 indexes into PlaceGraphSnapshot::
// places.
constexpr std::uint16_t kNoPlace = std::numeric_limits<std::uint16_t>::max();

enum class PlaceKind : std::uint8_t {
    kRoom = 0,
    kCorridor = 1,
    kOpenArea = 2,
    kPortal = 3,
    kOpenTransition = 4,
    kJunction = 5,
    kFrontierRegion = 6,
};

const char* to_string(PlaceKind kind);

struct CellCoord {
    int x = 0;
    int y = 0;
    bool operator==(const CellCoord& other) const {
        return x == other.x && y == other.y;
    }
};

// 2D point in map coordinates (meters).
struct Point2d {
    double x = 0.0;
    double y = 0.0;
};

struct PlaceRegion {
    std::string place_id;
    PlaceKind kind = PlaceKind::kRoom;

    // Member cells. For non-connector places these are the watershed-
    // assigned cells minus connector cells (spec Step 4.5). For
    // connector places these are the materialized centerline cells.
    std::vector<CellCoord> cell_region;

    Point2d centroid;
    // Most-interior point (distance-transform maximum) in world meters. A
    // better navigation target than centroid for concave/L-shaped places,
    // whose cell-average centroid can fall outside the region. Defaults to
    // centroid when no distance-transform seed is available.
    Point2d peak;
    double area_m2 = 0.0;
    double clearance_m = 0.0;
    double frontier_ratio = 0.0;
    // Rotation-invariant shape descriptors of the final cell_region, used to
    // distinguish elongated regions (corridors/hallways) from compact ones
    // (rooms) for both kind classification and the VLM labeling hint.
    // elongation = max(PCA major/minor axis ratio, ribbon area/(4*clearance^2)),
    // clamped to [1, 20]; 1.0 = square/round. extent_long/short_m are the
    // human-readable length/width derived from the clearance ribbon model.
    double elongation = 1.0;
    double extent_long_m = 0.0;
    double extent_short_m = 0.0;
    bool provisional = false;

    // For connectors: enumerates the endpoint place_ids. For non-
    // connectors: empty (adjacency is derived from the connector
    // graph, per the spec).
    std::vector<std::string> adjacent_place_ids;

    // For portals / open_transitions only.
    std::optional<double> width_m;
    std::optional<double> approach_yaw;

    // Semantic fields. Geometry-only v1 leaves these at defaults; the
    // later semantic phase fills them in. Matching propagates them
    // across map updates so operator/VLM labels survive refreshes.
    std::string semantic_label;
    std::string instance_label;
    std::string operator_label;
    bool label_locked = false;
    double confidence = 0.0;
    std::string source = "map";
    // Richer VLM semantics (geometry-only v1 leaves these empty). Like the
    // label fields above, matching propagates them across map updates.
    std::vector<std::string> objects;
    std::vector<std::string> affordances;
    std::string summary;

    // Matching bookkeeping (Phase 6).
    int age_refreshes = 0;
    bool stale = false;
    int stale_refresh_count = 0;
};

struct Config {
    double seg_resolution_m = 0.10;
    int seg_inflation_cells = 1;
    double unknown_close_m = 0.30;
    double r_open_m = 0.40;
    double a_min_m2 = 1.5;
    double a_open_m2 = 30.0;
    double corridor_aspect = 3.0;
    double w_max_portal_m = 2.5;
    double iou_match = 0.4;
    double kind_penalty = 0.1;
    double connector_centroid_match_m = 0.75;
    double connector_width_match_m = 0.75;
    int stale_refresh_count = 3;
    double frontier_lock_ratio = 0.2;

    // Threshold for classifying a free-space component as a
    // frontier_region in Step 2.3. The spec text fixes this at 0.5;
    // exposed here so deployments can tune without recompiling.
    double frontier_region_ratio = 0.5;
};

struct PlaceGraphSnapshot {
    // Same geometry as the (downsampled) segmentation grid.
    int width = 0;
    int height = 0;
    double resolution_m = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;

    std::vector<PlaceRegion> places;

    // Cached uint16 lookup grid. Stores index into `places`, or
    // kNoPlace if the cell is not part of any place (occupied,
    // unknown, or outside an explored component).
    std::vector<std::uint16_t> place_assignment;

    std::string map_version;

    // Monotone per-kind counter snapshot. Carried through Phase 6 so
    // newly minted IDs continue incrementing rather than recycling.
    std::unordered_map<PlaceKind, int> id_counters;

    int place_index_of(int cx, int cy) const {
        if (cx < 0 || cy < 0 || cx >= width || cy >= height) return -1;
        std::uint16_t v = place_assignment[cy * width + cx];
        return v == kNoPlace ? -1 : static_cast<int>(v);
    }

    // O(1) waypoint -> place lookup. Returns nullptr if the waypoint
    // does not land on a cell owned by a place.
    const PlaceRegion* place_at_world(double wx, double wy) const;

    const std::string* place_id_at_world(double wx, double wy) const;
};

// Events emitted by Phase 6 matching, surfaced so callers (the future
// semantic-evidence scheduler) can react.
enum class PlaceEventKind : std::uint8_t {
    kBirth,
    kSplit,
    kMerge,
    kDeath,
    kRefresh,
    kProvisionalCleared,
};

struct PlaceEvent {
    PlaceEventKind kind;
    std::vector<std::string> old_ids;
    std::vector<std::string> new_ids;
};

}  // namespace place_graph

namespace std {
template <>
struct hash<place_graph::CellCoord> {
    size_t operator()(const place_graph::CellCoord& c) const noexcept {
        return std::hash<long long>()(
            (static_cast<long long>(c.x) << 32) ^ static_cast<unsigned>(c.y));
    }
};
// PlaceKind is an enum class; libstdc++ already provides std::hash for
// scoped enums.
}  // namespace std
