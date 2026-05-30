#include "place_graph/connectors.hpp"

#include <algorithm>
#include <cmath>
#include <queue>
#include <set>

namespace place_graph {

namespace {

// Connected components of the boundary mask, free cells only. 8-conn.
std::vector<int> boundary_components(const WatershedResult& ws,
                                     const SegmentationGrid& seg,
                                     int* num) {
    const int w = seg.width;
    const int h = seg.height;
    std::vector<int> comp(w * h, -1);
    int next = 0;
    for (int sy = 0; sy < h; ++sy) {
        for (int sx = 0; sx < w; ++sx) {
            int si = sy * w + sx;
            if (ws.boundary[si] == 0 || comp[si] != -1) continue;
            int id = next++;
            std::queue<int> q;
            q.push(si);
            comp[si] = id;
            while (!q.empty()) {
                int i = q.front(); q.pop();
                int x = i % w, y = i / w;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                        int ni = ny * w + nx;
                        if (ws.boundary[ni] == 0 || comp[ni] != -1) continue;
                        comp[ni] = id;
                        q.push(ni);
                    }
                }
            }
        }
    }
    if (num) *num = next;
    return comp;
}

struct RawCandidate {
    std::vector<CellCoord> boundary_cells;
    std::set<int> endpoint_cores;
    // Smallest distance-to-obstacle across boundary cells (cells).
    float min_clearance_cells = 0.0f;
    int narrow_x = 0;
    int narrow_y = 0;
};

// Collect each boundary component's endpoint-core set by inspecting
// 8-connected neighbors of its cells on the core_assignment grid.
// Frontier_region cores are valid endpoints; they still need a
// connector to be reachable from the rest of the graph.
std::vector<RawCandidate> collect_candidates(
    const SegmentationGrid& seg,
    const WatershedResult& ws,
    const std::vector<int>& comp,
    const std::vector<float>& distance_cells,
    int num) {
    const int w = seg.width;
    const int h = seg.height;
    std::vector<RawCandidate> out(num);
    for (auto& c : out) c.min_clearance_cells = std::numeric_limits<float>::infinity();
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = y * w + x;
            int cid = comp[i];
            if (cid < 0) continue;
            auto& cand = out[cid];
            cand.boundary_cells.push_back({x, y});
            int me = ws.core_assignment[i];
            if (me >= 0) cand.endpoint_cores.insert(me);
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    int ni = ny * w + nx;
                    int oc = ws.core_assignment[ni];
                    if (oc >= 0) cand.endpoint_cores.insert(oc);
                }
            }
            float d = distance_cells[i];
            if (d < cand.min_clearance_cells) {
                cand.min_clearance_cells = d;
                cand.narrow_x = x;
                cand.narrow_y = y;
            }
        }
    }
    return out;
}

// Materialize a connector region: cells of the *free* mask within
// width_cells * 0.75 of any cell in the candidate boundary, capped by
// distance-transform-based "passage cells" (cells whose own clearance
// is <= width/2 + slack). The intersection captures the actual
// throat without bleeding into the adjacent rooms.
std::vector<CellCoord> materialize_region(
    const RawCandidate& cand,
    const SegmentationGrid& seg,
    const std::vector<float>& distance_cells,
    double width_cells) {
    const int w = seg.width;
    const int h = seg.height;
    int radius = std::max(1, static_cast<int>(std::round(width_cells * 0.75)));
    std::vector<int> bfs(w * h, -1);
    std::queue<int> q;
    for (const auto& c : cand.boundary_cells) {
        int i = c.y * w + c.x;
        bfs[i] = 0;
        q.push(i);
    }
    while (!q.empty()) {
        int i = q.front(); q.pop();
        if (bfs[i] >= radius) continue;
        int x = i % w, y = i / w;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int ni = ny * w + nx;
                if (!seg.free[ni] || bfs[ni] != -1) continue;
                bfs[ni] = bfs[i] + 1;
                q.push(ni);
            }
        }
    }
    // Slack of 0.75 cell so a cell at the narrowest point is included
    // even with floating-point rounding.
    float clearance_cap = static_cast<float>(width_cells / 2.0 + 0.75);
    std::vector<CellCoord> cells;
    for (int i = 0; i < w * h; ++i) {
        if (bfs[i] < 0) continue;
        if (distance_cells[i] > clearance_cap) continue;
        cells.push_back({i % w, i / w});
    }
    return cells;
}

}  // namespace

std::vector<ConnectorCandidate> extract_connectors(
    const SegmentationGrid& seg,
    const CoresResult& cores,
    const WatershedResult& ws,
    const Config& cfg) {
    std::vector<ConnectorCandidate> out;
    if (cores.cores.size() < 2) return out;
    int num = 0;
    auto comp = boundary_components(ws, seg, &num);
    auto cands = collect_candidates(seg, ws, comp, cores.distance_cells, num);
    double cell_size = seg.resolution_m;

    for (auto& cand : cands) {
        if (cand.endpoint_cores.size() < 2) continue;
        // width in meters = 2 * min_clearance (wall-to-wall). The
        // distance transform stores radius-to-nearest-obstacle.
        double width_m = 2.0 * cand.min_clearance_cells * cell_size;
        std::vector<int> endpoints(cand.endpoint_cores.begin(),
                                   cand.endpoint_cores.end());

        // Dispatch by endpoint-set size:
        //   |set| == 2 → portal if width_m ≤ w_max_portal_m, else
        //                open_transition;
        //   |set| ≥ 3 → junction.
        //
        // An earlier version filtered out candidates whose narrowest
        // clearance exceeded ~1.5 × r_open, on the theory that
        // watershed boundaries inside a uniform-width space would
        // otherwise surface as phantom portals. With full opening-
        // survivor seeding in watershed_assign, the watershed sits
        // at genuine chokepoints by construction, so the absolute
        // filter was suppressing legitimate wide transitions and
        // dropping portals in the 1.2–2.5 m range. It has been
        // removed in favor of trusting the segmentation.
        ConnectorCandidate cc;
        cc.endpoint_cores = endpoints;
        cc.width_m = width_m;
        if (endpoints.size() == 2) {
            cc.kind = width_m <= cfg.w_max_portal_m ? PlaceKind::kPortal
                                                    : PlaceKind::kOpenTransition;
        } else {
            cc.kind = PlaceKind::kJunction;
        }
        double width_cells = std::max(2.0, width_m / cell_size);
        cc.cells = materialize_region(cand, seg, cores.distance_cells,
                                      width_cells);
        if (cc.cells.empty()) continue;

        // Centroid.
        double sum_x = 0, sum_y = 0;
        for (const auto& c : cc.cells) { sum_x += c.x; sum_y += c.y; }
        double cx = sum_x / cc.cells.size();
        double cy = sum_y / cc.cells.size();
        cc.centroid.x = seg.origin_x + (cx + 0.5) * seg.resolution_m;
        cc.centroid.y = seg.origin_y + (cy + 0.5) * seg.resolution_m;

        // Principal axis of the boundary band via second-moment
        // matrix. For a thin band this is the centerline direction;
        // approach_yaw will be its perpendicular (set later, after
        // adjacent room centroids are known, so it can be flipped
        // toward the larger room). Cell-frame and map-frame axes
        // share the same angle because downsample is axis-aligned.
        double mx = 0.0, my = 0.0;
        for (const auto& c : cand.boundary_cells) { mx += c.x; my += c.y; }
        mx /= cand.boundary_cells.size();
        my /= cand.boundary_cells.size();
        double sxx = 0.0, syy = 0.0, sxy = 0.0;
        for (const auto& c : cand.boundary_cells) {
            double dx = c.x - mx, dy = c.y - my;
            sxx += dx * dx;
            syy += dy * dy;
            sxy += dx * dy;
        }
        cc.axis_angle = 0.5 * std::atan2(2.0 * sxy, sxx - syy);

        // Clearance: for portals/open_transitions, half the wall-to-
        // wall width is the closest obstacle distance at the throat.
        // For junctions the throat metric doesn't capture the open
        // interior, so report the max distance-transform value over
        // the connector cells — the most open spot inside it.
        if (cc.kind == PlaceKind::kJunction) {
            float max_d_cells = 0.0f;
            const int w_cells = seg.width;
            for (const auto& c : cc.cells) {
                float d = cores.distance_cells[c.y * w_cells + c.x];
                if (d > max_d_cells) max_d_cells = d;
            }
            cc.clearance_m = static_cast<double>(max_d_cells) * cell_size;
        } else {
            cc.clearance_m = width_m / 2.0;
        }

        out.push_back(cc);
    }
    return out;
}

}  // namespace place_graph
