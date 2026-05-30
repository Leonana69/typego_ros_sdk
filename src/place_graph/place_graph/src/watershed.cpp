#include "place_graph/watershed.hpp"

#include <algorithm>
#include <queue>

namespace place_graph {

WatershedResult watershed_assign(const SegmentationGrid& seg,
                                 const CoresResult& cores) {
    WatershedResult out;
    const int w = seg.width;
    const int h = seg.height;
    out.core_assignment.assign(w * h, -1);
    out.boundary.assign(w * h, 0);
    if (cores.cores.empty()) return out;

    // Pre-paint frontier_region cores; their cells are excluded from
    // the multi-source BFS that follows. This keeps a half-explored
    // pocket out of the watershed competition.
    std::vector<std::uint8_t> blocked(w * h, 0);
    for (int ci = 0; ci < static_cast<int>(cores.cores.size()); ++ci) {
        const Core& c = cores.cores[ci];
        if (c.source != Core::Source::kFrontierRegion) continue;
        for (const auto& cell : c.frontier_region_cells) {
            int i = cell.y * w + cell.x;
            out.core_assignment[i] = ci;
            blocked[i] = 1;
        }
    }

    // Multi-source BFS from non-frontier cores. Geodesic distance
    // through free cells, seeded from the entire opening survivor
    // (or the single fallback seed). Single-cell seeding gave
    // corridors and open_areas an unfair disadvantage because their
    // farthest interior cell is far from every doorway.
    std::queue<int> q;
    for (int ci = 0; ci < static_cast<int>(cores.cores.size()); ++ci) {
        const Core& c = cores.cores[ci];
        if (c.source == Core::Source::kFrontierRegion) continue;
        for (const auto& cell : c.seed_cells) {
            int i = cell.y * w + cell.x;
            if (seg.free[i] == 0 || blocked[i]) continue;
            if (out.core_assignment[i] != -1) continue;
            out.core_assignment[i] = ci;
            q.push(i);
        }
    }

    while (!q.empty()) {
        int i = q.front(); q.pop();
        int x = i % w, y = i / w;
        int my = out.core_assignment[i];
        static const int dxs[] = {1, -1, 0, 0};
        static const int dys[] = {0, 0, 1, -1};
        for (int k = 0; k < 4; ++k) {
            int nx = x + dxs[k], ny = y + dys[k];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            int ni = ny * w + nx;
            if (seg.free[ni] == 0 || blocked[ni]) continue;
            if (out.core_assignment[ni] != -1) continue;
            out.core_assignment[ni] = my;
            q.push(ni);
        }
    }

    // Boundary cells: free cells with at least one 8-neighbor whose
    // core_id differs (skip non-free neighbors).
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = y * w + x;
            int me = out.core_assignment[i];
            if (me < 0) continue;
            bool bdy = false;
            for (int dy = -1; dy <= 1 && !bdy; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    int ni = ny * w + nx;
                    int other = out.core_assignment[ni];
                    if (other >= 0 && other != me) { bdy = true; break; }
                }
            }
            if (bdy) out.boundary[i] = 1;
        }
    }
    return out;
}

}  // namespace place_graph
