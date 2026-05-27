#include "typego_place_graph/preprocessing.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

namespace typego_place_graph {

namespace {

constexpr double kPi = 3.14159265358979323846;

// Round-up so the coarse grid covers the whole input even when the
// downsample ratio is fractional.
int divceil(int n, int d) { return (n + d - 1) / d; }

SegmentationGrid downsample(const MapSnapshot& map, const Config& cfg) {
    SegmentationGrid out;
    if (map.width == 0 || map.height == 0) return out;

    double target_res = std::max(cfg.seg_resolution_m, map.resolution_m);
    int factor = std::max(1,
        static_cast<int>(std::round(target_res / map.resolution_m)));
    out.resolution_m = map.resolution_m * factor;
    out.origin_x = map.origin_x;
    out.origin_y = map.origin_y;
    out.width = divceil(map.width, factor);
    out.height = divceil(map.height, factor);
    out.free.assign(out.width * out.height, 0);
    out.unknown.assign(out.width * out.height, 0);

    for (int cy = 0; cy < out.height; ++cy) {
        for (int cx = 0; cx < out.width; ++cx) {
            bool any_occ = false;
            bool any_unknown = false;
            bool any_free = false;
            int y0 = cy * factor;
            int x0 = cx * factor;
            int y1 = std::min(y0 + factor, map.height);
            int x1 = std::min(x0 + factor, map.width);
            for (int y = y0; y < y1 && !any_occ; ++y) {
                for (int x = x0; x < x1; ++x) {
                    CellState s = map.at(x, y);
                    if (s == CellState::kOccupied) { any_occ = true; break; }
                    if (s == CellState::kUnknown) any_unknown = true;
                    else any_free = true;
                }
            }
            int idx = out.index(cx, cy);
            if (any_occ) {
                out.free[idx] = 0;
                out.unknown[idx] = 0;
            } else if (any_unknown && !any_free) {
                out.unknown[idx] = 1;
            } else if (any_free) {
                out.free[idx] = 1;
                // Mixed free/unknown coarse cell: keep it free for
                // segmentation but also surface as a frontier neighbor
                // (handled later by checking obstacle/unknown adjacency
                // of free cells).
                if (any_unknown) out.unknown[idx] = 1;
            }
        }
    }
    return out;
}

// Dilate the obstacle set by `radius_cells`. The dilation is applied
// to `free` by zeroing any free cell within `radius` of an obstacle.
// Unknown cells are treated as boundaries for this purpose.
void thin_inflate_obstacles(SegmentationGrid& g, int radius_cells) {
    if (radius_cells <= 0) return;
    const int w = g.width;
    const int h = g.height;
    std::vector<std::uint8_t> obstacle(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        obstacle[i] = (g.free[i] == 0 && g.unknown[i] == 0) ? 1 : 0;
    }
    // Two-pass Chebyshev distance via BFS from obstacles.
    std::vector<int> dist(w * h, -1);
    std::queue<int> q;
    for (int i = 0; i < w * h; ++i) {
        if (obstacle[i]) { dist[i] = 0; q.push(i); }
    }
    while (!q.empty()) {
        int idx = q.front(); q.pop();
        if (dist[idx] >= radius_cells) continue;
        int x = idx % w;
        int y = idx / w;
        for (int dy = -1; dy <= 1; ++dy) {
            for (int dx = -1; dx <= 1; ++dx) {
                if (dx == 0 && dy == 0) continue;
                int nx = x + dx, ny = y + dy;
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                int nidx = ny * w + nx;
                if (dist[nidx] != -1) continue;
                dist[nidx] = dist[idx] + 1;
                q.push(nidx);
            }
        }
    }
    for (int i = 0; i < w * h; ++i) {
        if (g.free[i] && dist[i] != -1 && dist[i] <= radius_cells) {
            g.free[i] = 0;
            // Inflated cells become obstacle, not unknown: the spec
            // wants hairline cracks sealed, not new frontier mass.
        }
    }
}

// Morphologically close interior unknown pockets. A pocket is filled
// to free only if it is genuinely embedded inside explored free space:
//
//   - The unknown component must not touch the grid border (the real
//     exploration frontier reaches the border and must not be eaten).
//   - The component must fit in a disk of radius <= close_radius_cells
//     (large unknown regions are the actual frontier, not occlusion).
//   - The component's *non-unknown* perimeter neighbors must be
//     majority free, not occupied. A small unknown void embedded in a
//     wall has occupied neighbors and should stay unknown; an
//     occlusion shadow inside a room has free neighbors and is the
//     case this routine targets.
void close_unknown_pockets(SegmentationGrid& g, int close_radius_cells) {
    if (close_radius_cells <= 0) return;
    const int w = g.width;
    const int h = g.height;
    std::vector<int> comp(w * h, -1);
    int next_id = 0;
    int max_pocket_cells =
        static_cast<int>(std::ceil(
            kPi * close_radius_cells * close_radius_cells));

    std::vector<bool> touches_border;
    std::vector<int> comp_size;
    std::vector<int> free_neighbors;
    std::vector<int> occupied_neighbors;
    for (int sy = 0; sy < h; ++sy) {
        for (int sx = 0; sx < w; ++sx) {
            int sidx = sy * w + sx;
            if (g.unknown[sidx] == 0 || comp[sidx] != -1) continue;
            int id = next_id++;
            touches_border.push_back(false);
            comp_size.push_back(0);
            free_neighbors.push_back(0);
            occupied_neighbors.push_back(0);
            std::queue<int> q;
            q.push(sidx);
            comp[sidx] = id;
            while (!q.empty()) {
                int idx = q.front(); q.pop();
                int x = idx % w, y = idx / w;
                comp_size[id]++;
                if (x == 0 || y == 0 || x == w - 1 || y == h - 1) {
                    touches_border[id] = true;
                }
                static const int dxs[] = {1, -1, 0, 0};
                static const int dys[] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    int nx = x + dxs[k], ny = y + dys[k];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    int nidx = ny * w + nx;
                    if (g.unknown[nidx] != 0) {
                        if (comp[nidx] == -1) {
                            comp[nidx] = id;
                            q.push(nidx);
                        }
                        continue;
                    }
                    if (g.free[nidx]) free_neighbors[id]++;
                    else occupied_neighbors[id]++;
                }
            }
        }
    }
    for (int i = 0; i < w * h; ++i) {
        if (comp[i] == -1) continue;
        int id = comp[i];
        if (touches_border[id]) continue;
        if (comp_size[id] > max_pocket_cells) continue;
        // Embedded in free space (perimeter majority-free)? Pockets
        // bordered mostly by occupied cells are wall interiors and
        // stay unknown.
        if (free_neighbors[id] <= occupied_neighbors[id]) continue;
        g.unknown[i] = 0;
        g.free[i] = 1;
    }
}

}  // namespace

SegmentationGrid build_segmentation_grid(const MapSnapshot& map,
                                         const Config& cfg) {
    SegmentationGrid g = downsample(map, cfg);
    if (g.width == 0 || g.height == 0) return g;
    thin_inflate_obstacles(g, cfg.seg_inflation_cells);
    int close_radius_cells = static_cast<int>(
        std::round(cfg.unknown_close_m / g.resolution_m));
    close_unknown_pockets(g, close_radius_cells);
    return g;
}

}  // namespace typego_place_graph
