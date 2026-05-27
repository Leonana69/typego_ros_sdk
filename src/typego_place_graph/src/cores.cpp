#include "typego_place_graph/cores.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_map>

namespace typego_place_graph {

namespace {

// Euclidean distance transform of the free mask, in cell units.
// Two-pass forward/backward propagation against neighbor distances
// (Felzenszwalb-style approximation that's exact for taxicab + cheap
// in C++). Good enough for the morphological radius and connector
// width checks we use it for.
std::vector<float> distance_transform(const SegmentationGrid& g) {
    const int w = g.width;
    const int h = g.height;
    const float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> d(w * h, kInf);
    for (int i = 0; i < w * h; ++i) {
        if (g.free[i] == 0) d[i] = 0.0f;
    }
    auto idx = [w](int x, int y) { return y * w + x; };
    // Forward pass.
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = idx(x, y);
            if (d[i] == 0.0f) continue;
            float best = d[i];
            if (x > 0) best = std::min(best, d[idx(x - 1, y)] + 1.0f);
            if (y > 0) best = std::min(best, d[idx(x, y - 1)] + 1.0f);
            if (x > 0 && y > 0)
                best = std::min(best, d[idx(x - 1, y - 1)] + 1.41421356f);
            if (x + 1 < w && y > 0)
                best = std::min(best, d[idx(x + 1, y - 1)] + 1.41421356f);
            d[i] = best;
        }
    }
    // Backward pass.
    for (int y = h - 1; y >= 0; --y) {
        for (int x = w - 1; x >= 0; --x) {
            int i = idx(x, y);
            if (d[i] == 0.0f) continue;
            float best = d[i];
            if (x + 1 < w) best = std::min(best, d[idx(x + 1, y)] + 1.0f);
            if (y + 1 < h) best = std::min(best, d[idx(x, y + 1)] + 1.0f);
            if (x + 1 < w && y + 1 < h)
                best = std::min(best, d[idx(x + 1, y + 1)] + 1.41421356f);
            if (x > 0 && y + 1 < h)
                best = std::min(best, d[idx(x - 1, y + 1)] + 1.41421356f);
            d[i] = best;
        }
    }
    return d;
}

// Morphological opening with a disk structuring element of radius
// r_cells. Standard definition: open(X) = dilate(erode(X)). The
// dilation walks through space (not through X), so a doorway that
// gets fully eroded prevents the two sides from re-merging — that is
// the exact behavior the spec relies on for room cores.
//
// A cell survives the opening iff it's within Euclidean distance r of
// some cell whose distance-to-nearest-obstacle is >= r. We compute
// that by treating the eroded set as the "obstacles" of a second
// distance transform, and accepting any cell whose distance to that
// set is <= r AND was free in the original X.
std::vector<std::uint8_t> open_mask(const SegmentationGrid& g,
                                    const std::vector<float>& dist,
                                    double r_cells_d) {
    const int w = g.width;
    const int h = g.height;
    std::vector<std::uint8_t> eroded(w * h, 0);
    float r = static_cast<float>(r_cells_d);
    for (int i = 0; i < w * h; ++i) {
        if (g.free[i] && dist[i] >= r) eroded[i] = 1;
    }
    // Distance from each cell to the nearest eroded survivor (free-
    // space distance via 8-connectivity). Two-pass approximate
    // Euclidean transform mirroring the obstacle distance transform.
    const float kInf = std::numeric_limits<float>::infinity();
    std::vector<float> d(w * h, kInf);
    for (int i = 0; i < w * h; ++i) {
        if (eroded[i]) d[i] = 0.0f;
    }
    auto idx = [w](int x, int y) { return y * w + x; };
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = idx(x, y);
            if (d[i] == 0.0f) continue;
            float best = d[i];
            if (x > 0) best = std::min(best, d[idx(x - 1, y)] + 1.0f);
            if (y > 0) best = std::min(best, d[idx(x, y - 1)] + 1.0f);
            if (x > 0 && y > 0)
                best = std::min(best, d[idx(x - 1, y - 1)] + 1.41421356f);
            if (x + 1 < w && y > 0)
                best = std::min(best, d[idx(x + 1, y - 1)] + 1.41421356f);
            d[i] = best;
        }
    }
    for (int y = h - 1; y >= 0; --y) {
        for (int x = w - 1; x >= 0; --x) {
            int i = idx(x, y);
            if (d[i] == 0.0f) continue;
            float best = d[i];
            if (x + 1 < w) best = std::min(best, d[idx(x + 1, y)] + 1.0f);
            if (y + 1 < h) best = std::min(best, d[idx(x, y + 1)] + 1.0f);
            if (x + 1 < w && y + 1 < h)
                best = std::min(best, d[idx(x + 1, y + 1)] + 1.41421356f);
            if (x > 0 && y + 1 < h)
                best = std::min(best, d[idx(x - 1, y + 1)] + 1.41421356f);
            d[i] = best;
        }
    }
    std::vector<std::uint8_t> out(w * h, 0);
    for (int i = 0; i < w * h; ++i) {
        if (g.free[i] && d[i] <= r) out[i] = 1;
    }
    return out;
}

// Flood the free cells of `g` to component ids. Unknown and occupied
// cells get -1.
std::vector<int> connected_components_of_free(const SegmentationGrid& g,
                                              int* num_comps) {
    const int w = g.width;
    const int h = g.height;
    std::vector<int> comp(w * h, -1);
    int next = 0;
    for (int sy = 0; sy < h; ++sy) {
        for (int sx = 0; sx < w; ++sx) {
            int si = sy * w + sx;
            if (g.free[si] == 0 || comp[si] != -1) continue;
            int id = next++;
            std::queue<int> q;
            q.push(si);
            comp[si] = id;
            while (!q.empty()) {
                int i = q.front(); q.pop();
                int x = i % w, y = i / w;
                static const int dxs[] = {1, -1, 0, 0};
                static const int dys[] = {0, 0, 1, -1};
                for (int k = 0; k < 4; ++k) {
                    int nx = x + dxs[k], ny = y + dys[k];
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                    int ni = ny * w + nx;
                    if (g.free[ni] == 0 || comp[ni] != -1) continue;
                    comp[ni] = id;
                    q.push(ni);
                }
            }
        }
    }
    if (num_comps) *num_comps = next;
    return comp;
}

// Connected components on a binary 8-mask. Used to label opening
// survivors. 8-connectivity is the natural choice here because the
// opening removed the thin chokepoints we care about, and 4-connect
// would over-fragment compact blobs that only touch diagonally.
std::vector<int> components_of_mask(const std::vector<std::uint8_t>& mask,
                                    int w, int h, int* num) {
    std::vector<int> comp(w * h, -1);
    int next = 0;
    for (int sy = 0; sy < h; ++sy) {
        for (int sx = 0; sx < w; ++sx) {
            int si = sy * w + sx;
            if (mask[si] == 0 || comp[si] != -1) continue;
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
                        if (mask[ni] == 0 || comp[ni] != -1) continue;
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

struct OpenedCoreStats {
    std::vector<CellCoord> cells;
    int seed_x = 0;
    int seed_y = 0;
    float max_dist = 0.0f;
    double area_m2 = 0.0;
    int min_x = 0, max_x = 0, min_y = 0, max_y = 0;
};

OpenedCoreStats stats_for_opened_core(
    const std::vector<int>& opened_comp, int id,
    const std::vector<float>& dist, int w, int h, double cell_area_m2) {
    OpenedCoreStats s;
    s.min_x = w; s.min_y = h; s.max_x = -1; s.max_y = -1;
    for (int i = 0; i < w * h; ++i) {
        if (opened_comp[i] != id) continue;
        int x = i % w, y = i / w;
        s.cells.push_back({x, y});
        if (dist[i] > s.max_dist) {
            s.max_dist = dist[i];
            s.seed_x = x;
            s.seed_y = y;
        }
        s.min_x = std::min(s.min_x, x);
        s.max_x = std::max(s.max_x, x);
        s.min_y = std::min(s.min_y, y);
        s.max_y = std::max(s.max_y, y);
    }
    s.area_m2 = s.cells.size() * cell_area_m2;
    return s;
}

PlaceKind classify_core(const OpenedCoreStats& s, const Config& cfg) {
    int bw = s.max_x - s.min_x + 1;
    int bh = s.max_y - s.min_y + 1;
    double aspect =
        static_cast<double>(std::max(bw, bh)) / std::max(1, std::min(bw, bh));
    if (s.area_m2 >= cfg.a_open_m2) return PlaceKind::kOpenArea;
    if (aspect >= cfg.corridor_aspect) return PlaceKind::kCorridor;
    return PlaceKind::kRoom;
}

}  // namespace

CoresResult extract_cores(const SegmentationGrid& seg, const Config& cfg) {
    CoresResult out;
    if (seg.width == 0 || seg.height == 0) {
        return out;
    }

    const int w = seg.width;
    const int h = seg.height;
    out.distance_cells = distance_transform(seg);

    int num_free_comps = 0;
    out.component_of = connected_components_of_free(seg, &num_free_comps);

    double cell_size = seg.resolution_m;
    double cell_area = cell_size * cell_size;

    // Opening radius in cells. r_open is meters; cell size is meters.
    double r_open_cells = cfg.r_open_m / std::max(cell_size, 1e-6);
    auto opened = open_mask(seg, out.distance_cells, r_open_cells);

    int num_opened = 0;
    auto opened_comp = components_of_mask(opened, w, h, &num_opened);

    // Drop opening artifacts smaller than A_min.
    std::vector<bool> opened_alive(num_opened, true);
    std::vector<int> opened_size(num_opened, 0);
    for (int i = 0; i < w * h; ++i) {
        if (opened_comp[i] >= 0) opened_size[opened_comp[i]]++;
    }
    int a_min_cells = static_cast<int>(std::ceil(cfg.a_min_m2 / cell_area));
    for (int id = 0; id < num_opened; ++id) {
        if (opened_size[id] < a_min_cells) opened_alive[id] = false;
    }

    // Surviving opened cores -> Core records.
    std::unordered_map<int, int> free_comp_has_core;
    for (int id = 0; id < num_opened; ++id) {
        if (!opened_alive[id]) continue;
        OpenedCoreStats s = stats_for_opened_core(
            opened_comp, id, out.distance_cells, w, h, cell_area);
        Core c;
        c.source = Core::Source::kOpened;
        c.seed_x = s.seed_x;
        c.seed_y = s.seed_y;
        c.seed_cells = s.cells;
        c.area_m2 = s.area_m2;
        c.clearance_m = static_cast<double>(s.max_dist) * cell_size;
        c.kind = classify_core(s, cfg);
        out.cores.push_back(c);
        int parent_free = out.component_of[s.seed_y * w + s.seed_x];
        if (parent_free >= 0) free_comp_has_core[parent_free] = 1;
    }

    // Step 3 from the spec: for each unopened free-space component
    // without a surviving core, dispatch by frontier ratio.
    //
    // Frontier ratio is the fraction of the component's perimeter that
    // touches unknown cells. The component perimeter is the set of
    // free cells that have any 4-neighbor that's NOT a free cell of
    // the same component.
    struct CompFrontier {
        int perimeter = 0;
        int frontier = 0;
        int seed_x = 0;
        int seed_y = 0;
        float max_dist = 0.0f;
        int size = 0;
    };
    std::vector<CompFrontier> cf(num_free_comps);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = y * w + x;
            int cid = out.component_of[i];
            if (cid < 0) continue;
            cf[cid].size++;
            if (out.distance_cells[i] > cf[cid].max_dist) {
                cf[cid].max_dist = out.distance_cells[i];
                cf[cid].seed_x = x;
                cf[cid].seed_y = y;
            }
            bool is_perimeter = false;
            bool touches_unknown = false;
            static const int dxs[] = {1, -1, 0, 0};
            static const int dys[] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                int nx = x + dxs[k], ny = y + dys[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                    is_perimeter = true;
                    continue;
                }
                int ni = ny * w + nx;
                if (out.component_of[ni] != cid) {
                    is_perimeter = true;
                    if (seg.unknown[ni]) touches_unknown = true;
                }
            }
            if (is_perimeter) cf[cid].perimeter++;
            if (touches_unknown) cf[cid].frontier++;
        }
    }

    for (int cid = 0; cid < num_free_comps; ++cid) {
        if (free_comp_has_core.count(cid)) continue;
        if (cf[cid].size == 0) continue;
        double fratio = cf[cid].perimeter > 0
            ? static_cast<double>(cf[cid].frontier) / cf[cid].perimeter
            : 0.0;
        if (fratio > cfg.frontier_region_ratio) {
            Core c;
            c.source = Core::Source::kFrontierRegion;
            c.kind = PlaceKind::kFrontierRegion;
            c.seed_x = cf[cid].seed_x;
            c.seed_y = cf[cid].seed_y;
            c.area_m2 = cf[cid].size * cell_area;
            c.clearance_m = cf[cid].max_dist * cell_size;
            c.frontier_region_cells.reserve(cf[cid].size);
            for (int i = 0; i < w * h; ++i) {
                if (out.component_of[i] == cid) {
                    c.frontier_region_cells.push_back({i % w, i / w});
                }
            }
            out.cores.push_back(c);
        } else {
            // Fallback seed at the component's distance-transform max.
            // Classify the seeded core by treating the whole component
            // as its support: we can't classify by opening shape since
            // it didn't survive opening.
            Core c;
            c.source = Core::Source::kFallback;
            c.seed_x = cf[cid].seed_x;
            c.seed_y = cf[cid].seed_y;
            c.seed_cells.push_back({cf[cid].seed_x, cf[cid].seed_y});
            c.area_m2 = cf[cid].size * cell_area;
            c.clearance_m = cf[cid].max_dist * cell_size;
            // Use the component bbox to classify.
            int min_x = w, max_x = -1, min_y = h, max_y = -1;
            for (int i = 0; i < w * h; ++i) {
                if (out.component_of[i] != cid) continue;
                int x = i % w, y = i / w;
                min_x = std::min(min_x, x);
                max_x = std::max(max_x, x);
                min_y = std::min(min_y, y);
                max_y = std::max(max_y, y);
            }
            int bw = max_x - min_x + 1;
            int bh = max_y - min_y + 1;
            double aspect = static_cast<double>(std::max(bw, bh)) /
                            std::max(1, std::min(bw, bh));
            if (c.area_m2 >= cfg.a_open_m2) c.kind = PlaceKind::kOpenArea;
            else if (aspect >= cfg.corridor_aspect) c.kind = PlaceKind::kCorridor;
            else c.kind = PlaceKind::kRoom;
            out.cores.push_back(c);
        }
    }
    return out;
}

}  // namespace typego_place_graph
