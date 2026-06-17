#include "place_graph/sampling.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace place_graph {
namespace {

// World coordinate of a cell center (mirrors the convention in place_graph.cpp).
double cell_center_x(const PlaceGraphSnapshot& s, int cx) {
    return s.origin_x + (cx + 0.5) * s.resolution_m;
}
double cell_center_y(const PlaceGraphSnapshot& s, int cy) {
    return s.origin_y + (cy + 0.5) * s.resolution_m;
}

// Collect the region's cells reachable within `range_m` of in-region travel from
// the query cell. 8-connected; diagonal steps are barred from cutting a corner
// between two non-region cells, so the flooded set is genuinely reachable
// without leaving the region.
std::vector<CellCoord> geodesic_candidates(const PlaceGraphSnapshot& snap,
                                           int rcx, int rcy, int pidx,
                                           double range_m) {
    const int W = snap.width;
    const int H = snap.height;
    const double res = snap.resolution_m;
    const double inf = std::numeric_limits<double>::infinity();

    auto in_region = [&](int x, int y) {
        return snap.place_index_of(x, y) == pidx;
    };

    std::vector<double> dist(static_cast<std::size_t>(W) * H, inf);
    using QItem = std::pair<double, int>;  // (distance, flat index)
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;

    const int seed = rcy * W + rcx;
    dist[seed] = 0.0;
    pq.push({0.0, seed});

    const double orth = res;
    const double diag = std::sqrt(2.0) * res;
    static const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    std::vector<CellCoord> out;
    while (!pq.empty()) {
        auto [d, idx] = pq.top();
        pq.pop();
        if (d > dist[idx]) continue;  // stale queue entry
        const int cx = idx % W;
        const int cy = idx / W;
        out.push_back({cx, cy});
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dxs[k];
            const int ny = cy + dys[k];
            if (!in_region(nx, ny)) continue;
            const bool diagonal = (dxs[k] != 0 && dys[k] != 0);
            if (diagonal &&
                (!in_region(cx + dxs[k], cy) || !in_region(cx, cy + dys[k]))) {
                continue;  // no corner-cutting between non-region cells
            }
            const double nd = d + (diagonal ? diag : orth);
            if (nd > range_m) continue;
            const std::size_t nidx = static_cast<std::size_t>(ny) * W + nx;
            if (nd < dist[nidx]) {
                dist[nidx] = nd;
                pq.push({nd, static_cast<int>(nidx)});
            }
        }
    }
    return out;
}

std::vector<CellCoord> euclidean_candidates(const PlaceRegion& region,
                                            const PlaceGraphSnapshot& snap,
                                            double wx, double wy,
                                            double range_m) {
    const double r2 = range_m * range_m;
    std::vector<CellCoord> out;
    for (const CellCoord& c : region.cell_region) {
        const double dx = cell_center_x(snap, c.x) - wx;
        const double dy = cell_center_y(snap, c.y) - wy;
        if (dx * dx + dy * dy <= r2) out.push_back(c);
    }
    return out;
}

// True when cell (cx, cy) is at least `clearance_m` from the nearest wall /
// obstacle / unknown cell. In the assignment grid such cells (and the area
// outside the grid) are kNoPlace, i.e. place_index_of < 0; cells belonging to
// OTHER places (rooms, portals) keep their own index and are NOT treated as
// obstacles, so a doorway opening to another room does not count against the
// margin -- only real walls and obstacles do.
bool cell_has_clearance(const PlaceGraphSnapshot& snap, int cx, int cy,
                        double clearance_m) {
    if (clearance_m <= 0.0) return true;
    const double res = snap.resolution_m;
    const int r = static_cast<int>(std::ceil(clearance_m / res));
    for (int dy = -r; dy <= r; ++dy) {
        for (int dx = -r; dx <= r; ++dx) {
            // Other-place / in-region cells are free space; only kNoPlace and
            // out-of-bounds (both place_index_of < 0) are walls/obstacles.
            if (snap.place_index_of(cx + dx, cy + dy) >= 0) continue;
            const double dist = res * std::sqrt(static_cast<double>(dx) * dx +
                                                static_cast<double>(dy) * dy);
            if (dist + 1e-9 < clearance_m) return false;
        }
    }
    return true;
}

}  // namespace

std::optional<SampleResult> sample_in_region(const PlaceGraphSnapshot& snap,
                                             double wx, double wy,
                                             double range_m, std::mt19937& rng,
                                             SampleMode mode,
                                             double min_clearance_m) {
    if (snap.resolution_m <= 0.0 || range_m < 0.0) return std::nullopt;

    const PlaceRegion* region = snap.place_at_world(wx, wy);
    if (!region) return std::nullopt;  // query point not owned by any place

    // Region-level clearance fast-path: the region's clearance_m is its widest
    // interior clearance, so if it is already below the floor no cell can pass
    // the per-cell margin test below -- bail before flooding.
    if (min_clearance_m > 0.0 && region->clearance_m < min_clearance_m) {
        return std::nullopt;
    }

    // place_at_world resolved to `region`, so the query cell is owned by it.
    const int rcx = static_cast<int>((wx - snap.origin_x) / snap.resolution_m);
    const int rcy = static_cast<int>((wy - snap.origin_y) / snap.resolution_m);
    const int pidx = snap.place_index_of(rcx, rcy);
    if (pidx < 0) return std::nullopt;  // defensive; should not happen

    std::vector<CellCoord> candidates =
        (mode == SampleMode::kEuclidean)
            ? euclidean_candidates(*region, snap, wx, wy, range_m)
            : geodesic_candidates(snap, rcx, rcy, pidx, range_m);

    // Keep only points at least `min_clearance_m` from any wall/obstacle.
    if (min_clearance_m > 0.0) {
        std::vector<CellCoord> kept;
        kept.reserve(candidates.size());
        for (const CellCoord& c : candidates)
            if (cell_has_clearance(snap, c.x, c.y, min_clearance_m))
                kept.push_back(c);
        candidates.swap(kept);
    }
    if (candidates.empty()) return std::nullopt;

    std::uniform_int_distribution<std::size_t> pick(0, candidates.size() - 1);
    std::uniform_real_distribution<double> jit(0.0, 1.0);
    const CellCoord& chosen = candidates[pick(rng)];

    SampleResult result;
    // Full-cell jitter so results aren't grid-locked. (cell.x + U[0,1)) keeps
    // the point inside the chosen cell, so it still resolves to this region.
    result.point.x = snap.origin_x + (chosen.x + jit(rng)) * snap.resolution_m;
    result.point.y = snap.origin_y + (chosen.y + jit(rng)) * snap.resolution_m;
    result.place_id = region->place_id;
    return result;
}

}  // namespace place_graph
