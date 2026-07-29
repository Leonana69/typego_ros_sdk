#include "place_graph/coverage.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

namespace place_graph {
namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();
// "No obstacle in this row/column" sentinel for the distance transform. Large
// enough to dominate any real squared cell distance, small enough that adding
// q*q to it stays exact in double (a 1e20 sentinel would round q*q away and
// silently produce zero distances).
constexpr double kBig = 1e9;

// Lower envelope of parabolas: 1D squared distance transform of `f`
// (Felzenszwalb & Huttenlocher). `v` and `z` are scratch, sized >= n and n + 1,
// passed in so the 2D driver allocates once.
void edt_1d(const std::vector<double>& f, std::vector<double>& d, int n,
            std::vector<int>& v, std::vector<double>& z) {
    int k = 0;
    v[0] = 0;
    z[0] = -kInf;
    z[1] = kInf;
    for (int q = 1; q < n; ++q) {
        const double fq = f[q] + static_cast<double>(q) * q;
        double s = (fq - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
                   (2.0 * q - 2.0 * v[k]);
        while (s <= z[k]) {
            --k;
            s = (fq - (f[v[k]] + static_cast<double>(v[k]) * v[k])) /
                (2.0 * q - 2.0 * v[k]);
        }
        ++k;
        v[k] = q;
        z[k] = s;
        z[k + 1] = kInf;
    }
    k = 0;
    for (int q = 0; q < n; ++q) {
        while (z[k + 1] < q) ++k;
        const double dq = static_cast<double>(q) - v[k];
        d[q] = dq * dq + f[v[k]];
    }
}

// Exact squared euclidean distance (in cells) from every cell to the nearest
// cell where `blocked` is set. Separable, O(w*h) -- not a chamfer
// approximation, so the clearance it yields agrees with an exact disc test.
std::vector<double> squared_edt(const std::vector<char>& blocked, int w, int h) {
    std::vector<double> d(static_cast<std::size_t>(w) * h, 0.0);
    {
        std::vector<double> f(h), dd(h), z(h + 1);
        std::vector<int> v(h);
        for (int x = 0; x < w; ++x) {
            for (int y = 0; y < h; ++y)
                f[y] = blocked[static_cast<std::size_t>(y) * w + x] ? 0.0 : kBig;
            edt_1d(f, dd, h, v, z);
            for (int y = 0; y < h; ++y)
                d[static_cast<std::size_t>(y) * w + x] = dd[y];
        }
    }
    {
        std::vector<double> f(w), dd(w), z(w + 1);
        std::vector<int> v(w);
        for (int y = 0; y < h; ++y) {
            const std::size_t row = static_cast<std::size_t>(y) * w;
            for (int x = 0; x < w; ++x) f[x] = d[row + x];
            edt_1d(f, dd, w, v, z);
            for (int x = 0; x < w; ++x) d[row + x] = dd[x];
        }
    }
    return d;
}

// Region membership as a flat mask, for O(1) lookup in the inner loops.
std::vector<char> region_mask(const PlaceGraphSnapshot& snap,
                              const PlaceRegion& region) {
    std::vector<char> mask(
        static_cast<std::size_t>(snap.width) * snap.height, 0);
    for (const CellCoord& c : region.cell_region) {
        if (c.x < 0 || c.y < 0 || c.x >= snap.width || c.y >= snap.height)
            continue;
        mask[static_cast<std::size_t>(c.y) * snap.width + c.x] = 1;
    }
    return mask;
}

// Multi-source geodesic Dijkstra restricted to `in_region`: 8-connected, with
// diagonal steps barred from cutting a corner between two non-region cells, so
// every distance is achievable without leaving the place. Lowers `dist` in
// place, which is what makes the sampler incremental -- adding a point never
// rebuilds the field.
void geodesic_relax(const std::vector<char>& in_region, int W, int H, double res,
                    const std::vector<int>& sources, std::vector<double>& dist,
                    double max_dist = kInf) {
    static const int dxs[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int dys[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    const double orth = res;
    const double diag = std::sqrt(2.0) * res;

    using QItem = std::pair<double, int>;
    std::priority_queue<QItem, std::vector<QItem>, std::greater<QItem>> pq;
    for (int s : sources) {
        if (dist[s] <= 0.0) continue;
        dist[s] = 0.0;
        pq.push({0.0, s});
    }
    while (!pq.empty()) {
        const auto [d, idx] = pq.top();
        pq.pop();
        if (d > dist[idx]) continue;  // stale queue entry
        if (d > max_dist) break;      // bounded flood: everything else is farther
        const int cx = idx % W;
        const int cy = idx / W;
        for (int k = 0; k < 8; ++k) {
            const int nx = cx + dxs[k];
            const int ny = cy + dys[k];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            const std::size_t ni = static_cast<std::size_t>(ny) * W + nx;
            if (!in_region[ni]) continue;
            const bool diagonal = (dxs[k] != 0 && dys[k] != 0);
            if (diagonal &&
                (!in_region[static_cast<std::size_t>(cy) * W + nx] ||
                 !in_region[static_cast<std::size_t>(ny) * W + cx])) {
                continue;
            }
            const double nd = d + (diagonal ? diag : orth);
            if (nd < dist[ni]) {
                dist[ni] = nd;
                pq.push({nd, static_cast<int>(ni)});
            }
        }
    }
}

// Distance from the region's own boundary, within the region. Its ridge is the
// place's centerline. Unlike wall clearance this needs no occupancy map, and it
// treats a doorway into the next room as an edge -- which is what we want when
// locating the END of a place.
std::vector<double> region_depth(const std::vector<char>& in_region, int W,
                                 int H, double res) {
    const std::size_t N = static_cast<std::size_t>(W) * H;
    std::vector<double> depth(N, kInf);
    std::vector<int> seeds;
    for (int y = 0; y < H; ++y) {
        for (int x = 0; x < W; ++x) {
            const std::size_t i = static_cast<std::size_t>(y) * W + x;
            if (!in_region[i]) continue;
            bool edge = false;
            for (int dy = -1; dy <= 1 && !edge; ++dy)
                for (int dx = -1; dx <= 1 && !edge; ++dx) {
                    const int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H ||
                        !in_region[static_cast<std::size_t>(ny) * W + nx])
                        edge = true;
                }
            if (edge) seeds.push_back(static_cast<int>(i));
        }
    }
    geodesic_relax(in_region, W, H, res, seeds, depth);
    return depth;
}

}  // namespace

double PlaceRoute::along_at(const PlaceGraphSnapshot& snap, double wx,
                            double wy) const {
    if (along.empty() || snap.resolution_m <= 0.0) return kInf;
    const int cx =
        static_cast<int>(std::floor((wx - snap.origin_x) / snap.resolution_m));
    const int cy =
        static_cast<int>(std::floor((wy - snap.origin_y) / snap.resolution_m));
    if (cx < 0 || cy < 0 || cx >= snap.width || cy >= snap.height) return kInf;
    return along[static_cast<std::size_t>(cy) * snap.width + cx];
}

PlaceRoute compute_place_route(const PlaceGraphSnapshot& snap,
                               const PlaceRegion& region,
                               const CoverageConfig& cfg,
                               const Point2d* preferred_start) {
    PlaceRoute r;
    if (region.cell_region.empty() || snap.width <= 0 || snap.height <= 0 ||
        snap.resolution_m <= 0.0)
        return r;
    // A long hallway usually classifies as kOpenArea (its area exceeds
    // Config::a_open_m2), so the elongation test is the one that catches it.
    if (region.kind != PlaceKind::kCorridor &&
        region.elongation < cfg.corridor_aspect)
        return r;

    const int W = snap.width;
    const int H = snap.height;
    const double res = snap.resolution_m;
    const std::size_t N = static_cast<std::size_t>(W) * H;
    const std::vector<char> in_region = region_mask(snap, region);

    std::vector<int> cells;
    cells.reserve(region.cell_region.size());
    for (std::size_t i = 0; i < N; ++i)
        if (in_region[i]) cells.push_back(static_cast<int>(i));
    if (cells.size() < 2) return r;

    auto world = [&](int i) {
        return Point2d{snap.origin_x + (i % W + 0.5) * res,
                       snap.origin_y + (i / W + 0.5) * res};
    };

    // A pinned terminal is honoured only if starting there actually reaches the
    // place. If it has landed on a detached fragment, most of the region would
    // come back at infinite distance and centering would be dead everywhere, so
    // fall through to a fresh sweep instead.
    int pinned = -1;
    if (preferred_start) {
        const int cx = static_cast<int>(
            std::floor((preferred_start->x - snap.origin_x) / res));
        const int cy = static_cast<int>(
            std::floor((preferred_start->y - snap.origin_y) / res));
        if (cx >= 0 && cy >= 0 && cx < W && cy < H &&
            in_region[static_cast<std::size_t>(cy) * W + cx]) {
            const int idx = static_cast<int>(static_cast<std::size_t>(cy) * W + cx);
            std::vector<double> probe(N, kInf);
            geodesic_relax(in_region, W, H, res, {idx}, probe);
            std::size_t reached = 0;
            for (int i : cells)
                if (probe[i] != kInf) ++reached;
            if (reached * 2 >= cells.size()) {
                pinned = idx;
                r.along = std::move(probe);
            }
        }
    }

    // Double sweep: the cell farthest from an arbitrary start is one end of the
    // geodesic diameter, and the cell farthest from THAT is the other end.
    auto sweep = [&](int from, std::vector<double>& dist) {
        dist.assign(N, kInf);
        geodesic_relax(in_region, W, H, res, {from}, dist);
        int best = from;
        for (int i : cells) {
            if (dist[i] == kInf) continue;
            if (dist[best] == kInf || dist[i] > dist[best] ||
                (dist[i] == dist[best] && i < best))
                best = i;
        }
        return best;
    };
    int a = pinned;
    int b = -1;
    if (a < 0) {
        // Start the sweep on the LARGEST component, not on cells.front(). A
        // place's cell_region can be disconnected -- connector carving punches
        // holes (connectors.cpp) -- and cells.front() is the raster-first cell,
        // which is exactly where a detached speckle island lands. Sweeping from
        // an island leaves `along` infinite for the real place and silently
        // disables centering everywhere.
        std::vector<double> scratch(N, kInf);
        int biggest_seed = cells.front();
        {
            std::vector<char> visited(N, 0);
            std::size_t biggest = 0;
            for (int seed : cells) {
                if (visited[seed]) continue;
                std::fill(scratch.begin(), scratch.end(), kInf);
                geodesic_relax(in_region, W, H, res, {seed}, scratch);
                std::size_t size = 0;
                for (int i : cells)
                    if (scratch[i] != kInf) {
                        visited[i] = 1;
                        ++size;
                    }
                if (size > biggest) {
                    biggest = size;
                    biggest_seed = seed;
                }
            }
        }
        const int a0 = sweep(biggest_seed, scratch);
        std::vector<double> from_a;
        const int b0 = sweep(a0, from_a);
        if (a0 == b0) return r;  // degenerate: no extent to traverse

        // Break the tie between the two diameter endpoints in world order. This
        // is only a tie-break, NOT a stability guarantee: it holds the direction
        // when the endpoint PAIR is stable, but on a place that loops the
        // diameter is not unique and both endpoints migrate together as the map
        // grows, so the choice can land on the other arm. Pass the previous
        // terminal as `preferred_start` if the direction has to hold.
        const Point2d wa = world(a0);
        const Point2d wb = world(b0);
        const bool swap_ends =
            (wb.x < wa.x) || (wb.x == wa.x && wb.y < wa.y);
        const int a_raw = swap_ends ? b0 : a0;
        const int b_raw = swap_ends ? a0 : b0;

        // The geodesic diameter of a corridor runs CORNER to CORNER, not end to
        // end along its middle, so the raw endpoints sit against a wall.
        // Measuring `along` from a corner makes its level sets arcs centered on
        // that corner rather than cross-sections of the corridor, which skews
        // the lateral centering near the ends. Pull each terminal onto the local
        // ridge of the region's own depth field -- the centerline at that end.
        const std::vector<double> depth = region_depth(in_region, W, H, res);
        // The radius has to reach the centerline from a CORNER, which is a
        // half-width away diagonally (~sqrt(2) * clearance), not a half-width
        // straight in. Undershooting leaves the terminal off-axis, and because
        // every along-value is measured from it, the level sets come out as
        // diagonal stripes instead of cross-sections -- which quietly defeats
        // the lateral centering further down.
        const double snap_r = std::max(4.0 * res, 1.5 * region.clearance_m);
        auto snap_to_middle = [&](int raw) {
            const int rx = raw % W;
            const int ry = raw / W;
            const int rad = static_cast<int>(std::ceil(snap_r / res));
            int best = raw;
            double best_depth = depth[raw];
            double best_d2 = 0.0;
            for (int dy = -rad; dy <= rad; ++dy) {
                for (int dx = -rad; dx <= rad; ++dx) {
                    const int nx = rx + dx, ny = ry + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    const double d2 = static_cast<double>(dx) * dx + dy * dy;
                    if (d2 * res * res > snap_r * snap_r) continue;
                    const std::size_t ni = static_cast<std::size_t>(ny) * W + nx;
                    if (!in_region[ni] || depth[ni] == kInf) continue;
                    // Deepest cell wins; among equals stay as close to the raw
                    // endpoint as possible so the terminal remains AT the end.
                    if (depth[ni] > best_depth ||
                        (depth[ni] == best_depth &&
                         (d2 < best_d2 || (d2 == best_d2 &&
                                           static_cast<int>(ni) < best)))) {
                        best = static_cast<int>(ni);
                        best_depth = depth[ni];
                        best_d2 = d2;
                    }
                }
            }
            return best;
        };
        a = snap_to_middle(a_raw);
        b = snap_to_middle(b_raw);

        r.along.assign(N, kInf);
        geodesic_relax(in_region, W, H, res, {a}, r.along);
    }
    // On the pinned path `along` is already filled; the far end is just its
    // maximum, which is what the sweep would have found anyway.
    if (b < 0) {
        b = a;
        for (int i : cells) {
            if (r.along[i] == kInf) continue;
            if (r.along[b] == kInf || r.along[i] > r.along[b] ||
                (r.along[i] == r.along[b] && i < b))
                b = i;
        }
    }
    r.length_m = r.along[b] == kInf ? 0.0 : r.along[b];

    r.corridor_like = true;
    r.terminal_a = {a % W, a / W};
    r.terminal_b = {b % W, b / W};
    r.terminal_a_world = world(a);
    r.terminal_b_world = world(b);
    // Centering a point on the axis is free only while the corridor is narrow
    // enough that a centerline point still reaches its walls. With points a
    // spacing apart along the axis, the worst-covered cell sits at half a
    // spacing longitudinally and a half-width h laterally, so the requirement
    // is sqrt(h^2 + (s/2)^2) <= s, i.e. h <= 0.866*s. 0.8 keeps a margin.
    r.center_on_axis = region.clearance_m <= 0.8 * cfg.spacing_m;
    return r;
}

std::vector<std::size_t> order_route(const PlaceGraphSnapshot& snap,
                                     const PlaceRegion& region,
                                     const std::vector<Point2d>& points,
                                     const PlaceRoute* route,
                                     std::vector<double>* route_position_m) {
    const std::size_t k = points.size();
    std::vector<std::size_t> order;
    if (route_position_m) route_position_m->assign(k, -1.0);
    if (k == 0) return order;
    if (snap.width <= 0 || snap.height <= 0 || snap.resolution_m <= 0.0 ||
        region.cell_region.empty()) {
        for (std::size_t i = 0; i < k; ++i) order.push_back(i);
        return order;
    }

    const int W = snap.width;
    const int H = snap.height;
    const double res = snap.resolution_m;
    const std::size_t N = static_cast<std::size_t>(W) * H;
    const std::vector<char> in_region = region_mask(snap, region);

    std::vector<int> cell(k, -1);
    std::vector<std::size_t> routable, stray;
    for (std::size_t i = 0; i < k; ++i) {
        const int cx =
            static_cast<int>(std::floor((points[i].x - snap.origin_x) / res));
        const int cy =
            static_cast<int>(std::floor((points[i].y - snap.origin_y) / res));
        if (cx >= 0 && cy >= 0 && cx < W && cy < H &&
            in_region[static_cast<std::size_t>(cy) * W + cx]) {
            cell[i] = static_cast<int>(static_cast<std::size_t>(cy) * W + cx);
            routable.push_back(i);
        } else {
            stray.push_back(i);
        }
    }
    if (routable.size() <= 1) {
        order = routable;
        if (route_position_m && !routable.empty())
            (*route_position_m)[routable[0]] = 0.0;
        order.insert(order.end(), stray.begin(), stray.end());
        return order;
    }

    // Pairwise geodesic distances: one Dijkstra per point.
    const std::size_t m = routable.size();
    std::vector<std::vector<double>> dmat(m, std::vector<double>(m, kInf));
    for (std::size_t i = 0; i < m; ++i) {
        std::vector<double> d(N, kInf);
        geodesic_relax(in_region, W, H, res, {cell[routable[i]]}, d);
        for (std::size_t j = 0; j < m; ++j) dmat[i][j] = d[cell[routable[j]]];
    }

    // Start at the end of the place, so a corridor is walked end to end rather
    // than from the middle outward.
    std::size_t root = 0;
    if (route && route->corridor_like && !route->along.empty()) {
        double best = kInf;
        for (std::size_t i = 0; i < m; ++i) {
            const double a = route->along[cell[routable[i]]];
            if (a < best ||
                (a == best && cell[routable[i]] < cell[routable[root]])) {
                best = a;
                root = i;
            }
        }
    } else {
        for (std::size_t i = 1; i < m; ++i)
            if (cell[routable[i]] < cell[routable[root]]) root = i;
    }

    // Prim MST over the geodesic distances.
    std::vector<char> in_tree(m, 0);
    std::vector<double> best_w(m, kInf);
    std::vector<int> parent(m, -1);
    best_w[root] = 0.0;
    for (std::size_t step = 0; step < m; ++step) {
        int u = -1;
        for (std::size_t i = 0; i < m; ++i) {
            if (in_tree[i]) continue;
            if (u < 0 || best_w[i] < best_w[u]) u = static_cast<int>(i);
        }
        if (u < 0 || best_w[u] == kInf) break;  // rest is a disjoint component
        in_tree[u] = 1;
        for (std::size_t v = 0; v < m; ++v)
            if (!in_tree[v] && dmat[u][v] < best_w[v]) {
                best_w[v] = dmat[u][v];
                parent[v] = u;
            }
    }

    std::vector<std::vector<std::size_t>> kids(m);
    for (std::size_t i = 0; i < m; ++i)
        if (parent[i] >= 0) kids[parent[i]].push_back(i);
    // Nearest child first, so walking into a branch is monotone.
    for (std::size_t i = 0; i < m; ++i)
        std::sort(kids[i].begin(), kids[i].end(),
                  [&](std::size_t a, std::size_t b) {
                      if (dmat[i][a] != dmat[i][b])
                          return dmat[i][a] < dmat[i][b];
                      return cell[routable[a]] < cell[routable[b]];
                  });

    std::vector<char> seen(m, 0);
    std::vector<std::size_t> stack{root};
    std::vector<std::size_t> visit;
    while (!stack.empty()) {
        const std::size_t u = stack.back();
        stack.pop_back();
        if (seen[u]) continue;
        seen[u] = 1;
        visit.push_back(u);
        for (auto it = kids[u].rbegin(); it != kids[u].rend(); ++it)
            stack.push_back(*it);
    }
    for (std::size_t i = 0; i < m; ++i)
        if (!seen[i]) visit.push_back(i);  // unreachable component

    // Accumulate the distance actually walked. Using the geodesic distance
    // between consecutive emitted points charges backtracking out of a branch
    // to the route, which is what the robot really drives.
    double acc = 0.0;
    for (std::size_t n = 0; n < visit.size(); ++n) {
        if (n > 0) {
            const double hop = dmat[visit[n - 1]][visit[n]];
            if (hop != kInf) acc += hop;
        }
        const std::size_t orig = routable[visit[n]];
        order.push_back(orig);
        if (route_position_m) (*route_position_m)[orig] = acc;
    }
    order.insert(order.end(), stray.begin(), stray.end());
    return order;
}

std::vector<CoveragePoint> sample_coverage(
    const PlaceGraphSnapshot& snap, const MapSnapshot& map,
    const PlaceRegion& region, const std::vector<Point2d>& seeds,
    const CoverageConfig& cfg,
    const std::function<bool(double, double)>& pose_ok,
    const PlaceRoute* route) {
    std::vector<CoveragePoint> out;
    if (region.cell_region.empty()) return out;
    if (snap.width <= 0 || snap.height <= 0 || snap.resolution_m <= 0.0)
        return out;
    if (map.width <= 0 || map.height <= 0 || map.resolution_m <= 0.0) return out;
    if (map.cells.size() !=
        static_cast<std::size_t>(map.width) * map.height)
        return out;
    if (cfg.spacing_m <= 0.0) return out;

    const int W = snap.width;
    const int H = snap.height;
    const double res = snap.resolution_m;
    const std::size_t N = static_cast<std::size_t>(W) * H;

    const std::vector<char> in_region = region_mask(snap, region);

    // Traversal coordinate: centers points across the corridor and orders the
    // result end to end. Computed here only if the caller did not supply one.
    PlaceRoute owned_route;
    if (!route) {
        owned_route = compute_place_route(snap, region, cfg);
        route = &owned_route;
    }

    // Wall clearance is measured against the OCCUPANCY map, not the region
    // boundary: a doorway into the next room is open space, and only real walls
    // and unknown cells count against the margin. (Same convention as
    // sampling.cpp's cell_has_clearance.)
    std::vector<char> blocked(map.cells.size(), 0);
    for (std::size_t i = 0; i < map.cells.size(); ++i)
        blocked[i] = map.cells[i] != CellState::kFree ? 1 : 0;
    const std::vector<double> sq = squared_edt(blocked, map.width, map.height);

    const double wx0 = snap.origin_x + 0.5 * res;
    const double wy0 = snap.origin_y + 0.5 * res;
    auto world_x = [&](int cx) { return wx0 + cx * res; };
    auto world_y = [&](int cy) { return wy0 + cy * res; };

    // The segmentation grid may be a downsampled view of the map, so go through
    // world coordinates rather than assuming the two grids line up.
    auto clearance_at = [&](int cx, int cy) -> double {
        const int mx = static_cast<int>(
            std::floor((world_x(cx) - map.origin_x) / map.resolution_m));
        const int my = static_cast<int>(
            std::floor((world_y(cy) - map.origin_y) / map.resolution_m));
        if (!map.in_bounds(mx, my)) return 0.0;
        const double d_wall =
            std::sqrt(sq[static_cast<std::size_t>(my) * map.width + mx]) *
            map.resolution_m;
        // Out of bounds counts as blocked too; the nearest such cell beyond
        // each edge sits one index past it.
        const int edge_cells = std::min(std::min(mx + 1, map.width - mx),
                                        std::min(my + 1, map.height - my));
        return std::min(d_wall, edge_cells * map.resolution_m);
    };

    // Eligibility is static, so evaluate the (possibly expensive) predicate
    // once per region cell rather than on every argmax scan.
    std::vector<char> eligible(N, 0);
    std::vector<double> clearance(N, 0.0);
    std::vector<int> cells;
    cells.reserve(region.cell_region.size());
    for (const CellCoord& c : region.cell_region) {
        if (c.x < 0 || c.y < 0 || c.x >= W || c.y >= H) continue;
        const std::size_t i = static_cast<std::size_t>(c.y) * W + c.x;
        clearance[i] = clearance_at(c.x, c.y);
        eligible[i] = pose_ok ? (pose_ok(world_x(c.x), world_y(c.y)) ? 1 : 0)
                              : (clearance[i] > cfg.min_clearance_m ? 1 : 0);
        cells.push_back(static_cast<int>(i));
    }
    if (cells.empty()) return out;

    // Geodesic distance to the nearest committed point, lowered in place as
    // points are added (an incremental multi-source Dijkstra, so the field is
    // never rebuilt from scratch).
    std::vector<double> dist(N, kInf);
    auto relax = [&](const std::vector<int>& sources) {
        geodesic_relax(in_region, W, H, res, sources, dist);
    };

    std::vector<int> sources;
    for (const Point2d& s : seeds) {
        const int cx = static_cast<int>(std::floor((s.x - snap.origin_x) / res));
        const int cy = static_cast<int>(std::floor((s.y - snap.origin_y) / res));
        if (cx < 0 || cy < 0 || cx >= W || cy >= H) continue;
        const std::size_t i = static_cast<std::size_t>(cy) * W + cx;
        if (in_region[i]) sources.push_back(static_cast<int>(i));
    }
    if (!sources.empty()) relax(sources);

    // Fallback nudge radius. In a corridor it has to span the width: at an end
    // cap the worst-covered cell is a corner, and a radius smaller than the
    // half-width sees only the handful of wall-adjacent cells beside it.
    const double nudge_m =
        route->center_on_axis
            ? std::max(cfg.ridge_nudge_m, region.clearance_m)
            : cfg.ridge_nudge_m;
    const int nudge_r = static_cast<int>(std::ceil(nudge_m / res));
    // Cells ruled out as point *centers* (nothing usable in their neighbourhood).
    // Every iteration either commits a point or excludes at least one cell, so
    // the loop terminates.
    std::vector<char> excluded(N, 0);
    // Scratch for the bounded flood used to keep the lateral nudge local.
    std::vector<double> near_far(N, kInf);

    auto separated = [&](int cx, int cy) {
        const double px = world_x(cx);
        const double py = world_y(cy);
        for (const CoveragePoint& q : out) {
            const double dx = px - q.point.x;
            const double dy = py - q.point.y;
            if (dx * dx + dy * dy <
                cfg.min_separation_m * cfg.min_separation_m)
                return false;
        }
        return true;
    };

    while (static_cast<int>(out.size()) < cfg.max_points) {
        // The worst-covered cell. Ties break on clearance then flat index, so
        // the choice is deterministic -- and with no seeds at all (every
        // distance still infinite) this naturally starts at the region's widest
        // interior point.
        int far = -1;
        for (int i : cells) {
            if (excluded[i]) continue;
            if (far < 0) {
                far = i;
                continue;
            }
            if (dist[i] > dist[far]) {
                far = i;
            } else if (dist[i] == dist[far]) {
                if (clearance[i] > clearance[far] ||
                    (clearance[i] == clearance[far] && i < far))
                    far = i;
            }
        }
        if (far < 0 || dist[far] < cfg.spacing_m) break;

        // Shift onto the local clearance ridge, but only across cells that keep
        // BOTH the geodesic spacing and the euclidean separation. Centering
        // that ignores those constraints is exactly how the lattice sampler
        // bunched neighbouring points together.
        const int fx = far % W;
        const int fy = far / W;
        int pick = -1;      // satisfies spacing AND separation
        int fallback = -1;  // satisfies spacing only

        // Lateral centering. Among eligible cells in the SAME cross-section --
        // the same distance along the corridor -- take the one with the most
        // wall clearance, i.e. the centerline. Restricting the move to a level
        // set of `along` makes it purely lateral: the point centers without
        // sliding ALONG the corridor, and sliding along is exactly what let the
        // old lattice sampler pull neighbouring points together.
        //
        // The bar here is min_separation_m rather than spacing_m: the centered
        // cell is nearer the previous point than the off-axis one was (it is on
        // the short side of the triangle), so demanding a full spacing would
        // reject the centerline and defeat the whole point. Landing slightly
        // closer than nominal costs nothing -- coverage is re-measured from the
        // committed pose -- while the bar still stops a point collapsing onto
        // one already placed.
        if (route->center_on_axis) {
            const double t = route->along[far];
            if (t != kInf) {
                // Reachability from `far`, flooded only as far as the nudge can
                // reach. Bounded, so this costs a few hundred cells rather than
                // a pass over the place.
                std::fill(near_far.begin(), near_far.end(), kInf);
                geodesic_relax(in_region, W, H, res, {far}, near_far, nudge_m);
                for (int i : cells) {
                    if (!eligible[i] || dist[i] < cfg.min_separation_m) continue;
                    const double ai = route->along[i];
                    if (ai == kInf || std::fabs(ai - t) > cfg.cross_section_tol_m)
                        continue;
                    // A matching along-value is necessary but not sufficient:
                    // on a place that loops, the two sides of the ring are the
                    // same distance from the terminal, so an unbounded scan can
                    // "center" a point by teleporting it across the loop. The
                    // bound has to be GEODESIC -- two cells a nudge apart in a
                    // straight line can be far apart across a wall stub, and a
                    // pick farther than spacing_m from `far` does not cover the
                    // cell it was chosen for, so `far` is simply re-selected
                    // next iteration and the point count inflates.
                    if (near_far[i] > nudge_m) continue;
                    const auto better = [&](int cur) {
                        return cur < 0 || clearance[i] > clearance[cur] ||
                               (clearance[i] == clearance[cur] && i < cur);
                    };
                    if (better(fallback)) fallback = i;
                    if (separated(i % W, i / W) && better(pick)) pick = i;
                }
            }
        }

        // Local ridge nudge, used when the place is not centerline-eligible or
        // its cross-section yielded nothing usable.
        if (pick < 0 && fallback < 0) {
            for (int dy = -nudge_r; dy <= nudge_r; ++dy) {
                for (int dx = -nudge_r; dx <= nudge_r; ++dx) {
                    const int nx = fx + dx;
                    const int ny = fy + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    if (std::hypot(dx * res, dy * res) > nudge_m) continue;
                    const std::size_t ni = static_cast<std::size_t>(ny) * W + nx;
                    if (!in_region[ni] || !eligible[ni]) continue;
                    if (dist[ni] < cfg.spacing_m) continue;
                    const auto better = [&](int cur) {
                        return cur < 0 || clearance[ni] > clearance[cur] ||
                               (clearance[ni] == clearance[cur] &&
                                static_cast<int>(ni) < cur);
                    };
                    if (better(fallback)) fallback = static_cast<int>(ni);
                    if (separated(nx, ny) && better(pick))
                        pick = static_cast<int>(ni);
                }
            }
        }
        // Coverage outranks separation. min_separation_m is a cosmetic floor --
        // it stops points that drifted together from reading as a cluster --
        // whereas leaving a region cell uncovered is the actual defect we are
        // here to fix. Where the two conflict (a thin wall stub makes two cells
        // euclidean-close but geodesically far, so both genuinely need their
        // own point) coverage wins, and the publish-time separation pass in
        // AnchorRegistry::reconcile remains free to withhold one.
        if (pick < 0) pick = fallback;

        if (pick < 0) {
            // Nothing in this neighbourhood can host a point (all too close to
            // a wall, or too close to an existing point across a wall stub).
            // Exclude the whole disc: any other center in it would search the
            // same cells and reach the same conclusion.
            for (int dy = -nudge_r; dy <= nudge_r; ++dy) {
                for (int dx = -nudge_r; dx <= nudge_r; ++dx) {
                    const int nx = fx + dx;
                    const int ny = fy + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    if (std::hypot(dx * res, dy * res) > nudge_m) continue;
                    excluded[static_cast<std::size_t>(ny) * W + nx] = 1;
                }
            }
            excluded[far] = 1;  // guarantee progress even if the disc is empty
            continue;
        }

        CoveragePoint cp;
        cp.cell = {pick % W, pick / W};
        cp.point.x = world_x(cp.cell.x);
        cp.point.y = world_y(cp.cell.y);
        cp.clearance_m = clearance[pick];
        if (route->corridor_like) cp.route_position_m = route->along[pick];
        out.push_back(cp);
        relax({pick});
    }

    // Order the result as a drivable route. Selection order is
    // worst-covered-first, which bisects the place rather than walking it.
    std::vector<Point2d> pts;
    pts.reserve(out.size());
    for (const CoveragePoint& p : out) pts.push_back(p.point);
    std::vector<double> pos;
    const std::vector<std::size_t> ord =
        order_route(snap, region, pts, route, &pos);
    std::vector<CoveragePoint> ordered;
    ordered.reserve(out.size());
    for (std::size_t i : ord) {
        CoveragePoint cp = out[i];
        cp.route_position_m = pos[i];
        ordered.push_back(cp);
    }
    out.swap(ordered);
    return out;
}

}  // namespace place_graph
