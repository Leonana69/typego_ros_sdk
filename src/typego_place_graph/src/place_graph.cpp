#include "typego_place_graph/place_graph.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace typego_place_graph {

const char* to_string(PlaceKind kind) {
    switch (kind) {
        case PlaceKind::kRoom: return "room";
        case PlaceKind::kCorridor: return "corridor";
        case PlaceKind::kOpenArea: return "open_area";
        case PlaceKind::kPortal: return "portal";
        case PlaceKind::kOpenTransition: return "open_transition";
        case PlaceKind::kJunction: return "junction";
        case PlaceKind::kFrontierRegion: return "frontier_region";
    }
    return "unknown";
}

const PlaceRegion* PlaceGraphSnapshot::place_at_world(double wx,
                                                     double wy) const {
    int cx = static_cast<int>((wx - origin_x) / resolution_m);
    int cy = static_cast<int>((wy - origin_y) / resolution_m);
    int idx = place_index_of(cx, cy);
    return idx < 0 ? nullptr : &places[idx];
}

const std::string* PlaceGraphSnapshot::place_id_at_world(double wx,
                                                        double wy) const {
    const PlaceRegion* p = place_at_world(wx, wy);
    return p ? &p->place_id : nullptr;
}

namespace {

std::string make_id(PlaceKind kind, int n) {
    return std::string(to_string(kind)) + "_" + std::to_string(n);
}

double cell_to_world_x(int cx, double origin, double res) {
    return origin + (cx + 0.5) * res;
}
double cell_to_world_y(int cy, double origin, double res) {
    return origin + (cy + 0.5) * res;
}

}  // namespace

PlaceGraphSnapshot build_place_graph(const MapSnapshot& map,
                                     const Config& cfg) {
    PlaceGraphSnapshot out;
    auto seg = build_segmentation_grid(map, cfg);
    out.width = seg.width;
    out.height = seg.height;
    out.resolution_m = seg.resolution_m;
    out.origin_x = seg.origin_x;
    out.origin_y = seg.origin_y;
    out.map_version = map.map_version;
    out.place_assignment.assign(seg.width * seg.height, kNoPlace);
    if (seg.width == 0 || seg.height == 0) return out;

    auto cores = extract_cores(seg, cfg);
    if (cores.cores.empty()) return out;
    auto ws = watershed_assign(seg, cores);
    auto conns = extract_connectors(seg, cores, ws, cfg);

    const int w = seg.width;
    const int h = seg.height;
    double cell_area = seg.resolution_m * seg.resolution_m;

    // Per-kind ID counters. The matching pass will refresh these from
    // the previous snapshot's counters; on a cold build the counters
    // are just per-kind incrementing.
    std::unordered_map<PlaceKind, int> next_id;
    auto mint = [&](PlaceKind k) {
        int n = next_id[k]++;
        return make_id(k, n);
    };

    // Step 5 (materialize cores into PlaceRegions and the assignment
    // grid). Connectors overwrite core_assignment, so we paint cores
    // first then connectors.
    std::vector<int> core_to_place(cores.cores.size(), -1);
    for (int ci = 0; ci < static_cast<int>(cores.cores.size()); ++ci) {
        PlaceRegion p;
        p.kind = cores.cores[ci].kind;
        p.place_id = mint(p.kind);
        p.clearance_m = cores.cores[ci].clearance_m;
        p.peak.x = cell_to_world_x(cores.cores[ci].seed_x, seg.origin_x,
                                   seg.resolution_m);
        p.peak.y = cell_to_world_y(cores.cores[ci].seed_y, seg.origin_y,
                                   seg.resolution_m);
        out.places.push_back(p);
        core_to_place[ci] = static_cast<int>(out.places.size()) - 1;
    }

    // Assignment grid: core cells first.
    for (int i = 0; i < w * h; ++i) {
        int c = ws.core_assignment[i];
        if (c < 0) continue;
        out.place_assignment[i] =
            static_cast<std::uint16_t>(core_to_place[c]);
    }

    // Connectors overlay. Each connector becomes its own PlaceRegion;
    // its cells overwrite the core assignment. Owning cells are
    // excluded from the parent core's cell_region (we recompute below).
    // approach_yaw here is the bare perpendicular to the connector
    // centerline; the post-pass below flips it to point toward the
    // larger adjacent room once cell_region sizes are known.
    constexpr double kPi = 3.14159265358979323846;
    for (const auto& cc : conns) {
        PlaceRegion p;
        p.kind = cc.kind;
        p.place_id = mint(p.kind);
        p.width_m = cc.width_m;
        p.clearance_m = cc.clearance_m;
        p.centroid = cc.centroid;
        p.peak = cc.centroid;
        std::unordered_set<std::string> adj_set;
        for (int core_idx : cc.endpoint_cores) {
            const auto& parent = out.places[core_to_place[core_idx]];
            adj_set.insert(parent.place_id);
        }
        p.adjacent_place_ids.assign(adj_set.begin(), adj_set.end());
        // Stable order for tests / determinism.
        std::sort(p.adjacent_place_ids.begin(), p.adjacent_place_ids.end());

        if (cc.kind != PlaceKind::kJunction) {
            double yaw = cc.axis_angle + kPi / 2.0;
            while (yaw > kPi) yaw -= 2.0 * kPi;
            while (yaw < -kPi) yaw += 2.0 * kPi;
            p.approach_yaw = yaw;
        }
        out.places.push_back(p);
        int pi = static_cast<int>(out.places.size()) - 1;
        for (const auto& c : cc.cells) {
            out.place_assignment[c.y * w + c.x] =
                static_cast<std::uint16_t>(pi);
        }
    }

    // Now build cell_region and per-place stats. Walks the assignment
    // grid once; allocates one entry per filled cell.
    std::vector<std::vector<CellCoord>> cells_of(out.places.size());
    std::vector<int> bbox_minx(out.places.size(), w),
        bbox_maxx(out.places.size(), -1),
        bbox_miny(out.places.size(), h),
        bbox_maxy(out.places.size(), -1);
    std::vector<int> perimeter(out.places.size(), 0);
    std::vector<int> frontier(out.places.size(), 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            int i = y * w + x;
            std::uint16_t pid = out.place_assignment[i];
            if (pid == kNoPlace) continue;
            cells_of[pid].push_back({x, y});
            bbox_minx[pid] = std::min(bbox_minx[pid], x);
            bbox_maxx[pid] = std::max(bbox_maxx[pid], x);
            bbox_miny[pid] = std::min(bbox_miny[pid], y);
            bbox_maxy[pid] = std::max(bbox_maxy[pid], y);
            // perimeter and frontier
            bool on_perim = false, touches_unknown = false;
            static const int dxs[] = {1, -1, 0, 0};
            static const int dys[] = {0, 0, 1, -1};
            for (int k = 0; k < 4; ++k) {
                int nx = x + dxs[k], ny = y + dys[k];
                if (nx < 0 || ny < 0 || nx >= w || ny >= h) {
                    on_perim = true;
                    continue;
                }
                int ni = ny * w + nx;
                std::uint16_t opid = out.place_assignment[ni];
                if (opid != pid) {
                    on_perim = true;
                    if (seg.unknown[ni]) touches_unknown = true;
                }
            }
            if (on_perim) perimeter[pid]++;
            if (touches_unknown) frontier[pid]++;
        }
    }

    for (size_t pi = 0; pi < out.places.size(); ++pi) {
        auto& p = out.places[pi];
        p.cell_region = std::move(cells_of[pi]);
        p.area_m2 = p.cell_region.size() * cell_area;
        if (!p.cell_region.empty()) {
            double sx = 0, sy = 0;
            for (const auto& c : p.cell_region) { sx += c.x; sy += c.y; }
            // Connectors already have an explicit centroid; keep it.
            if (p.centroid.x == 0.0 && p.centroid.y == 0.0) {
                p.centroid.x = cell_to_world_x(
                    static_cast<int>(sx / p.cell_region.size()),
                    seg.origin_x, seg.resolution_m);
                p.centroid.y = cell_to_world_y(
                    static_cast<int>(sy / p.cell_region.size()),
                    seg.origin_y, seg.resolution_m);
            }
        }
        p.frontier_ratio =
            perimeter[pi] > 0
                ? static_cast<double>(frontier[pi]) / perimeter[pi]
                : 0.0;
        p.provisional = p.frontier_ratio > cfg.frontier_lock_ratio &&
                        p.kind != PlaceKind::kFrontierRegion;
        // Re-classify the (now boundary-adjusted) non-connector kinds
        // from their final cell_region shape: an open_area whose
        // connectors carved off a wing might fall back to room. Skip
        // connectors and frontier_regions; their kinds are not shape-
        // derived.
        if (p.kind == PlaceKind::kRoom || p.kind == PlaceKind::kCorridor ||
            p.kind == PlaceKind::kOpenArea) {
            int bw = bbox_maxx[pi] - bbox_minx[pi] + 1;
            int bh = bbox_maxy[pi] - bbox_miny[pi] + 1;
            double aspect = static_cast<double>(std::max(bw, bh)) /
                            std::max(1, std::min(bw, bh));
            PlaceKind k = PlaceKind::kRoom;
            if (p.area_m2 >= cfg.a_open_m2) k = PlaceKind::kOpenArea;
            else if (aspect >= cfg.corridor_aspect) k = PlaceKind::kCorridor;
            // Keep the original ID prefix even if the kind shifts;
            // matching will re-mint or refresh as needed. Just update
            // the kind in place.
            p.kind = k;
        }
    }

    // Orient connector approach_yaw toward the larger adjacent place.
    // Deferred until here because we need final cell_region sizes for
    // each adjacent place (set in the loop above). Adjacency is keyed
    // by cold place_id since matching has not run yet; build a tiny
    // index for O(1) lookup.
    std::unordered_map<std::string, int> id_to_index;
    id_to_index.reserve(out.places.size());
    for (size_t i = 0; i < out.places.size(); ++i) {
        id_to_index[out.places[i].place_id] = static_cast<int>(i);
    }
    for (auto& p : out.places) {
        if (p.kind != PlaceKind::kPortal &&
            p.kind != PlaceKind::kOpenTransition) continue;
        if (!p.approach_yaw.has_value()) continue;
        int larger = -1;
        for (const auto& adj_id : p.adjacent_place_ids) {
            auto it = id_to_index.find(adj_id);
            if (it == id_to_index.end()) continue;
            int i = it->second;
            if (larger < 0 ||
                out.places[i].cell_region.size() >
                    out.places[larger].cell_region.size()) {
                larger = i;
            }
        }
        if (larger < 0) continue;
        double yaw = *p.approach_yaw;
        double dir_x = out.places[larger].centroid.x - p.centroid.x;
        double dir_y = out.places[larger].centroid.y - p.centroid.y;
        if (std::cos(yaw) * dir_x + std::sin(yaw) * dir_y < 0.0) {
            yaw += kPi;
            while (yaw > kPi) yaw -= 2.0 * kPi;
        }
        p.approach_yaw = yaw;
    }

    out.id_counters = next_id;
    return out;
}

}  // namespace typego_place_graph
