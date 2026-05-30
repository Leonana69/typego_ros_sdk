#include "place_graph/matching.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace place_graph {

namespace {

bool is_connector(PlaceKind k) {
    return k == PlaceKind::kPortal || k == PlaceKind::kOpenTransition ||
           k == PlaceKind::kJunction;
}

// Copy operator/VLM/LLM-set fields from a previous place onto the fresh
// place that inherited its ID. The geometry-only pipeline leaves these
// at defaults, so without this step every refresh would wipe semantic
// state — and operator/VLM labels are the whole point of carrying IDs
// forward.
void propagate_semantic_fields(const PlaceRegion& from, PlaceRegion& to) {
    to.semantic_label = from.semantic_label;
    to.instance_label = from.instance_label;
    to.operator_label = from.operator_label;
    to.label_locked = from.label_locked;
    to.confidence = from.confidence;
    to.source = from.source;
    to.objects = from.objects;
    to.affordances = from.affordances;
    to.summary = from.summary;
}

// Map a (prev) cell coordinate into the (fresh) grid by going through
// world coordinates. Returns -1 if it falls outside the fresh grid.
// The two grids share resolution in any given pipeline run because
// both come from build_segmentation_grid with the same Config, but
// origin and size can differ between refreshes.
int remap_cell(const PlaceGraphSnapshot& prev,
               const PlaceGraphSnapshot& fresh,
               int px, int py) {
    double wx = prev.origin_x + (px + 0.5) * prev.resolution_m;
    double wy = prev.origin_y + (py + 0.5) * prev.resolution_m;
    int fx = static_cast<int>(std::floor(
        (wx - fresh.origin_x) / fresh.resolution_m));
    int fy = static_cast<int>(std::floor(
        (wy - fresh.origin_y) / fresh.resolution_m));
    if (fx < 0 || fy < 0 || fx >= fresh.width || fy >= fresh.height) return -1;
    return fy * fresh.width + fx;
}

struct PairScore {
    int prev_idx;
    int fresh_idx;
    double score;
};

// Build cell-overlap IoU between every (non-connector) prev place and
// (non-connector) fresh place. Connectors are matched separately.
std::vector<PairScore> score_non_connector_pairs(
    const PlaceGraphSnapshot& prev,
    const PlaceGraphSnapshot& fresh,
    const Config& cfg) {
    // Pre-bucket fresh place cell sets so we can look up the fresh
    // place at any cell in O(1). The fresh snapshot's place_assignment
    // already does this, so we just consult it after remapping.
    std::vector<int> overlap_count;  // prev_idx * F + fresh_idx
    int P = static_cast<int>(prev.places.size());
    int F = static_cast<int>(fresh.places.size());
    overlap_count.assign(P * F, 0);
    std::vector<int> prev_size(P, 0);
    std::vector<int> fresh_size(F, 0);
    for (int pi = 0; pi < P; ++pi) {
        if (is_connector(prev.places[pi].kind)) continue;
        prev_size[pi] = static_cast<int>(prev.places[pi].cell_region.size());
        for (const auto& c : prev.places[pi].cell_region) {
            int fi = remap_cell(prev, fresh, c.x, c.y);
            if (fi < 0) continue;
            std::uint16_t fpid = fresh.place_assignment[fi];
            if (fpid == kNoPlace) continue;
            if (is_connector(fresh.places[fpid].kind)) continue;
            overlap_count[pi * F + fpid]++;
        }
    }
    for (int fi = 0; fi < F; ++fi) {
        if (is_connector(fresh.places[fi].kind)) continue;
        fresh_size[fi] = static_cast<int>(fresh.places[fi].cell_region.size());
    }
    std::vector<PairScore> pairs;
    for (int pi = 0; pi < P; ++pi) {
        if (is_connector(prev.places[pi].kind)) continue;
        for (int fi = 0; fi < F; ++fi) {
            if (is_connector(fresh.places[fi].kind)) continue;
            int inter = overlap_count[pi * F + fi];
            if (inter == 0) continue;
            int uni = prev_size[pi] + fresh_size[fi] - inter;
            if (uni <= 0) continue;
            double iou = static_cast<double>(inter) / uni;
            double s = iou;
            if (prev.places[pi].kind != fresh.places[fi].kind) {
                s -= cfg.kind_penalty;
            }
            if (s < cfg.iou_match) continue;
            pairs.push_back({pi, fi, s});
        }
    }
    return pairs;
}

}  // namespace

MatchResult match_place_graphs(const PlaceGraphSnapshot& prev,
                               const PlaceGraphSnapshot& fresh,
                               const Config& cfg) {
    MatchResult mr;
    mr.snapshot = fresh;
    // Snapshot the cold-mint IDs every fresh place currently holds.
    // The matching passes below rewrite `place_id` on matched fresh
    // places to inherit prev IDs, but each place's
    // `adjacent_place_ids` still contains cold IDs from
    // build_place_graph. We remap those just before connector matching
    // so adjacency reflects the post-matching world.
    std::vector<std::string> cold_ids;
    cold_ids.reserve(mr.snapshot.places.size());
    for (const auto& p : mr.snapshot.places) cold_ids.push_back(p.place_id);

    // Carry forward counters from prev so fresh IDs continue rather
    // than recycle. The fresh snapshot's id_counters represent only
    // the cold-build naming and will be overwritten in the next step.
    for (const auto& kv : prev.id_counters) {
        auto it = mr.snapshot.id_counters.find(kv.first);
        if (it == mr.snapshot.id_counters.end()) {
            mr.snapshot.id_counters[kv.first] = kv.second;
        } else {
            it->second = std::max(it->second, kv.second);
        }
    }
    auto mint = [&](PlaceKind k) {
        int n = mr.snapshot.id_counters[k]++;
        return std::string(to_string(k)) + "_" + std::to_string(n);
    };

    if (prev.places.empty()) {
        // Cold start: rename every place using the (already cold-
        // started) IDs in mr.snapshot. Emit births.
        for (auto& p : mr.snapshot.places) {
            mr.events.push_back({PlaceEventKind::kBirth, {}, {p.place_id}});
        }
        return mr;
    }

    // Phase 6.2-6.3: non-connector matching via IoU.
    auto pairs = score_non_connector_pairs(prev, mr.snapshot, cfg);
    std::sort(pairs.begin(), pairs.end(), [](const PairScore& a,
                                             const PairScore& b) {
        return a.score > b.score;
    });

    int P = static_cast<int>(prev.places.size());
    int F = static_cast<int>(mr.snapshot.places.size());
    std::vector<int> prev_match(P, -1);
    std::vector<int> fresh_match(F, -1);
    std::unordered_map<int, std::vector<int>> prev_successors;  // for splits
    // First pass: 1:1 greedy matches.
    for (const auto& ps : pairs) {
        if (prev_match[ps.prev_idx] != -1) continue;
        if (fresh_match[ps.fresh_idx] != -1) continue;
        prev_match[ps.prev_idx] = ps.fresh_idx;
        fresh_match[ps.fresh_idx] = ps.prev_idx;
        prev_successors[ps.prev_idx].push_back(ps.fresh_idx);
    }
    // Second pass for splits: any unmatched fresh non-connector
    // whose strongest cell overlap is with an already-matched prev
    // place is a split successor. Use the raw overlap (no
    // kind_penalty, no iou_match threshold) because the threshold is
    // tuned for 1:1 continuity, not for "this is one of the pieces a
    // prior place broke into". The keeper still wins by largest
    // cell_region.
    int F_total = static_cast<int>(mr.snapshot.places.size());
    for (int fi = 0; fi < F_total; ++fi) {
        if (fresh_match[fi] != -1) continue;
        if (is_connector(mr.snapshot.places[fi].kind)) continue;
        // Find prev place with largest cell overlap on this fresh
        // place by remapping the fresh cells through world coords.
        std::unordered_map<int, int> overlap;
        for (const auto& c : mr.snapshot.places[fi].cell_region) {
            // remap fresh -> prev
            double wx = mr.snapshot.origin_x +
                        (c.x + 0.5) * mr.snapshot.resolution_m;
            double wy = mr.snapshot.origin_y +
                        (c.y + 0.5) * mr.snapshot.resolution_m;
            int px = static_cast<int>(
                std::floor((wx - prev.origin_x) / prev.resolution_m));
            int py = static_cast<int>(
                std::floor((wy - prev.origin_y) / prev.resolution_m));
            if (px < 0 || py < 0 || px >= prev.width || py >= prev.height)
                continue;
            std::uint16_t prev_pid = prev.place_assignment[py * prev.width + px];
            if (prev_pid == kNoPlace) continue;
            if (is_connector(prev.places[prev_pid].kind)) continue;
            overlap[prev_pid]++;
        }
        if (overlap.empty()) continue;
        int best_pi = overlap.begin()->first;
        for (const auto& kv : overlap) {
            if (kv.second > overlap[best_pi]) best_pi = kv.first;
        }
        if (prev_match[best_pi] == -1) continue;  // not split, just birth
        fresh_match[fi] = -2;
        prev_successors[best_pi].push_back(fi);
    }

    // Apply prev IDs and propagate semantic-bookkeeping fields. Emit
    // events.
    std::vector<bool> id_assigned(F, false);
    for (int pi = 0; pi < P; ++pi) {
        if (is_connector(prev.places[pi].kind)) continue;
        const auto& po = prev.places[pi];
        auto it = prev_successors.find(pi);
        if (it == prev_successors.end()) {
            // Death candidate; stale tracking handled below.
            continue;
        }
        // Largest successor by cell_region size keeps the ID.
        int keeper = it->second.front();
        for (int fi : it->second) {
            if (mr.snapshot.places[fi].cell_region.size() >
                mr.snapshot.places[keeper].cell_region.size()) {
                keeper = fi;
            }
        }
        mr.snapshot.places[keeper].place_id = po.place_id;
        mr.snapshot.places[keeper].age_refreshes = po.age_refreshes + 1;
        propagate_semantic_fields(po, mr.snapshot.places[keeper]);
        id_assigned[keeper] = true;
        if (it->second.size() == 1) {
            mr.events.push_back(
                {PlaceEventKind::kRefresh, {po.place_id},
                 {mr.snapshot.places[keeper].place_id}});
        } else {
            // 1:N split. Mint new IDs for the non-keeper successors.
            std::vector<std::string> new_ids;
            for (int fi : it->second) {
                if (fi == keeper) continue;
                std::string nid = mint(mr.snapshot.places[fi].kind);
                mr.snapshot.places[fi].place_id = nid;
                mr.snapshot.places[fi].age_refreshes = 0;
                id_assigned[fi] = true;
                new_ids.push_back(nid);
            }
            mr.events.push_back(
                {PlaceEventKind::kSplit, {po.place_id},
                 [&]() {
                     std::vector<std::string> out{mr.snapshot.places[keeper].place_id};
                     out.insert(out.end(), new_ids.begin(), new_ids.end());
                     return out;
                 }()});
        }
        if (po.provisional && !mr.snapshot.places[keeper].provisional) {
            mr.events.push_back({PlaceEventKind::kProvisionalCleared,
                                 {po.place_id},
                                 {mr.snapshot.places[keeper].place_id}});
        }
    }
    // Births: any unassigned fresh non-connector.
    for (int fi = 0; fi < F; ++fi) {
        if (id_assigned[fi]) continue;
        if (is_connector(mr.snapshot.places[fi].kind)) continue;
        std::string nid = mint(mr.snapshot.places[fi].kind);
        mr.events.push_back({PlaceEventKind::kBirth, {}, {nid}});
        mr.snapshot.places[fi].place_id = nid;
        id_assigned[fi] = true;
    }

    // N:1 merge: if two prev places both proposed the same fresh
    // keeper, the keeper kept the highest-score winner's ID. The
    // others showed up here as "no successor" and should emit a merge
    // event onto the keeper. Detect by walking prev places that have
    // no entry in prev_successors but whose cells overlap a fresh
    // place that already has a non-self prev ID.
    for (int pi = 0; pi < P; ++pi) {
        if (is_connector(prev.places[pi].kind)) continue;
        if (prev_successors.count(pi)) continue;
        const auto& po = prev.places[pi];
        // Find best-overlap fresh non-connector.
        std::unordered_map<int, int> overlap;
        for (const auto& c : po.cell_region) {
            int fi = remap_cell(prev, mr.snapshot, c.x, c.y);
            if (fi < 0) continue;
            std::uint16_t fpid = mr.snapshot.place_assignment[fi];
            if (fpid == kNoPlace) continue;
            if (is_connector(mr.snapshot.places[fpid].kind)) continue;
            overlap[fpid]++;
        }
        if (overlap.empty()) {
            // True death candidate. Hold for stale_refresh_count
            // refreshes by re-inserting the prev place as stale.
            PlaceRegion s = po;
            s.stale = true;
            s.stale_refresh_count = po.stale_refresh_count + 1;
            if (s.stale_refresh_count >= cfg.stale_refresh_count) {
                mr.events.push_back({PlaceEventKind::kDeath, {po.place_id}, {}});
                continue;
            }
            mr.snapshot.places.push_back(s);
            // No place_assignment update: the stale region's cells
            // belong to whatever fresh place owns them now. We're
            // just keeping the metadata alive for ID return.
            continue;
        }
        int best_fi = overlap.begin()->first;
        for (const auto& kv : overlap) {
            if (kv.second > overlap[best_fi]) best_fi = kv.first;
        }
        // Merge into best_fi if its current ID is older than po. The
        // spec keeps the oldest prev ID.
        // The keeper has age_refreshes = prev_age + 1; compare with
        // po.age_refreshes to decide who is older. (Higher
        // age_refreshes = older.)
        auto& keeper = mr.snapshot.places[best_fi];
        if (po.age_refreshes > keeper.age_refreshes - 1) {
            std::string old_id_a = po.place_id;
            std::string old_id_b = keeper.place_id;
            keeper.place_id = po.place_id;
            keeper.age_refreshes = po.age_refreshes + 1;
            propagate_semantic_fields(po, keeper);
            mr.events.push_back({PlaceEventKind::kMerge,
                                 {old_id_a, old_id_b},
                                 {keeper.place_id}});
        } else {
            mr.events.push_back({PlaceEventKind::kMerge,
                                 {po.place_id, keeper.place_id},
                                 {keeper.place_id}});
        }
    }

    // Before connector matching: remap every fresh place's
    // adjacent_place_ids from cold-mint IDs (left over from
    // build_place_graph) to the final IDs assigned above. Without
    // this, a fresh connector whose endpoint room was renamed
    // (inheriting a prev ID, or born with a fresh ID from a split)
    // would still advertise the cold name, and the endpoint-set
    // comparison below would never match its prev counterpart.
    std::unordered_map<std::string, std::string> cold_to_final;
    cold_to_final.reserve(cold_ids.size());
    for (size_t i = 0; i < cold_ids.size() && i < mr.snapshot.places.size();
         ++i) {
        cold_to_final[cold_ids[i]] = mr.snapshot.places[i].place_id;
    }
    for (auto& p : mr.snapshot.places) {
        for (auto& adj : p.adjacent_place_ids) {
            auto it = cold_to_final.find(adj);
            if (it != cold_to_final.end()) adj = it->second;
        }
        // Keep deterministic ordering for endpoint-set comparisons
        // and downstream output.
        std::sort(p.adjacent_place_ids.begin(), p.adjacent_place_ids.end());
    }

    // Connector matching: score by endpoint-id set equality (now
    // computed against the remapped fresh adjacency), centroid
    // distance, and width delta.
    for (int pi = 0; pi < P; ++pi) {
        if (!is_connector(prev.places[pi].kind)) continue;
        const auto& po = prev.places[pi];
        std::unordered_set<std::string> prev_endpoints(
            po.adjacent_place_ids.begin(), po.adjacent_place_ids.end());

        int best_fi = -1;
        double best_score = -1;
        for (int fi = 0; fi < F; ++fi) {
            if (!is_connector(mr.snapshot.places[fi].kind)) continue;
            const auto& fp = mr.snapshot.places[fi];
            std::unordered_set<std::string> fset(fp.adjacent_place_ids.begin(),
                                                fp.adjacent_place_ids.end());
            if (fset != prev_endpoints) continue;
            double dx = fp.centroid.x - po.centroid.x;
            double dy = fp.centroid.y - po.centroid.y;
            double d = std::sqrt(dx * dx + dy * dy);
            if (d > cfg.connector_centroid_match_m) continue;
            if (po.width_m && fp.width_m &&
                po.kind != PlaceKind::kJunction &&
                fp.kind != PlaceKind::kJunction) {
                if (std::abs(*po.width_m - *fp.width_m) >
                    cfg.connector_width_match_m) continue;
            }
            double s = -d;
            if (fp.kind == po.kind) s += 1.0;
            if (s > best_score) { best_score = s; best_fi = fi; }
        }
        if (best_fi >= 0) {
            mr.snapshot.places[best_fi].place_id = po.place_id;
            mr.snapshot.places[best_fi].age_refreshes = po.age_refreshes + 1;
            propagate_semantic_fields(po, mr.snapshot.places[best_fi]);
            mr.events.push_back(
                {PlaceEventKind::kRefresh, {po.place_id}, {po.place_id}});
        }
    }
    // Birth IDs for unmatched fresh connectors. Detect by re-walking
    // and checking against prev ids.
    std::unordered_set<std::string> prev_ids;
    for (const auto& po : prev.places) prev_ids.insert(po.place_id);
    for (auto& fp : mr.snapshot.places) {
        if (!is_connector(fp.kind)) continue;
        if (prev_ids.count(fp.place_id)) continue;  // matched
        // It still has its cold-start ID like portal_0. Re-mint with
        // the post-prev counter so we don't collide.
        std::string nid = mint(fp.kind);
        mr.events.push_back({PlaceEventKind::kBirth, {}, {nid}});
        fp.place_id = nid;
    }

    return mr;
}

}  // namespace place_graph
