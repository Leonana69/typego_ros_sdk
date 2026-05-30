// place_graph_waypoints_node
//
// Generates LLM-selectable navigation waypoints from the geometry-only
// place_graph and exposes the place graph itself. Waypoints are
// "fixed anchors": once minted, an anchor keeps its ID *and* its frozen
// pose until that pose becomes invalid, so downstream object graphs keyed
// on (waypoint id, x, y) stay consistent across map refreshes. Semantic
// (CLIP/VLM) labeling is deferred; labels here are geometry-derived.
//
// See doc/waypoint_update.md for the design.

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <future>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <nlohmann/json.hpp>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "typego_interface/msg/place_event.hpp"
#include "typego_interface/msg/place_event_array.hpp"
#include "typego_interface/msg/place_graph.hpp"
#include "typego_interface/msg/place_region.hpp"
#include "typego_interface/msg/way_point.hpp"
#include "typego_interface/msg/way_point_array.hpp"
#include "typego_interface/srv/resolve_place_at_point.hpp"
#include "typego_interface/srv/set_place_label.hpp"

#include "place_graph/matching.hpp"
#include "place_graph/place_graph.hpp"

#include "typego_sdk/namespace_utils.hpp"
#include "typego_sdk/waypoint_anchor.hpp"

namespace tpg = place_graph;
using json = nlohmann::json;
using typego_sdk::AnchorRecord;
using typego_sdk::AnchorRegistry;
using typego_sdk::Candidate;
using typego_sdk::is_connector;
using typego_sdk::kFrontierIdBase;
using typego_sdk::pose_valid;
using typego_sdk::PublishedWaypoint;

namespace {

tpg::PlaceKind kind_from_string(const std::string& s) {
    if (s == "room") return tpg::PlaceKind::kRoom;
    if (s == "corridor") return tpg::PlaceKind::kCorridor;
    if (s == "open_area") return tpg::PlaceKind::kOpenArea;
    if (s == "portal") return tpg::PlaceKind::kPortal;
    if (s == "open_transition") return tpg::PlaceKind::kOpenTransition;
    if (s == "junction") return tpg::PlaceKind::kJunction;
    return tpg::PlaceKind::kFrontierRegion;
}

// World coordinates (meters) of a grid cell's center in snapshot `g`.
std::pair<double, double> cell_to_world(const tpg::PlaceGraphSnapshot& g,
                                        int cx, int cy) {
    return {g.origin_x + (cx + 0.5) * g.resolution_m,
            g.origin_y + (cy + 0.5) * g.resolution_m};
}

const char* event_kind_to_string(tpg::PlaceEventKind k) {
    switch (k) {
        case tpg::PlaceEventKind::kBirth: return "birth";
        case tpg::PlaceEventKind::kSplit: return "split";
        case tpg::PlaceEventKind::kMerge: return "merge";
        case tpg::PlaceEventKind::kDeath: return "death";
        case tpg::PlaceEventKind::kRefresh: return "refresh";
        case tpg::PlaceEventKind::kProvisionalCleared:
            return "provisional_cleared";
    }
    return "unknown";
}

// Trim surrounding whitespace and lowercase.
std::string normalize_token(const std::string& s) {
    std::size_t b = s.find_first_not_of(" \t\n\r");
    if (b == std::string::npos) return "";
    std::size_t e = s.find_last_not_of(" \t\n\r");
    std::string t = s.substr(b, e - b + 1);
    for (char& c : t)
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return t;
}

std::string clip_len(const std::string& s, std::size_t n) {
    return s.size() <= n ? s : s.substr(0, n);
}

// Controlled vocabularies (kept in sync with doc/semantic.md). semantic_label
// outside the room-type vocab is mapped to "unknown"; affordances outside the
// vocab are dropped.
const std::unordered_set<std::string>& room_type_vocab() {
    static const std::unordered_set<std::string> v = {
        "bedroom", "living_room", "kitchen", "dining_room", "bathroom",
        "toilet", "office", "study", "meeting_room", "hallway", "corridor",
        "foyer", "entryway", "closet", "storage", "pantry", "laundry_room",
        "garage", "lobby", "open_area", "unknown"};
    return v;
}
const std::unordered_set<std::string>& affordance_vocab() {
    static const std::unordered_set<std::string> v = {
        "sleep", "rest", "cook", "make_beverages", "eat", "wash", "toilet",
        "work", "meet", "transit", "wait", "store", "seating",
        "entertainment", "plant_care"};
    return v;
}

// Trim/lowercase/dedupe a token list; if `vocab` is non-null drop tokens not in
// it; cap to `max_n`.
std::vector<std::string> normalize_list(
    const std::vector<std::string>& in, std::size_t max_n,
    const std::unordered_set<std::string>* vocab) {
    std::vector<std::string> out;
    std::unordered_set<std::string> seen;
    for (const auto& raw : in) {
        std::string t = normalize_token(raw);
        if (t.empty()) continue;
        if (vocab && !vocab->count(t)) continue;
        if (!seen.insert(t).second) continue;
        out.push_back(std::move(t));
        if (out.size() >= max_n) break;
    }
    return out;
}

}  // namespace

class PlaceGraphWaypointsNode : public rclcpp::Node {
 public:
    PlaceGraphWaypointsNode()
        : rclcpp::Node("place_graph_waypoints",
                       typego_sdk::get_namespace_from_env()) {
        declare_params();

        rclcpp::QoS latched(1);
        latched.transient_local().reliable();

        map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, latched,
            std::bind(&PlaceGraphWaypointsNode::on_map, this,
                      std::placeholders::_1));
        waypoint_pub_ =
            create_publisher<typego_interface::msg::WayPointArray>(
                "waypoints", latched);
        graph_pub_ = create_publisher<typego_interface::msg::PlaceGraph>(
            "place_graph", latched);
        marker_pub_ =
            create_publisher<visualization_msgs::msg::MarkerArray>(
                "waypoint_markers", latched);
        // Events are transient happenings, not state: a normal (non-latched)
        // queue so late subscribers don't replay stale births.
        events_pub_ =
            create_publisher<typego_interface::msg::PlaceEventArray>(
                "typego/place_events", rclcpp::QoS(10));

        // Services run in a reentrant group so a SetPlaceLabel call that blocks
        // on the worker (to report real application) does not stall map intake
        // under the multi-threaded executor.
        service_cb_group_ =
            create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        set_label_srv_ = create_service<typego_interface::srv::SetPlaceLabel>(
            "typego/set_place_label",
            std::bind(&PlaceGraphWaypointsNode::on_set_label, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_cb_group_);
        resolve_srv_ =
            create_service<typego_interface::srv::ResolvePlaceAtPoint>(
                "typego/resolve_place_at_point",
                std::bind(&PlaceGraphWaypointsNode::on_resolve, this,
                          std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default, service_cb_group_);

        load_persisted();

        worker_ = std::thread(&PlaceGraphWaypointsNode::worker_loop, this);

        RCLCPP_INFO(get_logger(),
                    "place_graph_waypoints up: map=%s file=%s",
                    map_topic_.c_str(), waypoint_file_.c_str());
    }

    ~PlaceGraphWaypointsNode() override {
        stop_ = true;
        cv_.notify_all();
        if (worker_.joinable()) worker_.join();
    }

 private:
    // ----- parameters -----
    void declare_params() {
        map_topic_ = declare_parameter<std::string>("map_topic", "map");
        map_frame_ = declare_parameter<std::string>("map_frame", "map");
        slam_map_name_ =
            declare_parameter<std::string>("slam_map_name", "empty_map");
        std::string wf = declare_parameter<std::string>("waypoint_file", "");
        occupancy_threshold_ =
            declare_parameter<int>("occupancy_threshold", 65);
        min_refresh_interval_s_ =
            declare_parameter<double>("min_refresh_interval_s", 2.0);
        map_change_cell_threshold_ =
            declare_parameter<int>("map_change_cell_threshold", 500);
        entrance_offset_m_ =
            declare_parameter<double>("entrance_offset_m", 0.5);
        min_anchor_clearance_m_ =
            declare_parameter<double>("min_anchor_clearance_m", 0.25);
        waypoint_spacing_m_ =
            declare_parameter<double>("waypoint_spacing_m", 2.0);
        coverage_min_area_m2_ =
            declare_parameter<double>("coverage_min_area_m2", 12.0);
        stale_anchor_retire_refreshes_ =
            declare_parameter<int>("stale_anchor_retire_refreshes", 3);
        publish_stale_anchors_ =
            declare_parameter<bool>("publish_stale_anchors", false);

        registry_.min_anchor_clearance_m = min_anchor_clearance_m_;
        registry_.stale_anchor_retire_refreshes = stale_anchor_retire_refreshes_;
        registry_.publish_stale_anchors = publish_stale_anchors_;

        if (!wf.empty()) {
            waypoint_file_ = wf;
        } else {
            std::string share =
                ament_index_cpp::get_package_share_directory("typego_sdk");
            waypoint_file_ =
                share + "/resource/Map-" + slam_map_name_ + "/waypoints.json";
        }
        std::filesystem::path p(waypoint_file_);
        snapshot_file_ = (p.parent_path() / "place_graph_snapshot.json").string();
    }

    // ----- ROS callbacks -----
    void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        {
            std::lock_guard<std::mutex> lk(pending_mutex_);
            pending_map_ = msg;
        }
        cv_.notify_one();
    }

    tpg::MapSnapshot to_snapshot(const nav_msgs::msg::OccupancyGrid& g) const {
        tpg::MapSnapshot m;
        m.width = static_cast<int>(g.info.width);
        m.height = static_cast<int>(g.info.height);
        m.resolution_m = g.info.resolution;
        m.origin_x = g.info.origin.position.x;
        m.origin_y = g.info.origin.position.y;
        m.map_version = std::to_string(g.header.stamp.sec) + "." +
                        std::to_string(g.header.stamp.nanosec);
        m.cells.resize(static_cast<std::size_t>(m.width) * m.height);
        for (std::size_t i = 0; i < g.data.size() && i < m.cells.size(); ++i) {
            int8_t v = g.data[i];
            if (v < 0)
                m.cells[i] = tpg::CellState::kUnknown;
            else if (v >= occupancy_threshold_)
                m.cells[i] = tpg::CellState::kOccupied;
            else
                m.cells[i] = tpg::CellState::kFree;
        }
        return m;
    }

    std::size_t count_changed(const tpg::MapSnapshot& a) const {
        if (last_cells_.size() != a.cells.size()) return a.cells.size();
        std::size_t n = 0;
        for (std::size_t i = 0; i < a.cells.size(); ++i)
            if (a.cells[i] != last_cells_[i]) ++n;
        return n;
    }

    // ----- worker -----
    void worker_loop() {
        using clock = std::chrono::steady_clock;
        auto last_process = clock::time_point{};
        bool have_processed = false;
        while (!stop_) {
            nav_msgs::msg::OccupancyGrid::SharedPtr grid;
            std::vector<LabelRequest> labels;
            {
                std::unique_lock<std::mutex> lk(pending_mutex_);
                cv_.wait(lk, [&] {
                    return stop_ || pending_map_ != nullptr ||
                           !pending_labels_.empty();
                });
                if (stop_) break;
                grid = pending_map_;
                pending_map_ = nullptr;
                labels.swap(pending_labels_);
            }

            // Apply label writes promptly, independent of map processing, so a
            // label-only update does not wait for the next /map.
            if (!labels.empty()) apply_labels(labels);
            if (!grid) continue;  // label-only wake

            tpg::MapSnapshot snap = to_snapshot(*grid);
            std::size_t changed = count_changed(snap);
            if (have_processed && changed == 0) continue;

            // Debounce small changes by the min interval; a large change
            // (>= threshold) is processed promptly.
            if (have_processed &&
                changed < static_cast<std::size_t>(map_change_cell_threshold_)) {
                auto since = std::chrono::duration<double>(clock::now() -
                                                          last_process)
                                 .count();
                if (since < min_refresh_interval_s_) {
                    std::this_thread::sleep_for(std::chrono::duration<double>(
                        min_refresh_interval_s_ - since));
                    if (stop_) break;
                    // Prefer the freshest map that arrived during the wait.
                    std::lock_guard<std::mutex> lk(pending_mutex_);
                    if (pending_map_) {
                        grid = pending_map_;
                        pending_map_ = nullptr;
                        snap = to_snapshot(*grid);
                    }
                }
            }

            process(snap);
            last_cells_ = snap.cells;
            last_process = clock::now();
            have_processed = true;
        }
    }

    void process(const tpg::MapSnapshot& snap) {
        tpg::PlaceGraphSnapshot fresh = tpg::build_place_graph(snap, cfg_);
        // Always match — match_place_graphs accepts an empty prev (first graph
        // or fresh start), in which case every place is a birth. This both
        // mints stable IDs and gives us the PlaceEvent stream on run one.
        tpg::MatchResult mr = tpg::match_place_graphs(prev_, fresh, cfg_);
        tpg::PlaceGraphSnapshot matched = std::move(mr.snapshot);
        prev_ = matched;
        have_prev_ = true;

        std::vector<Candidate> candidates = generate_candidates(matched, snap);
        std::vector<PublishedWaypoint> published =
            registry_.reconcile(candidates, matched, snap);

        // Group member waypoint ids per place for the published graph.
        std::unordered_map<std::string, std::vector<std::uint32_t>> members;
        for (const auto& w : published) members[w.place_id].push_back(w.id);

        publish_waypoints(published);
        publish_graph(matched, members);
        publish_markers(published);
        persist(matched, published, members);
        publish_events(mr.events, matched.map_version);

        // Cache so a label-only update can re-publish/persist without a map,
        // and refresh the read-only snapshot used by ResolvePlaceAtPoint.
        last_published_ = std::move(published);
        last_members_ = std::move(members);
        refresh_pub_snapshot();
    }

    // ----- semantic labels + place resolution -----

    // Pending semantic-label writes, applied on the worker thread so the
    // worker-owned prev_ is never touched off-thread.
    struct LabelRequest {
        std::shared_ptr<typego_interface::srv::SetPlaceLabel::Request> req;
        std::promise<std::pair<bool, std::string>> done;
    };

    void on_set_label(
        const std::shared_ptr<typego_interface::srv::SetPlaceLabel::Request> req,
        std::shared_ptr<typego_interface::srv::SetPlaceLabel::Response> res) {
        std::promise<std::pair<bool, std::string>> pr;
        auto fut = pr.get_future();
        {
            std::lock_guard<std::mutex> lk(pending_mutex_);
            pending_labels_.push_back({req, std::move(pr)});
        }
        cv_.notify_one();
        // Block until the worker applies (or times out), so success/message
        // reflect the actual outcome rather than mere receipt.
        if (fut.wait_for(std::chrono::seconds(5)) ==
            std::future_status::ready) {
            auto [ok, msg] = fut.get();
            res->success = ok;
            res->message = msg;
        } else {
            res->success = false;
            res->message = "timed out applying label";
        }
    }

    void apply_labels(std::vector<LabelRequest>& labels) {
        bool any = false;
        for (auto& lr : labels) {
            const auto& req = *lr.req;
            tpg::PlaceRegion* place = nullptr;
            for (auto& p : prev_.places)
                if (p.place_id == req.place_id) { place = &p; break; }
            if (!place) {
                lr.done.set_value(
                    {false, "unknown place_id: " + req.place_id});
                continue;
            }
            if (place->label_locked && normalize_token(req.source) != "operator") {
                lr.done.set_value({false, "place label locked by operator"});
                continue;
            }
            // Monotonic confidence: a lower-confidence VLM pass must not clobber
            // a more-confident existing VLM label (operator writes bypass this).
            if (!req.lock && normalize_token(req.source) != "operator" &&
                (place->source == "vlm" || place->source == "vlm_lowconf") &&
                req.confidence < place->confidence) {
                lr.done.set_value({false, "existing label more confident"});
                continue;
            }
            apply_one_label(*place, req);
            any = true;
            lr.done.set_value({true, ""});
        }
        if (any) {
            // Re-publish/persist the updated graph using the cached waypoint
            // outputs (geometry is unchanged by a label write).
            publish_graph(prev_, last_members_);
            persist(prev_, last_published_, last_members_);
            refresh_pub_snapshot();
        }
    }

    void apply_one_label(
        tpg::PlaceRegion& p,
        const typego_interface::srv::SetPlaceLabel::Request& req) {
        std::string type = normalize_token(req.semantic_label);
        if (!type.empty() && !room_type_vocab().count(type)) type = "unknown";
        p.semantic_label = type;
        p.instance_label = clip_len(req.instance_label, 200);
        p.summary = clip_len(req.summary, 500);
        p.objects = normalize_list(req.objects, 32, nullptr);
        p.affordances = normalize_list(req.affordances, 32, &affordance_vocab());
        p.confidence = std::clamp(req.confidence, 0.0, 1.0);
        std::string src = normalize_token(req.source);
        if (src != "vlm" && src != "vlm_lowconf" && src != "operator")
            src = "vlm";
        if (req.lock) {
            p.operator_label = !p.instance_label.empty()
                                   ? p.instance_label
                                   : (!p.semantic_label.empty()
                                          ? p.semantic_label
                                          : p.operator_label);
            p.label_locked = true;
            p.source = "operator";
        } else {
            p.source = src;
        }
    }

    void on_resolve(
        const std::shared_ptr<
            typego_interface::srv::ResolvePlaceAtPoint::Request> req,
        std::shared_ptr<typego_interface::srv::ResolvePlaceAtPoint::Response>
            res) {
        std::shared_ptr<const tpg::PlaceGraphSnapshot> snap;
        {
            std::lock_guard<std::mutex> lk(snap_mutex_);
            snap = pub_snapshot_;
        }
        std::size_t n = std::min(req->xs.size(), req->ys.size());
        res->place_ids.assign(n, std::string());
        if (!snap) return;  // no graph yet -> all empty
        for (std::size_t i = 0; i < n; ++i) {
            const std::string* id =
                snap->place_id_at_world(req->xs[i], req->ys[i]);
            if (id) res->place_ids[i] = *id;
        }
    }

    void refresh_pub_snapshot() {
        auto copy = std::make_shared<tpg::PlaceGraphSnapshot>(prev_);
        std::lock_guard<std::mutex> lk(snap_mutex_);
        pub_snapshot_ = std::move(copy);
    }

    void publish_events(const std::vector<tpg::PlaceEvent>& events,
                        const std::string& map_version) {
        if (events.empty()) return;
        typego_interface::msg::PlaceEventArray arr;
        arr.header.frame_id = map_frame_;
        arr.header.stamp = now();
        arr.map_version = map_version;
        for (const auto& e : events) {
            typego_interface::msg::PlaceEvent m;
            m.kind = event_kind_to_string(e.kind);
            m.old_ids = e.old_ids;
            m.new_ids = e.new_ids;
            arr.events.push_back(std::move(m));
        }
        events_pub_->publish(arr);
    }

    // ----- candidate generation -----
    std::vector<Candidate> generate_candidates(
        const tpg::PlaceGraphSnapshot& g, const tpg::MapSnapshot& map) const {
        std::vector<Candidate> out;
        // place_id -> connector places that list it as an endpoint.
        std::unordered_map<std::string, std::vector<const tpg::PlaceRegion*>>
            connectors_of;
        for (const auto& p : g.places) {
            if (!is_connector(p.kind)) continue;
            for (const auto& adj : p.adjacent_place_ids)
                connectors_of[adj].push_back(&p);
        }

        for (const auto& p : g.places) {
            switch (p.kind) {
                case tpg::PlaceKind::kRoom:
                case tpg::PlaceKind::kOpenArea:
                case tpg::PlaceKind::kCorridor: {
                    // Entrances first: coverage dedups against them, and they
                    // decide whether a center is still needed.
                    std::vector<std::pair<double, double>> entrance_pts;
                    auto it = connectors_of.find(p.place_id);
                    if (it != connectors_of.end()) {
                        for (const auto* conn : it->second) {
                            auto e = entrance_candidate(p, *conn, g);
                            if (e) {
                                entrance_pts.emplace_back(e->x, e->y);
                                out.push_back(*e);
                            }
                        }
                    }

                    // interval coverage: corridors always; rooms/open areas
                    // above the area threshold.
                    std::vector<Candidate> coverage;
                    if (p.kind == tpg::PlaceKind::kCorridor ||
                        p.area_m2 > coverage_min_area_m2_) {
                        add_grid_coverage_candidates(g, map, p, entrance_pts,
                                                     coverage);
                    }

                    if (!coverage.empty()) {
                        // Big/long place: interval coverage replaces the lone
                        // center waypoint.
                        for (auto& c : coverage) out.push_back(std::move(c));
                    } else {
                        // Small place (or no coverage produced): a single
                        // center at the most-interior point.
                        Candidate c;
                        c.generation_key = p.place_id + "|center";
                        c.place_id = p.place_id;
                        c.place_kind = p.kind;
                        c.waypoint_role = "center";
                        c.x = p.peak.x;
                        c.y = p.peak.y;
                        c.yaw = 0.0;
                        c.semantic_context = "center of " + p.place_id;
                        out.push_back(c);
                    }
                    break;
                }
                case tpg::PlaceKind::kPortal:
                case tpg::PlaceKind::kOpenTransition:
                case tpg::PlaceKind::kJunction: {
                    Candidate c;
                    c.place_id = p.place_id;
                    c.place_kind = p.kind;
                    c.x = p.centroid.x;
                    c.y = p.centroid.y;
                    if (p.kind == tpg::PlaceKind::kPortal) {
                        c.waypoint_role = "doorway";
                    } else if (p.kind == tpg::PlaceKind::kOpenTransition) {
                        c.waypoint_role = "transition";
                    } else {
                        c.waypoint_role = "junction";
                    }
                    c.generation_key = p.place_id + "|" + c.waypoint_role;
                    c.yaw = p.approach_yaw.value_or(0.0);
                    c.semantic_context = connector_context(p);
                    out.push_back(c);
                    break;
                }
                case tpg::PlaceKind::kFrontierRegion: {
                    // Volatile: still proposed, handled specially.
                    Candidate c;
                    c.generation_key = p.place_id + "|frontier";
                    c.place_id = p.place_id;
                    c.place_kind = p.kind;
                    c.waypoint_role = "frontier";
                    c.x = p.peak.x;
                    c.y = p.peak.y;
                    c.yaw = 0.0;
                    c.semantic_context = "frontier in " + p.place_id;
                    out.push_back(c);
                    break;
                }
            }
        }
        return out;
    }

    std::optional<Candidate> entrance_candidate(
        const tpg::PlaceRegion& room, const tpg::PlaceRegion& conn,
        const tpg::PlaceGraphSnapshot& g) const {
        double dx = room.centroid.x - conn.centroid.x;
        double dy = room.centroid.y - conn.centroid.y;
        double n = std::hypot(dx, dy);
        if (n < 1e-6) return std::nullopt;
        dx /= n;
        dy /= n;
        Candidate c;
        c.generation_key = room.place_id + "|entrance|" + conn.place_id;
        c.place_id = room.place_id;
        c.place_kind = room.kind;
        c.waypoint_role = "entrance";
        c.x = conn.centroid.x + entrance_offset_m_ * dx;
        c.y = conn.centroid.y + entrance_offset_m_ * dy;
        c.yaw = std::atan2(dy, dx);
        c.semantic_context =
            "entrance to " + room.place_id + " from " + conn.place_id;
        // Validate it lands inside the room; if not, snap to the nearest
        // room cell along the inward direction.
        const tpg::PlaceRegion* at = g.place_at_world(c.x, c.y);
        if (!at || at->place_id != room.place_id) {
            if (!snap_into_place(g, room, c.x, c.y)) {
                // fall back to the room peak rather than dropping it
                c.x = room.peak.x;
                c.y = room.peak.y;
            }
        }
        return c;
    }

    // Find a cell of `room` nearest to (x,y); rewrite x,y to its center.
    bool snap_into_place(const tpg::PlaceGraphSnapshot& g,
                         const tpg::PlaceRegion& room, double& x,
                         double& y) const {
        double best = std::numeric_limits<double>::infinity();
        bool found = false;
        for (const auto& cell : room.cell_region) {
            auto [wx, wy] = cell_to_world(g, cell.x, cell.y);
            double d = std::hypot(wx - x, wy - y);
            if (d < best) {
                best = d;
                x = wx;
                y = wy;
                found = true;
            }
        }
        return found;
    }

    // Euclidean distance (m) from a world point to the nearest non-free cell
    // of the input map, capped at cap_m. Used to push coverage points off the
    // walls toward the open middle of a corridor/room.
    double free_radius(const tpg::MapSnapshot& map, double wx, double wy,
                       double cap_m) const {
        if (map.resolution_m <= 0.0) return 0.0;
        int cap = static_cast<int>(std::ceil(cap_m / map.resolution_m));
        int cx = static_cast<int>((wx - map.origin_x) / map.resolution_m);
        int cy = static_cast<int>((wy - map.origin_y) / map.resolution_m);
        double best = cap_m;
        for (int dy = -cap; dy <= cap; ++dy) {
            for (int dx = -cap; dx <= cap; ++dx) {
                int nx = cx + dx, ny = cy + dy;
                bool blocked = !map.in_bounds(nx, ny) ||
                               map.at(nx, ny) != tpg::CellState::kFree;
                if (blocked) {
                    double d = std::hypot(static_cast<double>(dx),
                                          static_cast<double>(dy)) *
                               map.resolution_m;
                    if (d < best) best = d;
                }
            }
        }
        return best;
    }

    // Within max_disp of (x,y), move to the in-place point with the greatest
    // wall clearance. Centers a coverage point in the corridor/room width
    // (keeping it off the walls), and is a no-op deep in open space where
    // clearance is already locally maximal. Narrow corridors simply yield a
    // smaller clearance maximum on the centerline.
    void snap_to_clearest(const tpg::PlaceGraphSnapshot& g,
                          const tpg::MapSnapshot& map,
                          const std::string& place_id, double& x, double& y,
                          double max_disp) const {
        const double cap = std::min(max_disp + 0.3, 1.0);
        const double step = std::max(map.resolution_m, max_disp / 4.0);
        double bx = x, by = y;
        double bc = free_radius(map, x, y, cap);
        for (double ox = -max_disp; ox <= max_disp + 1e-9; ox += step) {
            for (double oy = -max_disp; oy <= max_disp + 1e-9; oy += step) {
                if (std::hypot(ox, oy) > max_disp) continue;
                double nx = x + ox, ny = y + oy;
                const tpg::PlaceRegion* at = g.place_at_world(nx, ny);
                if (!at || at->place_id != place_id) continue;
                double c = free_radius(map, nx, ny, cap);
                if (c > bc) {
                    bc = c;
                    bx = nx;
                    by = ny;
                }
            }
        }
        x = bx;
        y = by;
    }

    // Sample interval waypoints on an absolute world lattice of pitch
    // waypoint_spacing_m. Anchoring the lattice to world (0,0) keeps each
    // point's generation_key tied to its absolute position, so it is stable as
    // the region grows (no ordinal reindexing). Each accepted point is centered
    // in the local free width; points landing within ~half-spacing of an
    // entrance or an already-placed coverage point are suppressed so the
    // entrance itself serves as the coverage there.
    void add_grid_coverage_candidates(
        const tpg::PlaceGraphSnapshot& g, const tpg::MapSnapshot& map,
        const tpg::PlaceRegion& p,
        const std::vector<std::pair<double, double>>& entrance_pts,
        std::vector<Candidate>& out) const {
        if (p.cell_region.empty()) return;
        const double s = waypoint_spacing_m_;
        if (s <= 0.0) return;
        const double max_disp = 0.5 * s;
        const double dedup_r = 0.5 * s;

        double minx = 1e18, miny = 1e18, maxx = -1e18, maxy = -1e18;
        for (const auto& c : p.cell_region) {
            auto [wx, wy] = cell_to_world(g, c.x, c.y);
            minx = std::min(minx, wx);
            maxx = std::max(maxx, wx);
            miny = std::min(miny, wy);
            maxy = std::max(maxy, wy);
        }
        int gx0 = static_cast<int>(std::floor(minx / s));
        int gx1 = static_cast<int>(std::ceil(maxx / s));
        int gy0 = static_cast<int>(std::floor(miny / s));
        int gy1 = static_cast<int>(std::ceil(maxy / s));

        // Entrances are committed coverage too; coverage dedups against them
        // and against coverage points already placed in this pass.
        std::vector<std::pair<double, double>> placed = entrance_pts;

        for (int gx = gx0; gx <= gx1; ++gx) {
            for (int gy = gy0; gy <= gy1; ++gy) {
                double px = gx * s, py = gy * s;
                double ax = px, ay = py;
                bool ok = pose_valid(px, py, p.place_id, false, g, map,
                                     min_anchor_clearance_m_);
                if (!ok) {
                    double tx = p.centroid.x - px, ty = p.centroid.y - py;
                    double tn = std::hypot(tx, ty);
                    if (tn > 1e-6) {
                        tx /= tn;
                        ty /= tn;
                        for (double off : {0.25 * s, 0.5 * s}) {
                            double nx = px + off * tx, ny = py + off * ty;
                            if (pose_valid(nx, ny, p.place_id, false, g, map,
                                           min_anchor_clearance_m_)) {
                                ax = nx;
                                ay = ny;
                                ok = true;
                                break;
                            }
                        }
                    }
                }
                if (!ok) continue;

                // Center it in the open width before deduping on final pose.
                snap_to_clearest(g, map, p.place_id, ax, ay, max_disp);

                bool clustered = false;
                for (const auto& q : placed) {
                    if (std::hypot(ax - q.first, ay - q.second) < dedup_r) {
                        clustered = true;
                        break;
                    }
                }
                if (clustered) continue;

                Candidate c;
                c.generation_key = p.place_id + "|grid|" +
                                   std::to_string(gx) + "_" + std::to_string(gy);
                c.place_id = p.place_id;
                c.place_kind = p.kind;
                c.waypoint_role = "coverage";
                c.x = ax;
                c.y = ay;
                c.yaw = 0.0;
                c.semantic_context = "coverage in " + p.place_id;
                placed.emplace_back(ax, ay);
                out.push_back(c);
            }
        }
    }

    std::string connector_context(const tpg::PlaceRegion& p) const {
        std::string kind = tpg::to_string(p.kind);
        std::string s = kind + ":";
        for (std::size_t i = 0; i < p.adjacent_place_ids.size(); ++i) {
            s += (i == 0 ? " " : " <-> ") + p.adjacent_place_ids[i];
        }
        return s;
    }

    // ----- publishing -----
    void publish_waypoints(const std::vector<PublishedWaypoint>& wps) {
        typego_interface::msg::WayPointArray arr;
        for (const auto& w : wps) {
            typego_interface::msg::WayPoint m;
            m.id = w.id;
            m.x = w.x;
            m.y = w.y;
            m.yaw = w.yaw;
            m.label = w.label;
            m.semantic_context = w.semantic_context;
            m.confidence = w.confidence;
            m.source = w.source;
            m.clearance_m = w.clearance_m;
            m.generation_reason = w.generation_reason;
            m.place_id = w.place_id;
            m.waypoint_role = w.waypoint_role;
            arr.waypoints.push_back(m);
        }
        waypoint_pub_->publish(arr);
    }

    void publish_graph(
        const tpg::PlaceGraphSnapshot& g,
        const std::unordered_map<std::string, std::vector<std::uint32_t>>&
            members) {
        typego_interface::msg::PlaceGraph msg;
        msg.header.frame_id = map_frame_;
        msg.header.stamp = now();
        msg.map_version = g.map_version;
        for (const auto& p : g.places) {
            typego_interface::msg::PlaceRegion r;
            r.place_id = p.place_id;
            r.kind = tpg::to_string(p.kind);
            r.semantic_label = p.semantic_label;
            r.instance_label = p.instance_label;
            r.operator_label = p.operator_label;
            r.label_locked = p.label_locked;
            r.confidence = p.confidence;
            r.source = p.source;
            r.objects = p.objects;
            r.affordances = p.affordances;
            r.summary = p.summary;
            r.centroid_x = p.centroid.x;
            r.centroid_y = p.centroid.y;
            r.peak_x = p.peak.x;
            r.peak_y = p.peak.y;
            r.area_m2 = p.area_m2;
            r.clearance_m = p.clearance_m;
            r.frontier_ratio = p.frontier_ratio;
            r.provisional = p.provisional;
            r.stale = p.stale;
            r.has_approach_yaw = p.approach_yaw.has_value();
            r.approach_yaw = p.approach_yaw.value_or(0.0);
            r.has_width_m = p.width_m.has_value();
            r.width_m = p.width_m.value_or(0.0);
            r.adjacent_place_ids = p.adjacent_place_ids;
            auto it = members.find(p.place_id);
            if (it != members.end()) r.member_waypoint_ids = it->second;
            msg.places.push_back(r);
        }
        graph_pub_->publish(msg);
    }

    void publish_markers(const std::vector<PublishedWaypoint>& wps) {
        visualization_msgs::msg::MarkerArray arr;
        visualization_msgs::msg::Marker clear;
        clear.header.frame_id = map_frame_;
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(clear);
        for (const auto& w : wps) {
            visualization_msgs::msg::Marker s;
            s.header.frame_id = map_frame_;
            s.header.stamp = now();
            s.ns = "waypoints";
            s.id = static_cast<int>(w.id);
            s.type = visualization_msgs::msg::Marker::SPHERE;
            s.action = visualization_msgs::msg::Marker::ADD;
            s.pose.position.x = w.x;
            s.pose.position.y = w.y;
            s.pose.orientation.w = 1.0;
            s.scale.x = s.scale.y = s.scale.z = 0.2;
            s.color.g = 1.0f;
            s.color.a = 1.0f;
            arr.markers.push_back(s);

            visualization_msgs::msg::Marker t;
            t.header.frame_id = map_frame_;
            t.header.stamp = now();
            t.ns = "waypoint_labels";
            t.id = static_cast<int>(w.id);
            t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            t.action = visualization_msgs::msg::Marker::ADD;
            t.pose.position.x = w.x;
            t.pose.position.y = w.y;
            t.pose.position.z = 0.4;
            t.scale.z = 0.22;
            t.color.r = t.color.g = t.color.b = 1.0f;
            t.color.a = 1.0f;
            t.text = std::to_string(w.id) + " " + w.waypoint_role + " (" +
                     w.label + ")";
            arr.markers.push_back(t);
        }
        marker_pub_->publish(arr);
    }

    // ----- persistence -----
    bool atomic_write(const std::string& file, const std::string& content) {
        std::filesystem::path fp(file);
        std::error_code ec;
        if (!fp.parent_path().empty())
            std::filesystem::create_directories(fp.parent_path(), ec);
        std::string tmp = file + ".tmp";
        {
            std::ofstream f(tmp, std::ios::trunc);
            if (!f.is_open()) return false;
            f << content;
        }
        std::filesystem::rename(tmp, file, ec);
        if (ec) {
            std::filesystem::remove(file, ec);
            ec.clear();
            std::filesystem::rename(tmp, file, ec);
        }
        return !ec;
    }

    void persist(
        const tpg::PlaceGraphSnapshot& g,
        const std::vector<PublishedWaypoint>& wps,
        const std::unordered_map<std::string, std::vector<std::uint32_t>>&
            members) {
        write_waypoints_json(g, wps, members);
        write_snapshot_json(g);
    }

    void write_waypoints_json(
        const tpg::PlaceGraphSnapshot& g,
        const std::vector<PublishedWaypoint>& wps,
        const std::unordered_map<std::string, std::vector<std::uint32_t>>&
            members) {
        json doc;
        doc["version"] = 3;
        doc["map_version"] = g.map_version;
        doc["next_waypoint_id"] = registry_.next_id;

        doc["waypoints"] = json::array();
        for (const auto& w : wps) {
            if (w.id >= kFrontierIdBase) continue;  // volatile, not persisted
            doc["waypoints"].push_back({
                {"id", w.id},
                {"x", w.x},
                {"y", w.y},
                {"yaw", w.yaw},
                {"label", w.label},
                {"semantic_context", w.semantic_context},
                {"confidence", w.confidence},
                {"source", w.source},
                {"clearance_m", w.clearance_m},
                {"generation_reason", w.generation_reason},
                {"place_id", w.place_id},
                {"waypoint_role", w.waypoint_role},
            });
        }

        doc["places"] = json::array();
        for (const auto& p : g.places) {
            json jp = {
                {"place_id", p.place_id},
                {"kind", tpg::to_string(p.kind)},
                {"semantic_label", p.semantic_label},
                {"instance_label", p.instance_label},
                {"operator_label", p.operator_label},
                {"label_locked", p.label_locked},
                {"confidence", p.confidence},
                {"source", p.source},
                {"objects", p.objects},
                {"affordances", p.affordances},
                {"summary", p.summary},
                {"centroid", {{"x", p.centroid.x}, {"y", p.centroid.y}}},
                {"peak", {{"x", p.peak.x}, {"y", p.peak.y}}},
                {"area_m2", p.area_m2},
                {"clearance_m", p.clearance_m},
                {"frontier_ratio", p.frontier_ratio},
                {"provisional", p.provisional},
                {"stale", p.stale},
                {"adjacent_place_ids", p.adjacent_place_ids},
            };
            auto it = members.find(p.place_id);
            jp["member_waypoint_ids"] =
                it != members.end() ? it->second : std::vector<std::uint32_t>{};
            doc["places"].push_back(jp);
        }

        doc["anchor_registry"] = json::array();
        for (const auto& kv : registry_.records) {
            const AnchorRecord& r = kv.second;
            doc["anchor_registry"].push_back({
                {"id", r.id},
                {"generation_key", r.generation_key},
                {"place_id", r.place_id},
                {"waypoint_role", r.waypoint_role},
                {"x", r.x},
                {"y", r.y},
                {"yaw", r.yaw},
                {"active", r.active},
                {"stale", r.stale},
                {"stale_refresh_count", r.stale_refresh_count},
            });
        }
        atomic_write(waypoint_file_, doc.dump(2) + "\n");
    }

    void write_snapshot_json(const tpg::PlaceGraphSnapshot& g) {
        json doc;
        doc["width"] = g.width;
        doc["height"] = g.height;
        doc["resolution_m"] = g.resolution_m;
        doc["origin_x"] = g.origin_x;
        doc["origin_y"] = g.origin_y;
        doc["map_version"] = g.map_version;
        json counters = json::object();
        for (const auto& kv : g.id_counters)
            counters[tpg::to_string(kv.first)] = kv.second;
        doc["id_counters"] = counters;

        doc["places"] = json::array();
        for (const auto& p : g.places) {
            doc["places"].push_back({
                {"place_id", p.place_id},
                {"kind", tpg::to_string(p.kind)},
                {"semantic_label", p.semantic_label},
                {"instance_label", p.instance_label},
                {"operator_label", p.operator_label},
                {"label_locked", p.label_locked},
                {"confidence", p.confidence},
                {"source", p.source},
                {"objects", p.objects},
                {"affordances", p.affordances},
                {"summary", p.summary},
                {"centroid_x", p.centroid.x},
                {"centroid_y", p.centroid.y},
                {"peak_x", p.peak.x},
                {"peak_y", p.peak.y},
                {"area_m2", p.area_m2},
                {"clearance_m", p.clearance_m},
                {"frontier_ratio", p.frontier_ratio},
                {"provisional", p.provisional},
                {"stale", p.stale},
                {"age_refreshes", p.age_refreshes},
                {"has_approach_yaw", p.approach_yaw.has_value()},
                {"approach_yaw", p.approach_yaw.value_or(0.0)},
                {"has_width_m", p.width_m.has_value()},
                {"width_m", p.width_m.value_or(0.0)},
                {"adjacent_place_ids", p.adjacent_place_ids},
            });
        }

        // RLE-encode the uint16 place_assignment grid.
        json rle = json::array();
        const auto& pa = g.place_assignment;
        for (std::size_t i = 0; i < pa.size();) {
            std::uint16_t v = pa[i];
            std::size_t j = i + 1;
            while (j < pa.size() && pa[j] == v) ++j;
            rle.push_back({v, static_cast<std::uint64_t>(j - i)});
            i = j;
        }
        doc["place_assignment_rle"] = rle;
        atomic_write(snapshot_file_, doc.dump() + "\n");
    }

    void load_persisted() {
        load_waypoints_json();
        load_snapshot_json();
    }

    void load_waypoints_json() {
        std::ifstream f(waypoint_file_);
        if (!f.is_open()) return;
        try {
            json doc = json::parse(f);
            registry_.next_id = doc.value("next_waypoint_id", 0u);
            if (doc.contains("anchor_registry")) {
                for (const auto& a : doc["anchor_registry"]) {
                    AnchorRecord r;
                    r.id = a.value("id", 0u);
                    r.generation_key = a.value("generation_key", "");
                    r.place_id = a.value("place_id", "");
                    r.waypoint_role = a.value("waypoint_role", "");
                    r.x = a.value("x", 0.0);
                    r.y = a.value("y", 0.0);
                    r.yaw = a.value("yaw", 0.0);
                    r.active = a.value("active", false);
                    r.stale = a.value("stale", false);
                    r.stale_refresh_count = a.value("stale_refresh_count", 0u);
                    if (!r.generation_key.empty())
                        registry_.records[r.generation_key] = r;
                }
            }
            RCLCPP_INFO(get_logger(),
                        "loaded %zu anchors, next_id=%u",
                        registry_.records.size(), registry_.next_id);
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "failed to load %s: %s",
                        waypoint_file_.c_str(), e.what());
        }
    }

    void load_snapshot_json() {
        std::ifstream f(snapshot_file_);
        if (!f.is_open()) return;
        try {
            json doc = json::parse(f);
            tpg::PlaceGraphSnapshot g;
            g.width = doc.value("width", 0);
            g.height = doc.value("height", 0);
            g.resolution_m = doc.value("resolution_m", 0.0);
            g.origin_x = doc.value("origin_x", 0.0);
            g.origin_y = doc.value("origin_y", 0.0);
            g.map_version = doc.value("map_version", "");
            if (doc.contains("id_counters")) {
                for (auto it = doc["id_counters"].begin();
                     it != doc["id_counters"].end(); ++it) {
                    g.id_counters[kind_from_string(it.key())] =
                        it.value().get<int>();
                }
            }
            for (const auto& jp : doc["places"]) {
                tpg::PlaceRegion p;
                p.place_id = jp.value("place_id", "");
                p.kind = kind_from_string(jp.value("kind", "room"));
                p.semantic_label = jp.value("semantic_label", "");
                p.instance_label = jp.value("instance_label", "");
                p.operator_label = jp.value("operator_label", "");
                p.label_locked = jp.value("label_locked", false);
                p.confidence = jp.value("confidence", 0.0);
                p.source = jp.value("source", "map");
                p.objects = jp.value("objects", std::vector<std::string>{});
                p.affordances =
                    jp.value("affordances", std::vector<std::string>{});
                p.summary = jp.value("summary", "");
                p.centroid.x = jp.value("centroid_x", 0.0);
                p.centroid.y = jp.value("centroid_y", 0.0);
                p.peak.x = jp.value("peak_x", 0.0);
                p.peak.y = jp.value("peak_y", 0.0);
                p.area_m2 = jp.value("area_m2", 0.0);
                p.clearance_m = jp.value("clearance_m", 0.0);
                p.frontier_ratio = jp.value("frontier_ratio", 0.0);
                p.provisional = jp.value("provisional", false);
                p.stale = jp.value("stale", false);
                p.age_refreshes = jp.value("age_refreshes", 0);
                if (jp.value("has_approach_yaw", false))
                    p.approach_yaw = jp.value("approach_yaw", 0.0);
                if (jp.value("has_width_m", false))
                    p.width_m = jp.value("width_m", 0.0);
                p.adjacent_place_ids =
                    jp.value("adjacent_place_ids", std::vector<std::string>{});
                g.places.push_back(p);
            }
            // Decode the RLE grid and rebuild cell_region per place.
            g.place_assignment.assign(
                static_cast<std::size_t>(g.width) * g.height, tpg::kNoPlace);
            std::size_t idx = 0;
            for (const auto& run : doc["place_assignment_rle"]) {
                std::uint16_t v = run[0].get<std::uint16_t>();
                std::uint64_t n = run[1].get<std::uint64_t>();
                for (std::uint64_t k = 0; k < n && idx < g.place_assignment.size();
                     ++k, ++idx)
                    g.place_assignment[idx] = v;
            }
            for (int y = 0; y < g.height; ++y) {
                for (int x = 0; x < g.width; ++x) {
                    std::uint16_t v = g.place_assignment[y * g.width + x];
                    if (v == tpg::kNoPlace) continue;
                    if (v < g.places.size())
                        g.places[v].cell_region.push_back({x, y});
                }
            }
            prev_ = std::move(g);
            have_prev_ = !prev_.places.empty();
            RCLCPP_INFO(get_logger(),
                        "restored snapshot: %zu places for restart matching",
                        prev_.places.size());
        } catch (const std::exception& e) {
            RCLCPP_WARN(get_logger(), "failed to load snapshot %s: %s",
                        snapshot_file_.c_str(), e.what());
        }
    }

    // ----- members -----
    std::string map_topic_, map_frame_, slam_map_name_;
    std::string waypoint_file_, snapshot_file_;
    int occupancy_threshold_ = 65;
    double min_refresh_interval_s_ = 2.0;
    int map_change_cell_threshold_ = 500;
    double entrance_offset_m_ = 0.5;
    double min_anchor_clearance_m_ = 0.25;
    double waypoint_spacing_m_ = 2.0;
    double coverage_min_area_m2_ = 12.0;
    int stale_anchor_retire_refreshes_ = 3;
    bool publish_stale_anchors_ = false;
    tpg::Config cfg_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Publisher<typego_interface::msg::WayPointArray>::SharedPtr
        waypoint_pub_;
    rclcpp::Publisher<typego_interface::msg::PlaceGraph>::SharedPtr graph_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        marker_pub_;
    rclcpp::Publisher<typego_interface::msg::PlaceEventArray>::SharedPtr
        events_pub_;
    rclcpp::CallbackGroup::SharedPtr service_cb_group_;
    rclcpp::Service<typego_interface::srv::SetPlaceLabel>::SharedPtr
        set_label_srv_;
    rclcpp::Service<typego_interface::srv::ResolvePlaceAtPoint>::SharedPtr
        resolve_srv_;

    std::thread worker_;
    std::atomic<bool> stop_{false};
    std::condition_variable cv_;
    std::mutex pending_mutex_;
    nav_msgs::msg::OccupancyGrid::SharedPtr pending_map_;
    std::vector<LabelRequest> pending_labels_;  // guarded by pending_mutex_
    std::vector<tpg::CellState> last_cells_;

    // Worker-thread-owned (no lock needed; only the worker touches these).
    tpg::PlaceGraphSnapshot prev_;
    bool have_prev_ = false;
    AnchorRegistry registry_;
    // Cached last outputs so a label-only write can re-publish/persist.
    std::vector<PublishedWaypoint> last_published_;
    std::unordered_map<std::string, std::vector<std::uint32_t>> last_members_;

    // Immutable snapshot copy for ResolvePlaceAtPoint reads off the worker
    // thread (guarded by snap_mutex_).
    std::mutex snap_mutex_;
    std::shared_ptr<const tpg::PlaceGraphSnapshot> pub_snapshot_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    // Multi-threaded so a SetPlaceLabel call that blocks on the worker (to
    // return a real apply result) does not stall the map subscription.
    rclcpp::executors::MultiThreadedExecutor exec;
    auto node = std::make_shared<PlaceGraphWaypointsNode>();
    exec.add_node(node);
    exec.spin();
    rclcpp::shutdown();
    return 0;
}
