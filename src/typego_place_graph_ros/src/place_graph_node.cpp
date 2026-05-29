// Live test harness for typego_place_graph.
//
// Subscribes to a nav_msgs/OccupancyGrid (default topic "/map"),
// converts it to a typego_place_graph::MapSnapshot, runs the
// segmentation pipeline (build_place_graph + optional stable-ID
// matching), and republishes the result as RViz markers so the room /
// corridor / connector segmentation can be eyeballed against a live
// SLAM session.
//
// This node deliberately lives in its own package: the core
// typego_place_graph library stays free of ROS dependencies.

#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "typego_place_graph/matching.hpp"
#include "typego_place_graph/place_graph.hpp"

namespace tpg = typego_place_graph;

namespace {

std_msgs::msg::ColorRGBA make_color(double r, double g, double b, double a) {
  std_msgs::msg::ColorRGBA c;
  c.r = static_cast<float>(r);
  c.g = static_cast<float>(g);
  c.b = static_cast<float>(b);
  c.a = static_cast<float>(a);
  return c;
}

// HSV (h in [0,1)) -> RGB, full helper so each place gets a distinct,
// reasonably bright color.
std_msgs::msg::ColorRGBA hsv(double h, double s, double v, double a) {
  double r = v, g = v, b = v;
  double hh = h * 6.0;
  int i = static_cast<int>(hh) % 6;
  double f = hh - std::floor(hh);
  double p = v * (1.0 - s);
  double q = v * (1.0 - s * f);
  double t = v * (1.0 - s * (1.0 - f));
  switch (i) {
    case 0: r = v; g = t; b = p; break;
    case 1: r = q; g = v; b = p; break;
    case 2: r = p; g = v; b = t; break;
    case 3: r = p; g = q; b = v; break;
    case 4: r = t; g = p; b = v; break;
    default: r = v; g = p; b = q; break;
  }
  return make_color(r, g, b, a);
}

// Deterministic color per place_id so a region keeps its color across
// refreshes (matching keeps the id stable).
std_msgs::msg::ColorRGBA color_for_id(const std::string& id, double alpha) {
  std::size_t h = std::hash<std::string>{}(id);
  double hue = (h % 1000) / 1000.0;
  return hsv(hue, 0.65, 0.95, alpha);
}

bool is_connector(tpg::PlaceKind k) {
  return k == tpg::PlaceKind::kPortal ||
         k == tpg::PlaceKind::kOpenTransition ||
         k == tpg::PlaceKind::kJunction;
}

std_msgs::msg::ColorRGBA connector_color(tpg::PlaceKind k, double alpha) {
  switch (k) {
    case tpg::PlaceKind::kPortal: return make_color(1.0, 0.15, 0.15, alpha);
    case tpg::PlaceKind::kOpenTransition: return make_color(1.0, 0.6, 0.1, alpha);
    case tpg::PlaceKind::kJunction: return make_color(0.85, 0.2, 1.0, alpha);
    default: return make_color(0.7, 0.7, 0.7, alpha);
  }
}

}  // namespace

class PlaceGraphNode : public rclcpp::Node {
 public:
  PlaceGraphNode() : rclcpp::Node("place_graph_node") {
    map_topic_ = declare_parameter<std::string>("map_topic", "/map");
    occupancy_threshold_ = declare_parameter<int>("occupancy_threshold", 65);
    enable_matching_ = declare_parameter<bool>("enable_matching", true);

    // Pipeline tuning, surfaced as ROS params so they can be changed at
    // launch without recompiling. Defaults mirror tpg::Config.
    tpg::Config def;
    cfg_.seg_resolution_m =
        declare_parameter<double>("seg_resolution_m", def.seg_resolution_m);
    cfg_.unknown_close_m =
        declare_parameter<double>("unknown_close_m", def.unknown_close_m);
    cfg_.r_open_m = declare_parameter<double>("r_open_m", def.r_open_m);
    cfg_.a_min_m2 = declare_parameter<double>("a_min_m2", def.a_min_m2);
    cfg_.a_open_m2 = declare_parameter<double>("a_open_m2", def.a_open_m2);
    cfg_.corridor_aspect =
        declare_parameter<double>("corridor_aspect", def.corridor_aspect);
    cfg_.w_max_portal_m =
        declare_parameter<double>("w_max_portal_m", def.w_max_portal_m);
    cfg_.frontier_region_ratio = declare_parameter<double>(
        "frontier_region_ratio", def.frontier_region_ratio);

    rclcpp::QoS map_qos(1);
    map_qos.transient_local().reliable();
    sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic_, map_qos,
        std::bind(&PlaceGraphNode::on_map, this, std::placeholders::_1));

    rclcpp::QoS marker_qos(1);
    marker_qos.transient_local().reliable();
    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "~/markers", marker_qos);

    RCLCPP_INFO(get_logger(),
                "place_graph_node up: map_topic=%s seg_res=%.2fm "
                "r_open=%.2fm matching=%s",
                map_topic_.c_str(), cfg_.seg_resolution_m, cfg_.r_open_m,
                enable_matching_ ? "on" : "off");
  }

 private:
  tpg::MapSnapshot to_snapshot(const nav_msgs::msg::OccupancyGrid& msg) const {
    tpg::MapSnapshot m;
    m.width = static_cast<int>(msg.info.width);
    m.height = static_cast<int>(msg.info.height);
    m.resolution_m = msg.info.resolution;
    m.origin_x = msg.info.origin.position.x;
    m.origin_y = msg.info.origin.position.y;
    m.map_version = std::to_string(msg.header.stamp.sec) + "." +
                    std::to_string(msg.header.stamp.nanosec);
    m.cells.resize(static_cast<std::size_t>(m.width) * m.height);
    for (std::size_t i = 0; i < msg.data.size() && i < m.cells.size(); ++i) {
      int8_t v = msg.data[i];
      if (v < 0) {
        m.cells[i] = tpg::CellState::kUnknown;
      } else if (v >= occupancy_threshold_) {
        m.cells[i] = tpg::CellState::kOccupied;
      } else {
        m.cells[i] = tpg::CellState::kFree;
      }
    }
    return m;
  }

  void on_map(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Origin yaw is assumed ~0 (standard for map_server / slam_toolbox).
    // The flat x/y cell->world mapping ignores rotation; warn once if a
    // rotated map shows up so the misalignment isn't silently puzzling.
    double qz = msg->info.origin.orientation.z;
    double qw = msg->info.origin.orientation.w;
    double yaw = std::atan2(2.0 * qz * qw, 1.0 - 2.0 * qz * qz);
    if (std::abs(yaw) > 0.02 && !warned_rotation_) {
      RCLCPP_WARN(get_logger(),
                  "map origin yaw=%.3f rad is non-zero; markers assume an "
                  "axis-aligned grid and will be misaligned.",
                  yaw);
      warned_rotation_ = true;
    }

    frame_id_ = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    tpg::MapSnapshot snap = to_snapshot(*msg);
    tpg::PlaceGraphSnapshot fresh = tpg::build_place_graph(snap, cfg_);

    tpg::PlaceGraphSnapshot result;
    if (enable_matching_ && have_prev_) {
      tpg::MatchResult mr = tpg::match_place_graphs(prev_, fresh, cfg_);
      result = mr.snapshot;
      log_events(mr.events);
    } else {
      result = fresh;
    }
    prev_ = result;
    have_prev_ = true;

    publish_markers(result);
    log_summary(result);
  }

  void log_events(const std::vector<tpg::PlaceEvent>& events) const {
    for (const auto& e : events) {
      const char* k = "?";
      switch (e.kind) {
        case tpg::PlaceEventKind::kBirth: k = "birth"; break;
        case tpg::PlaceEventKind::kSplit: k = "split"; break;
        case tpg::PlaceEventKind::kMerge: k = "merge"; break;
        case tpg::PlaceEventKind::kDeath: k = "death"; break;
        case tpg::PlaceEventKind::kRefresh: k = "refresh"; break;
        case tpg::PlaceEventKind::kProvisionalCleared:
          k = "provisional_cleared"; break;
      }
      if (e.kind == tpg::PlaceEventKind::kRefresh) continue;  // too chatty
      RCLCPP_INFO(get_logger(), "event %s: %zu old -> %zu new", k,
                  e.old_ids.size(), e.new_ids.size());
    }
  }

  void log_summary(const tpg::PlaceGraphSnapshot& g) const {
    std::unordered_map<int, int> by_kind;
    for (const auto& p : g.places)
      by_kind[static_cast<int>(p.kind)]++;
    RCLCPP_INFO(
        get_logger(),
        "places=%zu | room=%d corridor=%d open=%d portal=%d "
        "open_trans=%d junction=%d frontier=%d",
        g.places.size(), by_kind[(int)tpg::PlaceKind::kRoom],
        by_kind[(int)tpg::PlaceKind::kCorridor],
        by_kind[(int)tpg::PlaceKind::kOpenArea],
        by_kind[(int)tpg::PlaceKind::kPortal],
        by_kind[(int)tpg::PlaceKind::kOpenTransition],
        by_kind[(int)tpg::PlaceKind::kJunction],
        by_kind[(int)tpg::PlaceKind::kFrontierRegion]);
  }

  geometry_msgs::msg::Point cell_center(const tpg::PlaceGraphSnapshot& g,
                                        const tpg::CellCoord& c,
                                        double z) const {
    geometry_msgs::msg::Point p;
    p.x = g.origin_x + (c.x + 0.5) * g.resolution_m;
    p.y = g.origin_y + (c.y + 0.5) * g.resolution_m;
    p.z = z;
    return p;
  }

  void publish_markers(const tpg::PlaceGraphSnapshot& g) {
    visualization_msgs::msg::MarkerArray arr;

    // Clear everything from the previous publish first; place count and
    // ids change between refreshes.
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = frame_id_;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    const double res = g.resolution_m;
    auto base_marker = [&](const std::string& ns, int id, int type) {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame_id_;
      m.header.stamp = now();
      m.ns = ns;
      m.id = id;
      m.type = type;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.pose.orientation.w = 1.0;
      return m;
    };

    // One CUBE_LIST for all region cells, per-point colored.
    visualization_msgs::msg::Marker regions =
        base_marker("regions", 0, visualization_msgs::msg::Marker::CUBE_LIST);
    regions.scale.x = res;
    regions.scale.y = res;
    regions.scale.z = 0.02;

    visualization_msgs::msg::Marker conns =
        base_marker("connectors", 0, visualization_msgs::msg::Marker::CUBE_LIST);
    conns.scale.x = res;
    conns.scale.y = res;
    conns.scale.z = 0.05;

    int label_id = 0;
    int arrow_id = 0;
    for (const auto& p : g.places) {
      bool conn = is_connector(p.kind);
      auto& target = conn ? conns : regions;
      std_msgs::msg::ColorRGBA col =
          conn ? connector_color(p.kind, 0.9) : color_for_id(p.place_id, 0.7);
      double z = conn ? 0.06 : 0.01;
      for (const auto& c : p.cell_region) {
        target.points.push_back(cell_center(g, c, z));
        target.colors.push_back(col);
      }

      // Text label at the centroid.
      visualization_msgs::msg::Marker txt = base_marker(
          "labels", label_id++,
          visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
      txt.pose.position.x = p.centroid.x;
      txt.pose.position.y = p.centroid.y;
      txt.pose.position.z = 0.35;
      txt.scale.z = 0.22;
      txt.color = make_color(1.0, 1.0, 1.0, 1.0);
      txt.text = p.place_id + " (" + tpg::to_string(p.kind) + ")";
      if (p.provisional) txt.text += "*";
      arr.markers.push_back(txt);

      // Approach arrow for portals / open transitions.
      if (p.approach_yaw.has_value()) {
        double a = *p.approach_yaw;
        double len = p.width_m.value_or(0.6);
        len = std::max(0.4, std::min(len, 1.5));
        visualization_msgs::msg::Marker arw = base_marker(
            "approach", arrow_id++, visualization_msgs::msg::Marker::ARROW);
        geometry_msgs::msg::Point s, e;
        s.x = p.centroid.x;
        s.y = p.centroid.y;
        s.z = 0.1;
        e.x = p.centroid.x + len * std::cos(a);
        e.y = p.centroid.y + len * std::sin(a);
        e.z = 0.1;
        arw.points.push_back(s);
        arw.points.push_back(e);
        arw.scale.x = 0.04;  // shaft diameter
        arw.scale.y = 0.1;   // head diameter
        arw.scale.z = 0.12;  // head length
        arw.color = make_color(0.1, 1.0, 0.4, 1.0);
        arr.markers.push_back(arw);
      }
    }

    arr.markers.push_back(regions);
    arr.markers.push_back(conns);
    pub_->publish(arr);
  }

  std::string map_topic_;
  std::string frame_id_ = "map";
  int occupancy_threshold_ = 65;
  bool enable_matching_ = true;
  bool warned_rotation_ = false;
  tpg::Config cfg_;

  bool have_prev_ = false;
  tpg::PlaceGraphSnapshot prev_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlaceGraphNode>());
  rclcpp::shutdown();
  return 0;
}
