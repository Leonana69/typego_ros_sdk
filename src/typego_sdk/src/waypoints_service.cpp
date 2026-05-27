#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fstream>
#include <cmath>
#include <queue>
#include <deque>
#include <vector>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>
#include <limits>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <opencv2/opencv.hpp>
#include <curl/curl.h>
#include <nlohmann/json.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <algorithm>
#include <utility>

#include "typego_interface/msg/way_point.hpp"
#include "typego_interface/msg/way_point_array.hpp"

#include "typego_sdk/namespace_utils.hpp"
#include "typego_sdk/config_utils.hpp"

struct Waypoint {
    int id;
    double x;
    double y;
    double yaw;
    std::string label;
    std::string semantic_context;
    double confidence;
    std::string source;
    double clearance_m;
    std::string generation_reason;
    std::string pending_refresh_label;
    int pending_refresh_count = 0;
};

struct MapContext {
    std::string label = "unknown_area";
    double clearance_m = 0.0;
    double unknown_ratio = 0.0;
    double free_ratio = 0.0;
    int open_sector_groups = 0;
};

struct VisualSemanticResult {
    std::string label = "no_label";
    double confidence = 0.0;
    bool valid = false;
};

struct MapSnapshot {
    nav_msgs::msg::OccupancyGrid::SharedPtr grid;
    cv::Mat clearance_cells;
    uint64_t version = 0;
};

struct SemanticJob {
    int waypoint_id = 0;
    cv::Mat image;
    MapContext map_context;
    std::string reason;
};

struct WaypointPoseSnapshot {
    int id = 0;
    double x = 0.0;
    double y = 0.0;
};

struct WaypointContextSnapshot {
    int id = 0;
    MapContext context;
};

constexpr double kPi = 3.14159265358979323846;

// Helper function for HTTP requests
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* output) {
    size_t total_size = size * nmemb;
    output->append((char*)contents, total_size);
    return total_size;
}

std::string trim_copy(const std::string &value) {
    const auto first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return "";
    }
    const auto last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

double normalize_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion &q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

std::string prompt_from_label(const std::string &label) {
    const bool starts_with_vowel =
        !label.empty() && std::string("aeiouAEIOU").find(label.front()) != std::string::npos;
    return "a robot camera view of " + std::string(starts_with_vowel ? "an " : "a ") + label;
}

std::string format_double(double value, int precision = 2) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

class AdaptiveWaypointNode : public rclcpp::Node {
public:
    AdaptiveWaypointNode()
        : Node("waypoints_service", typego_sdk::get_namespace_from_env()), 
          tf_buffer_(this->get_clock()), 
          tf_listener_(tf_buffer_) {

        declare_and_load_parameters();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_, 10, std::bind(&AdaptiveWaypointNode::map_callback, this, std::placeholders::_1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic_,
            rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort),
            std::bind(&AdaptiveWaypointNode::image_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::milliseconds(timer_period_ms_), std::bind(&AdaptiveWaypointNode::on_timer, this));

        waypoint_pub_ = this->create_publisher<typego_interface::msg::WayPointArray>("waypoints", 10);
        waypoint_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", 10);
        
        RCLCPP_INFO(this->get_logger(), "Using slam_map_name: %s", slam_map_name_.c_str());
        RCLCPP_INFO(this->get_logger(), "Waypoint spacing %.2fm, turn spacing %.2fm, semantic classes: %zu",
            min_spacing_m_, turn_spacing_m_, semantic_labels_.size());

        edge_service_ip_ = typego_sdk::resolve_edge_service_ip(this);
        edge_service_port_ = std::to_string(typego_sdk::resolve_edge_service_port(this));
        RCLCPP_INFO(this->get_logger(),
            "Using EDGE_SERVICE at %s:%s",
            edge_service_ip_.c_str(), edge_service_port_.c_str());

        load_waypoints(waypoint_file_, slam_map_name_ == "empty_map" && clear_empty_map_waypoints_);
        save_worker_ = std::thread(&AdaptiveWaypointNode::save_worker_loop, this);
        semantic_worker_ = std::thread(&AdaptiveWaypointNode::semantic_worker_loop, this);
    }

    ~AdaptiveWaypointNode() override {
        stop_workers_ = true;
        worker_cv_.notify_all();
        if (semantic_worker_.joinable()) {
            semantic_worker_.join();
        }
        save_cv_.notify_all();
        if (save_worker_.joinable()) {
            save_worker_.join();
        }
    }

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<typego_interface::msg::WayPointArray>::SharedPtr waypoint_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    MapSnapshot latest_map_;
    sensor_msgs::msg::Image::SharedPtr latest_image_;
    std::vector<Waypoint> waypoints_;
    std::string waypoint_file_;
    std::string edge_service_ip_;
    std::string edge_service_port_;
    std::mutex map_mutex_;
    std::mutex image_mutex_;
    std::mutex waypoint_mutex_;
    std::mutex worker_mutex_;
    std::condition_variable worker_cv_;
    std::deque<SemanticJob> semantic_jobs_;
    std::thread semantic_worker_;
    std::mutex save_mutex_;
    std::condition_variable save_cv_;
    std::thread save_worker_;
    std::atomic<bool> stop_workers_{false};
    bool save_requested_ = false;

    int next_id_ = 0;
    std::string slam_map_name_;
    std::string map_frame_ = "map";
    std::string base_frame_ = "base_link";
    std::string map_topic_ = "map";
    std::string camera_topic_ = "camera/color/image_raw";
    int timer_period_ms_ = 100;
    double min_spacing_m_ = 3.0;
    double turn_spacing_m_ = 1.5;
    double heading_change_rad_ = 0.75;
    double context_radius_m_ = 3.0;
    double clearance_search_m_ = 3.0;
    double semantic_confidence_threshold_ = 0.25;
    int semantic_queue_depth_ = 2;
    int context_change_stability_ticks_ = 3;
    int stale_refresh_stability_ticks_ = 3;
    int stale_refresh_interval_ms_ = 3000;
    bool enable_visual_semantics_ = true;
    bool clear_empty_map_waypoints_ = true;
    bool have_last_added_pose_ = false;
    double last_added_x_ = 0.0;
    double last_added_y_ = 0.0;
    double last_added_yaw_ = 0.0;
    std::string last_context_label_;
    std::string stable_current_context_label_;
    std::string pending_current_context_label_;
    int pending_current_context_count_ = 0;
    std::chrono::steady_clock::time_point last_refresh_time_;
    std::vector<std::string> semantic_labels_;
    std::vector<std::string> semantic_queries_;

    void declare_and_load_parameters() {
        const std::vector<std::string> default_labels = {
            "hallway",
            "room",
            "open space",
            "doorway",
            "intersection",
            "charging station",
            "human workspace",
            "shelf or storage",
            "stairs or elevator"
        };

        this->declare_parameter<std::string>("slam_map_name", "empty_map");
        this->declare_parameter<std::string>("waypoint_file", "");
        this->declare_parameter<std::string>("map_frame", map_frame_);
        this->declare_parameter<std::string>("base_frame", base_frame_);
        this->declare_parameter<std::string>("map_topic", map_topic_);
        this->declare_parameter<std::string>("camera_topic", camera_topic_);
        this->declare_parameter<int>("timer_period_ms", timer_period_ms_);
        this->declare_parameter<double>("waypoint_min_spacing_m", min_spacing_m_);
        this->declare_parameter<double>("waypoint_turn_spacing_m", turn_spacing_m_);
        this->declare_parameter<double>("waypoint_heading_change_rad", heading_change_rad_);
        this->declare_parameter<double>("waypoint_context_radius_m", context_radius_m_);
        this->declare_parameter<double>("waypoint_clearance_search_m", clearance_search_m_);
        this->declare_parameter<double>("semantic_confidence_threshold", semantic_confidence_threshold_);
        this->declare_parameter<int>("semantic_queue_depth", semantic_queue_depth_);
        this->declare_parameter<int>("context_change_stability_ticks", context_change_stability_ticks_);
        this->declare_parameter<int>("stale_refresh_stability_ticks", stale_refresh_stability_ticks_);
        this->declare_parameter<int>("stale_refresh_interval_ms", stale_refresh_interval_ms_);
        this->declare_parameter<bool>("enable_visual_semantics", enable_visual_semantics_);
        this->declare_parameter<bool>("clear_empty_map_waypoints", clear_empty_map_waypoints_);
        this->declare_parameter<std::vector<std::string>>("semantic_labels", default_labels);
        this->declare_parameter<std::vector<std::string>>("semantic_queries", std::vector<std::string>{});

        slam_map_name_ = this->get_parameter("slam_map_name").as_string();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        map_topic_ = this->get_parameter("map_topic").as_string();
        camera_topic_ = this->get_parameter("camera_topic").as_string();
        timer_period_ms_ = std::max(50, static_cast<int>(this->get_parameter("timer_period_ms").as_int()));
        min_spacing_m_ = std::max(0.5, this->get_parameter("waypoint_min_spacing_m").as_double());
        turn_spacing_m_ = std::max(0.25, this->get_parameter("waypoint_turn_spacing_m").as_double());
        heading_change_rad_ = std::max(0.1, this->get_parameter("waypoint_heading_change_rad").as_double());
        context_radius_m_ = std::max(0.5, this->get_parameter("waypoint_context_radius_m").as_double());
        clearance_search_m_ = std::max(0.5, this->get_parameter("waypoint_clearance_search_m").as_double());
        semantic_confidence_threshold_ = this->get_parameter("semantic_confidence_threshold").as_double();
        semantic_queue_depth_ = std::max(1, static_cast<int>(this->get_parameter("semantic_queue_depth").as_int()));
        context_change_stability_ticks_ = std::max(1, static_cast<int>(this->get_parameter("context_change_stability_ticks").as_int()));
        stale_refresh_stability_ticks_ = std::max(1, static_cast<int>(this->get_parameter("stale_refresh_stability_ticks").as_int()));
        stale_refresh_interval_ms_ = std::max(250, static_cast<int>(this->get_parameter("stale_refresh_interval_ms").as_int()));
        enable_visual_semantics_ = this->get_parameter("enable_visual_semantics").as_bool();
        clear_empty_map_waypoints_ = this->get_parameter("clear_empty_map_waypoints").as_bool();

        auto configured_labels = this->get_parameter("semantic_labels").as_string_array();
        auto configured_queries = this->get_parameter("semantic_queries").as_string_array();
        semantic_labels_.clear();
        semantic_queries_.clear();
        for (const auto &raw_label : configured_labels) {
            auto label = trim_copy(raw_label);
            if (!label.empty()) {
                semantic_labels_.push_back(label);
            }
        }
        if (semantic_labels_.empty()) {
            semantic_labels_ = default_labels;
        }

        if (configured_queries.size() == semantic_labels_.size()) {
            for (const auto &raw_query : configured_queries) {
                auto query = trim_copy(raw_query);
                semantic_queries_.push_back(query.empty() ? prompt_from_label(semantic_labels_[semantic_queries_.size()]) : query);
            }
        } else {
            for (const auto &label : semantic_labels_) {
                semantic_queries_.push_back(prompt_from_label(label));
            }
            if (!configured_queries.empty()) {
                RCLCPP_WARN(this->get_logger(),
                    "semantic_queries size (%zu) does not match semantic_labels size (%zu); generated default prompts",
                    configured_queries.size(), semantic_labels_.size());
            }
        }

        const auto waypoint_file_param = this->get_parameter("waypoint_file").as_string();
        if (!waypoint_file_param.empty()) {
            waypoint_file_ = waypoint_file_param;
        } else {
            const std::string pkg_typego_sdk = ament_index_cpp::get_package_share_directory("typego_sdk");
            waypoint_file_ = pkg_typego_sdk + "/resource/Map-" + slam_map_name_ + "/waypoints.json";
        }
    }

    std::string loaded_semantic_context(const Waypoint &wp) const {
        return "Loaded waypoint labeled " + wp.label +
               " with persisted navigation pose and no fresh map context yet.";
    }

    void update_last_added_from_loaded() {
        if (waypoints_.empty()) {
            have_last_added_pose_ = false;
            return;
        }
        auto last_it = std::max_element(waypoints_.begin(), waypoints_.end(), [](const Waypoint &a, const Waypoint &b) {
            return a.id < b.id;
        });
        next_id_ = last_it->id + 1;
        have_last_added_pose_ = true;
        last_added_x_ = last_it->x;
        last_added_y_ = last_it->y;
        last_added_yaw_ = last_it->yaw;
        last_context_label_ = last_it->label;
    }

    void load_waypoints(const std::string &file, const bool is_empty_map = false) {
        if (is_empty_map) {
            waypoints_.clear();
            have_last_added_pose_ = false;
            write_waypoints_json({});
            return;
        }

        std::ifstream f(file);
        if (!f.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Waypoint JSON not found: %s", file.c_str());
            return;
        }
        try {
            auto doc = nlohmann::json::parse(f);
            const auto &items = doc.contains("waypoints") ? doc["waypoints"] : doc;
            if (!items.is_array()) {
                RCLCPP_WARN(this->get_logger(), "Waypoint JSON is not an array: %s", file.c_str());
                return;
            }
            waypoints_.clear();
            for (const auto &item : items) {
                Waypoint wp;
                wp.id = item.value("id", 0);
                wp.x = item.value("x", 0.0);
                wp.y = item.value("y", 0.0);
                wp.yaw = item.value("yaw", 0.0);
                wp.label = item.value("label", "no_label");
                wp.semantic_context = item.value("semantic_context", "");
                wp.confidence = item.value("confidence", 0.0);
                wp.source = item.value("source", "loaded");
                wp.clearance_m = item.value("clearance_m", 0.0);
                wp.generation_reason = item.value("generation_reason", "loaded");
                if (wp.semantic_context.empty()) {
                    wp.semantic_context = loaded_semantic_context(wp);
                }
                waypoints_.push_back(wp);
            }
            update_last_added_from_loaded();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load waypoint JSON %s: %s", file.c_str(), e.what());
            waypoints_.clear();
        }
    }

    bool atomic_write_text(const std::string &file, const std::string &content) {
        std::filesystem::path file_path(file);
        std::filesystem::path dir_path = file_path.parent_path();
        if (!dir_path.empty() && !std::filesystem::exists(dir_path)) {
            try {
                std::filesystem::create_directories(dir_path);
            } catch (const std::filesystem::filesystem_error& e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to create waypoint directory: %s", e.what());
                return false;
            }
        }

        const std::string tmp_file = file + ".tmp";
        {
            std::ofstream f(tmp_file, std::ios::trunc);
            if (!f.is_open()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint temp file: %s", tmp_file.c_str());
                return false;
            }
            f << content;
        }

        std::error_code ec;
        std::filesystem::rename(tmp_file, file, ec);
        if (ec) {
            std::filesystem::remove(file, ec);
            ec.clear();
            std::filesystem::rename(tmp_file, file, ec);
        }
        if (ec) {
            RCLCPP_ERROR(this->get_logger(), "Failed to replace waypoint file %s: %s", file.c_str(), ec.message().c_str());
            return false;
        }
        return true;
    }

    void write_waypoints_json(const std::vector<Waypoint> &snapshot) {
        nlohmann::json doc;
        doc["version"] = 2;
        doc["waypoints"] = nlohmann::json::array();
        for (const auto &wp : snapshot) {
            doc["waypoints"].push_back({
                {"id", wp.id},
                {"x", wp.x},
                {"y", wp.y},
                {"yaw", wp.yaw},
                {"label", wp.label},
                {"semantic_context", wp.semantic_context},
                {"confidence", wp.confidence},
                {"source", wp.source},
                {"clearance_m", wp.clearance_m},
                {"generation_reason", wp.generation_reason}
            });
        }
        atomic_write_text(waypoint_file_, doc.dump(2) + "\n");
    }

    void schedule_save() {
        {
            std::lock_guard<std::mutex> lock(save_mutex_);
            save_requested_ = true;
        }
        save_cv_.notify_one();
    }

    void save_worker_loop() {
        while (true) {
            {
                std::unique_lock<std::mutex> lock(save_mutex_);
                save_cv_.wait(lock, [this]() { return save_requested_ || stop_workers_.load(); });
                if (!save_requested_ && stop_workers_.load()) {
                    break;
                }
                save_requested_ = false;
            }

            std::vector<Waypoint> snapshot;
            {
                std::lock_guard<std::mutex> lock(waypoint_mutex_);
                snapshot = waypoints_;
            }
            write_waypoints_json(snapshot);

            if (stop_workers_.load()) {
                std::lock_guard<std::mutex> lock(save_mutex_);
                if (!save_requested_) {
                    break;
                }
            }
        }
    }

    cv::Mat build_clearance_cells(const nav_msgs::msg::OccupancyGrid &grid) const {
        if (grid.info.width == 0 || grid.info.height == 0) {
            return {};
        }
        cv::Mat traversable(static_cast<int>(grid.info.height), static_cast<int>(grid.info.width), CV_8UC1);
        for (int y = 0; y < static_cast<int>(grid.info.height); ++y) {
            for (int x = 0; x < static_cast<int>(grid.info.width); ++x) {
                const int8_t value = grid.data[grid_index(grid, x, y)];
                traversable.at<uint8_t>(y, x) = (value >= 0 && value <= 50) ? 255 : 0;
            }
        }
        cv::Mat distance_cells;
        cv::distanceTransform(traversable, distance_cells, cv::DIST_L2, 3);
        return distance_cells;
    }

    void map_callback(nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        MapSnapshot snapshot;
        snapshot.grid = msg;
        snapshot.clearance_cells = build_clearance_cells(*msg);
        {
            std::lock_guard<std::mutex> lock(map_mutex_);
            snapshot.version = latest_map_.version + 1;
            latest_map_ = snapshot;
        }
    }

    MapSnapshot get_map_snapshot() {
        std::lock_guard<std::mutex> lock(map_mutex_);
        return latest_map_;
    }

    void image_callback(sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(image_mutex_);
        latest_image_ = msg;
    }

    bool world_to_grid(const nav_msgs::msg::OccupancyGrid &grid, double x, double y, int &cx, int &cy) const {
        if (grid.info.resolution <= 0.0) {
            return false;
        }
        cx = static_cast<int>(std::floor((x - grid.info.origin.position.x) / grid.info.resolution));
        cy = static_cast<int>(std::floor((y - grid.info.origin.position.y) / grid.info.resolution));
        return cx >= 0 && cy >= 0 &&
               cx < static_cast<int>(grid.info.width) &&
               cy < static_cast<int>(grid.info.height);
    }

    int grid_index(const nav_msgs::msg::OccupancyGrid &grid, int cx, int cy) const {
        return cy * static_cast<int>(grid.info.width) + cx;
    }

    bool is_free_cell(const nav_msgs::msg::OccupancyGrid &grid, int cx, int cy) const {
        if (cx < 0 || cy < 0 ||
            cx >= static_cast<int>(grid.info.width) ||
            cy >= static_cast<int>(grid.info.height)) {
            return false;
        }
        const int8_t value = grid.data[grid_index(grid, cx, cy)];
        return value >= 0 && value <= 50;
    }

    double ray_free_distance(const nav_msgs::msg::OccupancyGrid &grid, int cx, int cy, double angle, double max_range_m) const {
        const int max_cells = static_cast<int>(std::ceil(max_range_m / grid.info.resolution));
        int x0 = cx;
        int y0 = cy;
        const int x1 = cx + static_cast<int>(std::round(std::cos(angle) * max_cells));
        const int y1 = cy + static_cast<int>(std::round(std::sin(angle) * max_cells));
        const int dx = std::abs(x1 - x0);
        const int dy = std::abs(y1 - y0);
        const int sx = x0 < x1 ? 1 : -1;
        const int sy = y0 < y1 ? 1 : -1;
        int err = dx - dy;
        int travelled_cells = 0;

        while (travelled_cells < max_cells) {
            const int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                x0 += sx;
            }
            if (e2 < dx) {
                err += dx;
                y0 += sy;
            }
            ++travelled_cells;

            if (x0 < 0 || y0 < 0 ||
                x0 >= static_cast<int>(grid.info.width) ||
                y0 >= static_cast<int>(grid.info.height)) {
                break;
            }
            const int8_t value = grid.data[grid_index(grid, x0, y0)];
            if (value < 0 || value > 50) {
                break;
            }
        }
        return std::max(0, travelled_cells - 1) * static_cast<double>(grid.info.resolution);
    }

    double lookup_clearance_m(const MapSnapshot &snapshot, int cx, int cy) const {
        if (!snapshot.grid || snapshot.clearance_cells.empty() ||
            cx < 0 || cy < 0 ||
            cx >= snapshot.clearance_cells.cols ||
            cy >= snapshot.clearance_cells.rows) {
            return 0.0;
        }
        return std::min(
            static_cast<double>(snapshot.clearance_cells.at<float>(cy, cx)) *
                static_cast<double>(snapshot.grid->info.resolution),
            clearance_search_m_);
    }

    int count_open_sector_groups(const std::vector<bool> &open_sectors) const {
        if (open_sectors.empty()) {
            return 0;
        }
        const bool any_open = std::any_of(open_sectors.begin(), open_sectors.end(), [](bool open) { return open; });
        if (!any_open) {
            return 0;
        }
        int groups = 0;
        for (size_t i = 0; i < open_sectors.size(); ++i) {
            const size_t prev = (i + open_sectors.size() - 1) % open_sectors.size();
            if (open_sectors[i] && !open_sectors[prev]) {
                ++groups;
            }
        }
        return groups == 0 ? 1 : groups;
    }

    MapContext analyze_map_context(const MapSnapshot &snapshot, double x, double y) const {
        MapContext context;
        if (!snapshot.grid) {
            context.label = "unmapped";
            return context;
        }
        const auto &grid = *snapshot.grid;
        int cx = 0;
        int cy = 0;
        if (!world_to_grid(grid, x, y, cx, cy)) {
            context.label = "out_of_map";
            return context;
        }

        const int radius_cells = static_cast<int>(std::ceil(context_radius_m_ / grid.info.resolution));
        int free_cells = 0;
        int unknown_cells = 0;
        int occupied_cells = 0;
        int total_cells = 0;
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const double dist_m = std::hypot(dx, dy) * grid.info.resolution;
                if (dist_m > context_radius_m_) {
                    continue;
                }
                const int nx = cx + dx;
                const int ny = cy + dy;
                if (nx < 0 || ny < 0 ||
                    nx >= static_cast<int>(grid.info.width) ||
                    ny >= static_cast<int>(grid.info.height)) {
                    ++unknown_cells;
                    ++total_cells;
                    continue;
                }
                const int8_t value = grid.data[grid_index(grid, nx, ny)];
                if (value < 0) {
                    ++unknown_cells;
                } else if (value > 50) {
                    ++occupied_cells;
                } else {
                    ++free_cells;
                }
                ++total_cells;
            }
        }

        context.clearance_m = lookup_clearance_m(snapshot, cx, cy);
        context.unknown_ratio = total_cells > 0 ? static_cast<double>(unknown_cells) / total_cells : 0.0;
        context.free_ratio = total_cells > 0 ? static_cast<double>(free_cells) / total_cells : 0.0;

        constexpr int sector_count = 16;
        std::vector<bool> open_sectors(sector_count, false);
        int open_count = 0;
        for (int i = 0; i < sector_count; ++i) {
            const double angle = 2.0 * kPi * static_cast<double>(i) / sector_count;
            const double distance = ray_free_distance(grid, cx, cy, angle, context_radius_m_);
            open_sectors[i] = distance >= context_radius_m_ * 0.75;
            if (open_sectors[i]) {
                ++open_count;
            }
        }
        context.open_sector_groups = count_open_sector_groups(open_sectors);

        if (context.unknown_ratio > 0.35 && context.free_ratio > 0.15) {
            context.label = "exploration frontier";
        } else if (open_count >= 10 && context.clearance_m > 1.2) {
            context.label = "open space";
        } else if (context.open_sector_groups >= 3) {
            context.label = "intersection";
        } else if (context.clearance_m < 0.8 && context.open_sector_groups >= 2) {
            context.label = "doorway";
        } else if (context.open_sector_groups == 2 || open_count <= 5) {
            context.label = "hallway";
        } else if (occupied_cells > free_cells / 2) {
            context.label = "room";
        } else {
            context.label = "free space";
        }

        return context;
    }

    double nearest_waypoint_distance(double x, double y, const MapSnapshot &map_snapshot) {
        std::vector<Waypoint> waypoints_snapshot;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            waypoints_snapshot = waypoints_;
        }
        if (waypoints_snapshot.empty()) {
            return std::numeric_limits<double>::infinity();
        }

        if (!map_snapshot.grid || map_snapshot.grid->info.resolution <= 0.0) {
            double min_dist = std::numeric_limits<double>::infinity();
            for (const auto &wp : waypoints_snapshot) {
                min_dist = std::min(min_dist, std::hypot(x - wp.x, y - wp.y));
            }
            return min_dist;
        }

        const auto &grid = *map_snapshot.grid;
        int robot_cx = 0;
        int robot_cy = 0;
        if (!world_to_grid(grid, x, y, robot_cx, robot_cy) || !is_free_cell(grid, robot_cx, robot_cy)) {
            double min_dist = std::numeric_limits<double>::infinity();
            for (const auto &wp : waypoints_snapshot) {
                min_dist = std::min(min_dist, std::hypot(x - wp.x, y - wp.y));
            }
            return min_dist;
        }

        const int width = static_cast<int>(grid.info.width);
        const int height = static_cast<int>(grid.info.height);
        const int max_cells = static_cast<int>(std::ceil(min_spacing_m_ * 2.0 / grid.info.resolution));
        std::queue<std::pair<int, int>> q;
        std::vector<int> dist(width * height, -1);
        q.push({robot_cx, robot_cy});
        dist[grid_index(grid, robot_cx, robot_cy)] = 0;

        constexpr int dx[] = {1, -1, 0, 0};
        constexpr int dy[] = {0, 0, 1, -1};
        while (!q.empty()) {
            auto [cx, cy] = q.front();
            q.pop();
            const int current_dist = dist[grid_index(grid, cx, cy)];
            if (current_dist >= max_cells) {
                continue;
            }
            for (int i = 0; i < 4; ++i) {
                const int nx = cx + dx[i];
                const int ny = cy + dy[i];
                if (!is_free_cell(grid, nx, ny)) {
                    continue;
                }
                const int idx = grid_index(grid, nx, ny);
                if (dist[idx] >= 0) {
                    continue;
                }
                dist[idx] = current_dist + 1;
                q.push({nx, ny});
            }
        }

        double min_reachable_dist = std::numeric_limits<double>::infinity();
        for (const auto &wp : waypoints_snapshot) {
            const double euclidean = std::hypot(x - wp.x, y - wp.y);
            if (euclidean > min_spacing_m_ * 2.0) {
                continue;
            }
            int wp_cx = 0;
            int wp_cy = 0;
            if (!world_to_grid(grid, wp.x, wp.y, wp_cx, wp_cy)) {
                continue;
            }
            const int dist_cells = dist[grid_index(grid, wp_cx, wp_cy)];
            if (dist_cells >= 0) {
                min_reachable_dist = std::min(
                    min_reachable_dist,
                    static_cast<double>(dist_cells) * static_cast<double>(grid.info.resolution));
            }
        }
        return min_reachable_dist;
    }

    std::string build_semantic_context(
        const MapContext &map_context,
        const std::string &visual_label,
        double visual_confidence,
        const std::string &source,
        const std::string &reason) const {
        std::ostringstream ss;
        ss << "Map context suggests " << map_context.label
           << " with about " << format_double(map_context.clearance_m)
           << " m clearance";
        if (map_context.open_sector_groups > 0) {
            ss << " and " << map_context.open_sector_groups << " open direction group";
            if (map_context.open_sector_groups != 1) {
                ss << "s";
            }
        }
        ss << ". Added because of " << reason << ".";
        if (!visual_label.empty()) {
            ss << " Visual semantics suggested " << visual_label
               << " with confidence " << format_double(visual_confidence, 2) << ".";
        }
        if (source == "loaded") {
            ss << " This record was restored from disk.";
        }
        return ss.str();
    }

    std::string stable_current_context_label(const std::string &raw_label) {
        if (stable_current_context_label_.empty()) {
            stable_current_context_label_ = raw_label;
            pending_current_context_label_ = raw_label;
            pending_current_context_count_ = context_change_stability_ticks_;
            return raw_label;
        }

        if (raw_label == stable_current_context_label_) {
            pending_current_context_label_ = raw_label;
            pending_current_context_count_ = context_change_stability_ticks_;
            return stable_current_context_label_;
        }

        if (raw_label == pending_current_context_label_) {
            ++pending_current_context_count_;
        } else {
            pending_current_context_label_ = raw_label;
            pending_current_context_count_ = 1;
        }

        if (pending_current_context_count_ >= context_change_stability_ticks_) {
            stable_current_context_label_ = raw_label;
        }
        return stable_current_context_label_;
    }

    bool should_add_waypoint(
        double x,
        double y,
        double yaw,
        const MapContext &map_context,
        double nearest_dist,
        std::string &reason) const {
        if (nearest_dist <= turn_spacing_m_) {
            return false;
        }
        if (!have_last_added_pose_) {
            reason = "first_pose";
            return true;
        }

        const double dist_from_last = std::hypot(x - last_added_x_, y - last_added_y_);
        const double heading_delta = std::abs(normalize_angle(yaw - last_added_yaw_));

        if (nearest_dist > min_spacing_m_) {
            reason = "distance";
            return true;
        }
        if (dist_from_last > turn_spacing_m_ && heading_delta > heading_change_rad_) {
            reason = "turn";
            return true;
        }
        if (dist_from_last > turn_spacing_m_ && map_context.label != last_context_label_) {
            reason = "context_change";
            return true;
        }
        if ((map_context.label == "intersection" || map_context.label == "doorway" ||
             map_context.label == "exploration frontier") &&
            dist_from_last > turn_spacing_m_) {
            reason = map_context.label;
            return true;
        }

        return false;
    }

    bool waypoint_label_can_follow_map(const Waypoint &wp) const {
        return wp.source == "map" || wp.source == "loaded" ||
               wp.confidence < semantic_confidence_threshold_;
    }

    void refresh_stale_waypoints(const MapSnapshot &map_snapshot) {
        if (!map_snapshot.grid) {
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (last_refresh_time_.time_since_epoch().count() != 0 &&
            std::chrono::duration_cast<std::chrono::milliseconds>(now - last_refresh_time_).count() <
                stale_refresh_interval_ms_) {
            return;
        }
        last_refresh_time_ = now;

        std::vector<WaypointPoseSnapshot> poses;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            poses.reserve(waypoints_.size());
            for (const auto &wp : waypoints_) {
                poses.push_back({wp.id, wp.x, wp.y});
            }
        }

        std::vector<WaypointContextSnapshot> contexts;
        contexts.reserve(poses.size());
        for (const auto &pose : poses) {
            contexts.push_back({pose.id, analyze_map_context(map_snapshot, pose.x, pose.y)});
        }

        bool updated = false;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            for (const auto &entry : contexts) {
                auto wp_it = std::find_if(waypoints_.begin(), waypoints_.end(), [&entry](const Waypoint &wp) {
                    return wp.id == entry.id;
                });
                if (wp_it == waypoints_.end()) {
                    continue;
                }
                auto &wp = *wp_it;
                const auto &context = entry.context;
                const bool map_owned = waypoint_label_can_follow_map(wp);

                if (map_owned && context.label != wp.label) {
                    if (wp.pending_refresh_label == context.label) {
                        ++wp.pending_refresh_count;
                    } else {
                        wp.pending_refresh_label = context.label;
                        wp.pending_refresh_count = 1;
                    }

                    if (wp.pending_refresh_count >= stale_refresh_stability_ticks_) {
                        wp.label = context.label;
                        wp.source = "map";
                        wp.generation_reason = "context_refresh";
                        wp.confidence = 0.0;
                        wp.clearance_m = context.clearance_m;
                        wp.semantic_context = build_semantic_context(context, "", 0.0, wp.source, wp.generation_reason);
                        wp.pending_refresh_label.clear();
                        wp.pending_refresh_count = 0;
                        updated = true;
                    }
                    continue;
                }

                wp.pending_refresh_label.clear();
                wp.pending_refresh_count = 0;
                if (std::abs(wp.clearance_m - context.clearance_m) > 0.2) {
                    wp.clearance_m = context.clearance_m;
                    if (map_owned) {
                        wp.semantic_context = build_semantic_context(context, "", 0.0, wp.source, wp.generation_reason);
                    }
                    updated = true;
                }
            }
        }

        if (updated) {
            schedule_save();
            publish_waypoints();
        }
    }

    cv::Mat latest_image_as_bgr() {
        std::lock_guard<std::mutex> lock(image_mutex_);
        if (!latest_image_) {
            return {};
        }

        cv::Mat image;
        if (latest_image_->encoding == sensor_msgs::image_encodings::BGR8) {
            image = cv::Mat(latest_image_->height, latest_image_->width, CV_8UC3,
                            const_cast<uint8_t*>(latest_image_->data.data()),
                            latest_image_->step).clone();
        } else if (latest_image_->encoding == sensor_msgs::image_encodings::RGB8) {
            cv::Mat rgb(latest_image_->height, latest_image_->width, CV_8UC3,
                        const_cast<uint8_t*>(latest_image_->data.data()),
                        latest_image_->step);
            cv::cvtColor(rgb, image, cv::COLOR_RGB2BGR);
        } else if (latest_image_->encoding == sensor_msgs::image_encodings::MONO8) {
            cv::Mat mono(latest_image_->height, latest_image_->width, CV_8UC1,
                         const_cast<uint8_t*>(latest_image_->data.data()),
                         latest_image_->step);
            cv::cvtColor(mono, image, cv::COLOR_GRAY2BGR);
        } else {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                "Unsupported waypoint semantic image encoding: %s", latest_image_->encoding.c_str());
            return {};
        }

        cv::Mat resized;
        cv::resize(image, resized, cv::Size(640, 360));
        return resized;
    }

    void enqueue_semantic_job(SemanticJob job) {
        {
            std::lock_guard<std::mutex> lock(worker_mutex_);
            if (semantic_jobs_.size() >= static_cast<size_t>(semantic_queue_depth_)) {
                RCLCPP_WARN(this->get_logger(),
                    "Skipping waypoint visual semantics because the CLIP queue is full");
                return;
            }
            semantic_jobs_.push_back(std::move(job));
        }
        worker_cv_.notify_one();
    }

    void apply_visual_semantics(const SemanticJob &job, const VisualSemanticResult &result) {
        bool updated = false;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            for (auto &wp : waypoints_) {
                if (wp.id != job.waypoint_id) {
                    continue;
                }
                const bool confident = result.confidence >= semantic_confidence_threshold_;
                if (confident) {
                    wp.label = result.label;
                    wp.source = "visual+map";
                } else {
                    wp.source = "map";
                }
                wp.confidence = result.confidence;
                wp.semantic_context = build_semantic_context(
                    job.map_context, result.label, result.confidence, wp.source, job.reason);
                updated = true;
                break;
            }
        }
        if (updated) {
            schedule_save();
            publish_waypoints();
        }
    }

    void semantic_worker_loop() {
        while (true) {
            SemanticJob job;
            {
                std::unique_lock<std::mutex> lock(worker_mutex_);
                worker_cv_.wait(lock, [this]() {
                    return !semantic_jobs_.empty() || stop_workers_.load();
                });
                if (semantic_jobs_.empty() && stop_workers_.load()) {
                    break;
                }
                job = std::move(semantic_jobs_.front());
                semantic_jobs_.pop_front();
            }

            const VisualSemanticResult result = send_image_for_detection(job.image);
            if (result.valid) {
                apply_visual_semantics(job, result);
            }
        }
    }

    void on_timer() {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_.lookupTransform(
                map_frame_,
                base_frame_,
                tf2::TimePointZero);
        } catch (const tf2::TransformException &e) {
            return;
        }

        double x = tf.transform.translation.x;
        double y = tf.transform.translation.y;
        const double yaw = yaw_from_quaternion(tf.transform.rotation);

        MapSnapshot map_snapshot = get_map_snapshot();
        MapContext map_context;
        if (map_snapshot.grid) {
            map_context = analyze_map_context(map_snapshot, x, y);
            map_context.label = stable_current_context_label(map_context.label);
            refresh_stale_waypoints(map_snapshot);
        } else {
            map_context.label = "unmapped";
        }

        const double nearest_dist = nearest_waypoint_distance(x, y, map_snapshot);
        std::string reason;
        if (!should_add_waypoint(x, y, yaw, map_context, nearest_dist, reason)) {
            publish_waypoints();
            return;
        }

        int waypoint_id = 0;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            waypoint_id = next_id_++;
            const std::string initial_label = map_context.label.empty() ? "pending" : map_context.label;
            Waypoint wp{
                waypoint_id,
                x,
                y,
                yaw,
                initial_label,
                build_semantic_context(map_context, "", 0.0, "map", reason),
                0.0,
                "map",
                map_context.clearance_m,
                reason,
                "",
                0
            };
            waypoints_.push_back(wp);
            have_last_added_pose_ = true;
            last_added_x_ = x;
            last_added_y_ = y;
            last_added_yaw_ = yaw;
            last_context_label_ = map_context.label;
        }

        RCLCPP_INFO(this->get_logger(),
            "Adding waypoint %d at (%.2f, %.2f), label=%s, reason=%s, nearest=%.2fm",
            waypoint_id, x, y, map_context.label.c_str(), reason.c_str(), nearest_dist);
        schedule_save();
        publish_waypoints();

        if (!enable_visual_semantics_) {
            return;
        }

        cv::Mat cloned_image = latest_image_as_bgr();
        if (cloned_image.empty()) {
            RCLCPP_WARN(this->get_logger(), "Skipping waypoint visual semantics because image is empty");
            return;
        }

        enqueue_semantic_job({waypoint_id, std::move(cloned_image), map_context, reason});
    }

    VisualSemanticResult send_image_for_detection(const cv::Mat& image) {
        RCLCPP_INFO(this->get_logger(), "Sending image for detection");
        // Convert image to webp format
        std::vector<uchar> buffer;
        if (!cv::imencode(".webp", image, buffer)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to encode waypoint image");
            return {};
        }
        
        // Prepare JSON data
        nlohmann::json json_data = {
            {"robot_info", "typego"},
            {"service_type", "clip"},
            {"image_id", 1},
            {"queries", semantic_queries_}
        };
        
        // Prepare the HTTP request
        CURL *curl = curl_easy_init();
        if (curl) {
            curl_mime *mime = curl_mime_init(curl);

            // Add image part
            curl_mimepart *part = curl_mime_addpart(mime);
            curl_mime_name(part, "image");
            curl_mime_filename(part, "image.webp");
            curl_mime_data(part, reinterpret_cast<const char*>(buffer.data()), buffer.size());
            curl_mime_type(part, "image/webp");

            // Add JSON part
            std::string json_str = json_data.dump();
            curl_mimepart *json_part = curl_mime_addpart(mime);
            curl_mime_name(json_part, "json_data");
            curl_mime_data(json_part, json_str.c_str(), json_str.size());

            // Set the URL
            std::string url = "http://" + edge_service_ip_ + ":" + edge_service_port_ + "/process";
            curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
            curl_easy_setopt(curl, CURLOPT_MIMEPOST, mime);

            // Response handling
            std::string response_string;
            curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
            curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_string);

            curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 2L);  // 2 seconds to connect
            curl_easy_setopt(curl, CURLOPT_TIMEOUT, 5L);         // 5 seconds total for the operation

            // Perform the request
            CURLcode res = curl_easy_perform(curl);
            if (res != CURLE_OK) {
                RCLCPP_ERROR(this->get_logger(), "curl_easy_perform() failed: %s", curl_easy_strerror(res));
            } else {
                try {
                    auto response_json = nlohmann::json::parse(response_string);
                    auto results = response_json["result"].get<std::vector<float>>();
                    if (results.empty()) {
                        RCLCPP_ERROR(this->get_logger(), "CLIP response did not include scores");
                        curl_mime_free(mime);
                        curl_easy_cleanup(curl);
                        return {};
                    }
                    auto max_it = std::max_element(results.begin(), results.end());
                    size_t best_index = std::distance(results.begin(), max_it);
                    RCLCPP_INFO(this->get_logger(), "CLIP Response: %s", response_string.c_str());
                    curl_mime_free(mime);
                    curl_easy_cleanup(curl);
                    VisualSemanticResult result;
                    if (best_index < semantic_labels_.size()) {
                        result.label = semantic_labels_[best_index];
                    }
                    result.confidence = static_cast<double>(*max_it);
                    result.valid = true;
                    return result;
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON response: %s", e.what());
                }
            }

            // Cleanup
            curl_mime_free(mime);
            curl_easy_cleanup(curl);
        }

        return {};
    }

    void publish_waypoints() {
        typego_interface::msg::WayPointArray array;
        visualization_msgs::msg::MarkerArray marker_array;
        std::vector<Waypoint> snapshot;
        {
            std::lock_guard<std::mutex> lock(waypoint_mutex_);
            snapshot = waypoints_;
        }

        for (size_t i = 0; i < snapshot.size(); ++i) {
            const auto &wp = snapshot[i];
            visualization_msgs::msg::Marker m;
            m.header.frame_id = map_frame_;
            m.header.stamp = this->now();
            m.ns = "waypoints";
            m.id = wp.id;
            m.type = m.SPHERE;
            m.action = m.ADD;
            m.pose.position.x = wp.x;
            m.pose.position.y = wp.y;
            m.pose.position.z = 0.0;
            m.scale.x = m.scale.y = m.scale.z = 0.2;
            m.color.r = 0.0f;
            m.color.g = 1.0f;
            m.color.b = 0.0f;
            m.color.a = 1.0f;
            marker_array.markers.push_back(m);

            // Add text marker for the label
            visualization_msgs::msg::Marker text_marker;
            text_marker.header.frame_id = map_frame_;
            text_marker.header.stamp = this->now();
            text_marker.ns = "waypoint_labels";
            text_marker.id = wp.id;
            text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
            text_marker.action = visualization_msgs::msg::Marker::ADD;
            text_marker.pose.position.x = wp.x;
            text_marker.pose.position.y = wp.y;
            text_marker.pose.position.z = 0.5;  // Above the waypoint
            text_marker.scale.z = 0.3;  // Text height
            text_marker.color.r = 1.0f;
            text_marker.color.g = 1.0f;
            text_marker.color.b = 1.0f;
            text_marker.color.a = 1.0f;
            text_marker.text = wp.label.empty() ? "no_label" : wp.label;
            marker_array.markers.push_back(text_marker);

            typego_interface::msg::WayPoint wp_msg;
            wp_msg.id = wp.id;
            wp_msg.x = wp.x;
            wp_msg.y = wp.y;
            wp_msg.label = wp.label;
            wp_msg.yaw = wp.yaw;
            wp_msg.semantic_context = wp.semantic_context;
            wp_msg.confidence = wp.confidence;
            wp_msg.source = wp.source;
            wp_msg.clearance_m = wp.clearance_m;
            wp_msg.generation_reason = wp.generation_reason;
            array.waypoints.push_back(wp_msg);
        }
        waypoint_pub_->publish(array);
        waypoint_marker_pub_->publish(marker_array);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AdaptiveWaypointNode>();
    rclcpp::spin(node);
    node.reset();
    rclcpp::shutdown();
    return 0;
}
