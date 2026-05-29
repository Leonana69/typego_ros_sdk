#include "test_fixtures.hpp"

namespace typego_place_graph::test {

MapSnapshot make_blank(int w, int h, double res, CellState fill) {
    MapSnapshot m;
    m.width = w;
    m.height = h;
    m.resolution_m = res;
    m.origin_x = 0.0;
    m.origin_y = 0.0;
    m.cells.assign(w * h, fill);
    m.map_version = "test";
    return m;
}

void fill_rect(MapSnapshot& m, int x0, int y0, int x1, int y1, CellState v) {
    for (int y = y0; y < y1; ++y) {
        for (int x = x0; x < x1; ++x) {
            if (x < 0 || y < 0 || x >= m.width || y >= m.height) continue;
            m.cells[y * m.width + x] = v;
        }
    }
}

void rect_walls(MapSnapshot& m, int x0, int y0, int x1, int y1) {
    for (int x = x0; x < x1; ++x) {
        m.cells[y0 * m.width + x] = CellState::kOccupied;
        m.cells[(y1 - 1) * m.width + x] = CellState::kOccupied;
    }
    for (int y = y0; y < y1; ++y) {
        m.cells[y * m.width + x0] = CellState::kOccupied;
        m.cells[y * m.width + (x1 - 1)] = CellState::kOccupied;
    }
}

MapSnapshot one_square_room(double res) {
    // 5m x 5m room (50 cells at 0.10m). Walls 1 cell thick.
    int w = 52, h = 52;
    auto m = make_blank(w, h, res, CellState::kFree);
    // Surround with walls
    for (int x = 0; x < w; ++x) {
        m.cells[0 * w + x] = CellState::kOccupied;
        m.cells[(h - 1) * w + x] = CellState::kOccupied;
    }
    for (int y = 0; y < h; ++y) {
        m.cells[y * w + 0] = CellState::kOccupied;
        m.cells[y * w + (w - 1)] = CellState::kOccupied;
    }
    return m;
}

MapSnapshot two_rooms_with_doorway(double res) {
    // Two 5 m x 5 m rooms with a wall in the middle and a 0.6 m doorway
    // (6 cells at 0.10 m resolution). With r_open = 0.4 m and 1-cell
    // segmentation inflation, doorways up to ~0.7 m separate cleanly
    // under the Euclidean opening; we keep a healthy margin.
    int w = 102, h = 52;
    auto m = make_blank(w, h, res, CellState::kFree);
    rect_walls(m, 0, 0, w, h);
    // Vertical dividing wall at x = 51; doorway y in [25, 31).
    for (int y = 1; y < h - 1; ++y) {
        if (y >= 25 && y < 31) continue;
        m.cells[y * w + 51] = CellState::kOccupied;
    }
    return m;
}

MapSnapshot corridor_with_side_rooms(double res) {
    // ~15 m corridor (x: 0..150), 1.5 m wide. Three rooms (3 m x 2 m)
    // hang off the south side, joined by 0.6 m doorways. Walls between
    // each room and the corridor are 3 cells thick to leave room for
    // a real doorway throat after inflation.
    int w = 152, h = 52;
    auto m = make_blank(w, h, res, CellState::kOccupied);
    fill_rect(m, 1, 28, 151, 43, CellState::kFree);
    fill_rect(m, 11, 1, 41, 25, CellState::kFree);
    fill_rect(m, 51, 1, 81, 25, CellState::kFree);
    fill_rect(m, 91, 1, 121, 25, CellState::kFree);
    fill_rect(m, 23, 25, 29, 28, CellState::kFree);
    fill_rect(m, 63, 25, 69, 28, CellState::kFree);
    fill_rect(m, 103, 25, 109, 28, CellState::kFree);
    return m;
}

MapSnapshot t_intersection(double res) {
    // Three corridors meeting at a central junction room. Each
    // corridor is 1.2 m wide and connects to the junction (2.5 m x
    // 2.5 m) via a 0.6 m doorway. A uniform-width T cannot be
    // segmented by the spec's algorithm — chokepoints are required —
    // so the test fixture explicitly puts a doorway at each junction
    // entry, which mirrors how real maps usually look.
    int w = 202, h = 122;
    auto m = make_blank(w, h, res, CellState::kOccupied);
    // Junction room (2.5 m square).
    fill_rect(m, 75, 50, 100, 75, CellState::kFree);
    // West corridor (12 cells tall = 1.2 m).
    fill_rect(m, 1, 56, 72, 68, CellState::kFree);
    // East corridor.
    fill_rect(m, 103, 56, 201, 68, CellState::kFree);
    // South stem.
    fill_rect(m, 81, 78, 93, 121, CellState::kFree);
    // 0.6 m doorways (6 cells wide, 3 cells deep).
    fill_rect(m, 72, 59, 75, 65, CellState::kFree);   // west doorway
    fill_rect(m, 100, 59, 103, 65, CellState::kFree); // east doorway
    fill_rect(m, 84, 75, 90, 78, CellState::kFree);   // south doorway
    return m;
}

MapSnapshot open_plan(double res) {
    // 10m x 10m one big room (no internal divider).
    int w = 102, h = 102;
    auto m = make_blank(w, h, res, CellState::kFree);
    rect_walls(m, 0, 0, w, h);
    return m;
}

MapSnapshot partially_explored_room(double res) {
    // 5m x 5m room but the right half is unknown.
    auto m = one_square_room(res);
    fill_rect(m, 26, 1, 51, 51, CellState::kUnknown);
    // Reopen the left half so the explored side has actual free cells.
    fill_rect(m, 1, 1, 26, 51, CellState::kFree);
    return m;
}

MapSnapshot noisy_map_with_artifacts(double res) {
    auto m = one_square_room(res);
    // Tiny free-space island outside the room (2x2 cells).
    // Carve a 4x4 occupied bubble first to host the island.
    // Pretend the world extends further: enlarge.
    int new_w = 80;
    int new_h = 80;
    MapSnapshot out;
    out.width = new_w;
    out.height = new_h;
    out.resolution_m = res;
    out.origin_x = 0.0;
    out.origin_y = 0.0;
    out.cells.assign(new_w * new_h, CellState::kOccupied);
    out.map_version = "test";
    // Copy original room.
    for (int y = 0; y < m.height; ++y) {
        for (int x = 0; x < m.width; ++x) {
            out.cells[y * new_w + x] = m.cells[y * m.width + x];
        }
    }
    // Add a 2x2 free island at (70,70). Below the 1.5 m^2 area
    // threshold at 0.10m resolution (which is 150 cells).
    fill_rect(out, 70, 70, 72, 72, CellState::kFree);
    return out;
}

MapSnapshot tight_closet_across_doorway(double res) {
    // Big room (8m x 8m) connected to a tiny closet (0.7m x 0.7m) by
    // a 0.7m doorway. The closet alone is below A_min (~0.5 m^2 < 1.5)
    // and its opening should not survive r_open. Fallback seeding
    // should still produce a place for it.
    int w = 122, h = 82;
    auto m = make_blank(w, h, res, CellState::kOccupied);
    fill_rect(m, 1, 1, 81, 81, CellState::kFree);          // big room
    fill_rect(m, 90, 35, 97, 42, CellState::kFree);        // closet
    fill_rect(m, 81, 36, 90, 41, CellState::kFree);        // 0.5m doorway-ish; widened
    return m;
}

MapSnapshot wide_gap_two_core(double res) {
    // Two rooms connected by an unusually wide opening (3.5 m, well
    // above the 2.5m portal threshold). Each side stays a separate
    // core because opening with r=0.4m won't kiss the other side
    // through such a wide bridge — but the watershed still finds two
    // distinct cores only if each side has its own opening survivor.
    // To force two cores we use longish rooms 6m x 4m and a 3.5m-wide
    // gap with the connecting corridor only 0.4m long.
    int w = 162, h = 82;
    auto m = make_blank(w, h, res, CellState::kOccupied);
    fill_rect(m, 1, 1, 61, 81, CellState::kFree);    // left room
    fill_rect(m, 101, 1, 161, 81, CellState::kFree); // right room
    fill_rect(m, 61, 25, 101, 60, CellState::kFree); // 3.5m wide bridge, 0.4m long
    return m;
}

}  // namespace typego_place_graph::test
