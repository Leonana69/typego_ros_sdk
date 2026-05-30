#pragma once

#include "place_graph/place_graph.hpp"

namespace place_graph::test {

MapSnapshot make_blank(int w, int h, double res, CellState fill);
void fill_rect(MapSnapshot& m, int x0, int y0, int x1, int y1, CellState v);
void rect_walls(MapSnapshot& m, int x0, int y0, int x1, int y1);

MapSnapshot one_square_room(double res);
MapSnapshot two_rooms_with_doorway(double res);
MapSnapshot corridor_with_side_rooms(double res);
MapSnapshot t_intersection(double res);
MapSnapshot open_plan(double res);
MapSnapshot partially_explored_room(double res);
MapSnapshot noisy_map_with_artifacts(double res);
MapSnapshot tight_closet_across_doorway(double res);
MapSnapshot wide_gap_two_core(double res);

}  // namespace place_graph::test
