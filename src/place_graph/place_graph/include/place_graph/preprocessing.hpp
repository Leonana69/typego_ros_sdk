#pragma once

#include "place_graph/types.hpp"

namespace place_graph {

// Phase 1 output. The segmentation mask is the working grid for all
// downstream phases. `free` = 1 where the cell is traversable for
// segmentation purposes (after thin inflation + unknown-pocket
// closure); `unknown` = 1 where the cell is unknown (for frontier
// accounting). Occupied cells appear as `free == 0` and `unknown == 0`.
struct SegmentationGrid {
    int width = 0;
    int height = 0;
    double resolution_m = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;

    // Both row-major, length width*height, values in {0, 1}.
    std::vector<std::uint8_t> free;
    std::vector<std::uint8_t> unknown;

    int index(int x, int y) const { return y * width + x; }
    bool in_bounds(int x, int y) const {
        return x >= 0 && y >= 0 && x < width && y < height;
    }
};

// Phase 1: downsample to seg_resolution, thin-inflate the obstacles,
// then morphologically close interior unknown pockets up to
// unknown_close_m radius.
//
// The downsample uses a conservative rule: a coarse cell is occupied
// if any covered fine cell is occupied; otherwise unknown if any
// covered fine cell is unknown; otherwise free. This errs toward
// preserving walls (so doorways don't accidentally close from
// resampling) and toward keeping frontiers visible.
SegmentationGrid build_segmentation_grid(const MapSnapshot& map,
                                         const Config& cfg);

}  // namespace place_graph
