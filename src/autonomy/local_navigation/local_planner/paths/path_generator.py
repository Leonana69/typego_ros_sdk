#!/usr/bin/env python3
"""Regenerate correspondences.txt for the local planner.

This is a dependency-light (numpy + scipy) port of the "find correspondence"
section of path_generator.m, for environments without MATLAB/Octave.

It does NOT regenerate the path geometry: paths.ply / startPaths.ply /
pathList.ply are read as-is, so the motion primitives stay byte-identical to
what the planner loads. Only paths/correspondences.txt is rewritten.

The voxel->path corridor is an oriented rectangle (the robot body) swept along
each path. The footprint is baked inflated by 1/minPathScale so the effective
world-frame footprint (baked * pathScale) equals the true footprint at the
minimum path scale and stays conservative above it.

Keep the parameters below in sync with path_generator.m and the local_planner
configuration. Run from anywhere:

    python3 path_generator.py
"""

import math
import os
import sys
import time

import numpy as np
from scipy.spatial import cKDTree

# --- Voxel grid (must match localPlanner.cpp and path_generator.m) -----------
VOXEL_SIZE = 0.02
SEARCH_RADIUS = 0.45        # voxel-grid extent; not the collision footprint
OFFSET_X = 3.2
OFFSET_Y = 4.5
VOXEL_NUM_X = 161
VOXEL_NUM_Y = 451

# --- Vehicle footprint for collision checking --------------------------------
# Keep in sync with:
#   vehicleLength / vehicleWidth -> robot.yaml  vehicle.length / vehicle.width
#   footprintMargin              -> local_planner.launch  vehicleFootprintMargin
#   minPathScale                 -> local_planner.launch  minPathScale
VEHICLE_LENGTH = 0.75
VEHICLE_WIDTH = 0.4
FOOTPRINT_MARGIN = 0.05
MIN_PATH_SCALE = 0.675

BAKE_HALF_L = (VEHICLE_LENGTH / 2.0 + FOOTPRINT_MARGIN) / MIN_PATH_SCALE
BAKE_HALF_W = (VEHICLE_WIDTH / 2.0 + FOOTPRINT_MARGIN) / MIN_PATH_SCALE
BAKE_RADIUS = math.hypot(BAKE_HALF_L, BAKE_HALF_W)   # rectangle bounding circle

PATHS_DIR = os.path.dirname(os.path.abspath(__file__))
VOXEL_CHUNK = 4000          # voxels per broad-phase query batch (bounds memory)


def read_paths_ply(path):
    """Read paths.ply -> (x, y, path_id) arrays, one entry per path sample."""
    with open(path, "r") as f:
        if f.readline().strip() != "ply":
            raise ValueError(f"{path}: not a PLY file")
        vertex_count = None
        properties = []
        while True:
            line = f.readline()
            if line == "":
                raise ValueError(f"{path}: unexpected end of header")
            tok = line.split()
            if tok and tok[0] == "element" and tok[1] == "vertex":
                vertex_count = int(tok[2])
            elif tok and tok[0] == "property":
                properties.append(tok[-1])
            elif tok and tok[0] == "end_header":
                break
        if vertex_count is None:
            raise ValueError(f"{path}: missing vertex count")

        data = np.loadtxt(f, max_rows=vertex_count)

    if data.shape[0] != vertex_count:
        raise ValueError(f"{path}: expected {vertex_count} vertices, got {data.shape[0]}")
    # Columns follow the PLY property order: x y z path_id group_id
    try:
        ix = properties.index("x")
        iy = properties.index("y")
        ipath = properties.index("path_id")
    except ValueError as exc:
        raise ValueError(f"{path}: missing expected property ({exc})")
    return data[:, ix].copy(), data[:, iy].copy(), data[:, ipath].astype(np.int64)


def build_voxel_points():
    """Voxel centre coordinates, in the same order path_generator.m emits them
    (indX outer 0..VOXEL_NUM_X-1, indY inner 0..VOXEL_NUM_Y-1)."""
    ind_x = np.arange(VOXEL_NUM_X)
    x = OFFSET_X - VOXEL_SIZE * ind_x                              # (VOXEL_NUM_X,)
    scale_y = x / OFFSET_X + SEARCH_RADIUS / OFFSET_Y * (OFFSET_X - x) / OFFSET_X
    ind_y = np.arange(VOXEL_NUM_Y)
    y = scale_y[:, None] * (OFFSET_Y - VOXEL_SIZE * ind_y)[None, :]  # (X, Y)
    vx = np.repeat(x, VOXEL_NUM_Y)
    vy = y.reshape(-1)
    return vx, vy


def compute_headings(x, y, path_id):
    """Per-sample heading = local path tangent (forward difference within a
    path); the last sample of each path reuses the preceding heading."""
    n = len(x)
    heading = np.empty(n)
    for c in range(n - 1):
        if path_id[c + 1] == path_id[c]:
            heading[c] = math.atan2(y[c + 1] - y[c], x[c + 1] - x[c])
        elif c > 0:
            heading[c] = heading[c - 1]
        else:
            heading[c] = 0.0
    if n > 1:
        heading[n - 1] = heading[n - 2]
    elif n == 1:
        heading[0] = 0.0
    return heading


def main():
    paths_ply = os.path.join(PATHS_DIR, "paths.ply")
    out_file = os.path.join(PATHS_DIR, "correspondences.txt")

    print(f"Footprint (real):   L={VEHICLE_LENGTH} W={VEHICLE_WIDTH} "
          f"margin={FOOTPRINT_MARGIN}")
    print(f"Footprint (baked):  half_L={BAKE_HALF_L:.4f} half_W={BAKE_HALF_W:.4f} "
          f"(inflated by 1/{MIN_PATH_SCALE}); bounding radius={BAKE_RADIUS:.4f}")

    print(f"\nReading {paths_ply}")
    x, y, path_id = read_paths_ply(paths_ply)
    print(f"  {len(x)} path samples, {path_id.max() + 1} paths")

    print("\nComputing path sample headings")
    heading = compute_headings(x, y, path_id)
    cos_h = np.cos(heading)
    sin_h = np.sin(heading)

    print("Preparing voxels")
    vx, vy = build_voxel_points()
    num_voxels = len(vx)
    if num_voxels != VOXEL_NUM_X * VOXEL_NUM_Y:
        raise RuntimeError(f"voxel count mismatch: {num_voxels}")

    print("Collision checking (oriented-rectangle sweep)")
    tree = cKDTree(np.column_stack((x, y)))
    voxel_xy = np.column_stack((vx, vy))

    start = time.time()
    lines = []
    for base in range(0, num_voxels, VOXEL_CHUNK):
        chunk = voxel_xy[base:base + VOXEL_CHUNK]
        # Broad phase: candidate path samples within the bounding circle.
        cand_lists = tree.query_ball_point(chunk, BAKE_RADIUS)
        for k, cand in enumerate(cand_lists):
            vi = base + k
            if cand:
                cand = np.asarray(cand, dtype=np.int64)
                # Narrow phase: keep candidates whose oriented body rectangle,
                # placed at the path sample and aligned with the local tangent,
                # contains the voxel.
                rx = vx[vi] - x[cand]
                ry = vy[vi] - y[cand]
                ch = cos_h[cand]
                sh = sin_h[cand]
                x_local = ch * rx + sh * ry
                y_local = -sh * rx + ch * ry
                in_rect = (np.abs(x_local) <= BAKE_HALF_L) & \
                          (np.abs(y_local) <= BAKE_HALF_W)
                hits = np.unique(path_id[cand[in_rect]])
            else:
                hits = ()
            if len(hits):
                lines.append(f"{vi} " + " ".join(str(p) for p in hits) + " -1")
            else:
                lines.append(f"{vi} -1")
        print(f"  {min(base + VOXEL_CHUNK, num_voxels)}/{num_voxels} voxels",
              end="\r", flush=True)
    print(f"\n  done in {time.time() - start:.1f} s")

    if len(lines) != num_voxels:
        raise RuntimeError(f"expected {num_voxels} lines, produced {len(lines)}")

    print(f"\nWriting {out_file}")
    with open(out_file, "w") as f:
        f.write("\n".join(lines))
        f.write("\n")

    size_mb = os.path.getsize(out_file) / 1e6
    print(f"Processing complete: {num_voxels} voxels, {size_mb:.1f} MB")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:  # noqa: BLE001
        print(f"ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
