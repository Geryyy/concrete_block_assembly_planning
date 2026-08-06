"""Evaluate the detector's local ground model at arbitrary (x, y).

A faithful Python port of ``blockpose::core::LocalGroundModel::support_z`` (see
``blockpose/cpp/include/blockpose/core/geometry.hpp``). Keep the two in step:
if the C++ fallback rule changes, plans corrected here silently stop matching
the terrain the detector actually fitted.
"""

import math


class TerrainModel:
    """Terrain height lookup built from a ``GroundModel`` message."""

    # A supporting cell can be at most two indices away and still fall inside
    # the 1.75-cell acceptance radius, so the search never scans the whole map.
    _SEARCH_RADIUS_CELLS = 2
    _ACCEPT_RADIUS_CELLS = 1.75

    def __init__(self, msg):
        self.valid = bool(msg.valid)
        self._nx = float(msg.plane_normal.x)
        self._ny = float(msg.plane_normal.y)
        self._nz = float(msg.plane_normal.z)
        self._offset = float(msg.plane_offset)
        self.source = msg.plane_source
        self.cell_size = float(msg.cell_size)
        self._cells = {
            (int(ix), int(iy)): float(z)
            for ix, iy, z in zip(msg.cell_ix, msg.cell_iy, msg.cell_z)
        }
        if abs(self._nz) < 1e-9 or self.cell_size <= 0.0:
            self.valid = False

    @property
    def num_cells(self) -> int:
        return len(self._cells)

    def plane_z(self, x: float, y: float) -> float:
        """World z of the RANSAC plane at (x, y), ignoring the cell map."""
        return -(self._nx * x + self._ny * y + self._offset) / self._nz

    def support_z(self, x: float, y: float):
        """World z of the terrain at (x, y), or None if the model is unusable.

        Returns the bare plane height where no fitted cell is close enough --
        the same fallback the detector applies, not an extrapolation.
        """
        if not self.valid:
            return None
        planar = self.plane_z(x, y)
        if not self._cells:
            return planar

        cell_x = math.floor(x / self.cell_size)
        cell_y = math.floor(y / self.cell_size)
        max_distance_sq = (self.cell_size * self._ACCEPT_RADIUS_CELLS) ** 2
        nearest_distance_sq = math.inf
        nearest_z = planar
        nearest_center = (0.0, 0.0)

        span = range(-self._SEARCH_RADIUS_CELLS, self._SEARCH_RADIUS_CELLS + 1)
        for dx in span:
            for dy in span:
                z = self._cells.get((cell_x + dx, cell_y + dy))
                if z is None:
                    continue
                cx = (cell_x + dx + 0.5) * self.cell_size
                cy = (cell_y + dy + 0.5) * self.cell_size
                distance_sq = (x - cx) ** 2 + (y - cy) ** 2
                if distance_sq < nearest_distance_sq:
                    nearest_distance_sq = distance_sq
                    nearest_z = z
                    nearest_center = (cx, cy)

        if nearest_distance_sq > max_distance_sq:
            return planar
        return planar + nearest_z - self.plane_z(*nearest_center)


def correct_plan_heights(tasks, terrain, plan_ground_z, max_shift_m):
    """Return per-task terrain-corrected positions plus a human-readable report.

    Each block keeps its height *above the plan's ground reference* and is
    re-seated on the measured terrain under its own (x, y), so courses stay
    stacked while the wall follows a slope. Blocks whose correction exceeds
    ``max_shift_m`` are left at their planned height: a shift that large is far
    more likely a bad ground fit than real terrain, and silently dropping a
    course into the dirt is worse than ignoring the measurement.

    ``tasks`` is the plan's task list; the returned list is parallel to it and
    holds ``[x, y, z]`` for every task, corrected or not.
    """
    positions = []
    shifts = []
    rejected = []
    for task in tasks:
        x, y, z = task["position"]
        terrain_z = terrain.support_z(x, y)
        if terrain_z is None:
            positions.append([x, y, z])
            continue
        shift = terrain_z - plan_ground_z
        if abs(shift) > max_shift_m:
            rejected.append((task["block_id"], shift))
            positions.append([x, y, z])
            continue
        positions.append([x, y, z + shift])
        shifts.append(shift)

    if not shifts:
        summary = "no blocks corrected"
    else:
        summary = (
            f"{len(shifts)} block(s) corrected, "
            f"shift {min(shifts):+.3f}..{max(shifts):+.3f} m"
        )
    if rejected:
        worst = max(rejected, key=lambda item: abs(item[1]))
        summary += (
            f"; {len(rejected)} skipped over the {max_shift_m:.2f} m limit "
            f"(worst: {worst[0]} at {worst[1]:+.3f} m)"
        )
    return positions, summary


def infer_plan_ground_z(tasks, block_height_m):
    """The ground level the plan was authored against: bottom of its lowest course.

    Used when the plan does not state ``ground_reference_z`` itself. Assumes the
    lowest block in the plan was meant to rest on the ground, which is what
    every wall plan the expander produces does.
    """
    if not tasks:
        return None
    return min(task["position"][2] for task in tasks) - block_height_m / 2.0
