"""Unit tests for the terrain height model and plan height correction.

The model must agree with ``blockpose::core::LocalGroundModel::support_z``, so
these tests pin the parts that are easy to get subtly wrong when re-porting:
the plane fallback outside the acceptance radius, and the fact that a cell's
stored height is an offset *relative to the plane*, not an absolute z.
"""

from types import SimpleNamespace

from concrete_block_assembly_planning.terrain import (
    TerrainModel,
    correct_plan_heights,
    infer_plan_ground_z,
)

BLOCK_H = 0.6


def _model(cells=(), normal=(0.0, 0.0, 1.0), offset=0.0, cell_size=0.5, valid=True):
    """Build a GroundModel-shaped stand-in. `cells` is ((ix, iy, z), ...)."""
    return TerrainModel(
        SimpleNamespace(
            valid=valid,
            plane_normal=SimpleNamespace(x=normal[0], y=normal[1], z=normal[2]),
            plane_offset=offset,
            plane_source="ransac",
            cell_size=cell_size,
            cell_ix=[c[0] for c in cells],
            cell_iy=[c[1] for c in cells],
            cell_z=[c[2] for c in cells],
        )
    )


def _task(bid, x, y, z):
    return {"block_id": bid, "position": [x, y, z]}


def test_plane_only_model_returns_plane_height():
    # z = 1.2 everywhere: normal (0,0,1), offset -1.2 -> support_z = 1.2
    model = _model(offset=-1.2)
    assert model.support_z(0.0, 0.0) == 1.2
    assert model.support_z(50.0, -30.0) == 1.2


def test_tilted_plane_follows_slope():
    # 10% slope along +x: normal (-0.1, 0, 1) normalised loosely, offset 0.
    model = _model(normal=(-0.1, 0.0, 1.0))
    assert model.support_z(0.0, 0.0) == 0.0
    assert model.support_z(10.0, 0.0) == 1.0


def test_cell_overrides_plane_near_its_centre():
    # Cell (0, 0) spans [0, 0.5) x [0, 0.5), centre (0.25, 0.25), fitted at 0.3.
    model = _model(cells=((0, 0, 0.3),))
    assert model.support_z(0.25, 0.25) == 0.3
    assert model.support_z(0.1, 0.1) == 0.3  # same cell, flat plane underneath


def test_far_from_any_cell_falls_back_to_plane():
    # Acceptance radius is 1.75 * cell_size = 0.875 m from the cell centre.
    model = _model(cells=((0, 0, 0.3),))
    assert model.support_z(0.25, 0.25) == 0.3  # inside
    assert model.support_z(20.0, 20.0) == 0.0  # far outside -> plane
    # Just beyond the radius, still the plane rather than the cell's 0.3.
    assert model.support_z(0.25 + 1.0, 0.25) == 0.0


def test_cell_height_is_relative_to_the_plane():
    # On a sloped plane the cell contributes its offset from the plane, so the
    # correction rides on top of the slope instead of replacing it.
    model = _model(cells=((0, 0, 0.3),), normal=(-0.1, 0.0, 1.0))
    # Cell centre (0.25, 0.25): plane is at 0.025, cell says 0.3 -> offset 0.275.
    assert model.support_z(0.25, 0.25) == 0.3
    # 4 m along the slope is outside the radius -> pure plane, no offset.
    assert abs(model.support_z(4.25, 0.25) - 0.425) < 1e-9


def test_invalid_model_yields_no_height():
    assert _model(valid=False).support_z(0.0, 0.0) is None
    # A degenerate normal (vertical plane) cannot give a height either.
    assert _model(normal=(1.0, 0.0, 0.0)).support_z(0.0, 0.0) is None


def test_infer_plan_ground_z_is_bottom_of_lowest_course():
    tasks = [_task("c0_b0", 0.0, 0.0, 0.3), _task("c1_b0", 0.0, 0.0, 0.9)]
    assert infer_plan_ground_z(tasks, BLOCK_H) == 0.0
    assert infer_plan_ground_z([], BLOCK_H) is None


def test_correction_reseats_wall_and_preserves_course_spacing():
    # Plan authored on ground z=0; terrain under it sits 0.4 m higher.
    tasks = [_task("c0_b0", 0.25, 0.25, 0.3), _task("c1_b0", 0.25, 0.25, 0.9)]
    model = _model(cells=((0, 0, 0.4),))
    positions, summary = correct_plan_heights(tasks, model, 0.0, max_shift_m=0.5)
    assert positions[0][2] == 0.7
    assert positions[1][2] == 1.3
    # Courses stay 0.6 apart -- the wall moves, it does not stretch.
    assert abs((positions[1][2] - positions[0][2]) - BLOCK_H) < 1e-9
    assert "2 block(s) corrected" in summary


def test_blocks_follow_terrain_independently():
    # Two cells at different heights: each block re-seats on its own ground.
    model = _model(cells=((0, 0, 0.2), (4, 0, 0.5)))
    tasks = [_task("c0_b0", 0.25, 0.25, 0.3), _task("c0_b1", 2.25, 0.25, 0.3)]
    positions, _ = correct_plan_heights(tasks, model, 0.0, max_shift_m=1.0)
    assert positions[0][2] == 0.5
    assert positions[1][2] == 0.8


def test_oversized_shift_is_rejected_and_reported():
    # A 2 m "terrain" reading is a bad fit, not a hill: keep the planned height.
    model = _model(cells=((0, 0, 2.0),))
    tasks = [_task("c0_b0", 0.25, 0.25, 0.3)]
    positions, summary = correct_plan_heights(tasks, model, 0.0, max_shift_m=0.5)
    assert positions[0][2] == 0.3
    assert "1 skipped" in summary
    assert "c0_b0" in summary


def test_unusable_model_leaves_every_height_untouched():
    model = _model(valid=False)
    tasks = [_task("c0_b0", 0.0, 0.0, 0.3), _task("c1_b0", 0.0, 0.0, 0.9)]
    positions, summary = correct_plan_heights(tasks, model, 0.0, max_shift_m=0.5)
    assert [p[2] for p in positions] == [0.3, 0.9]
    assert summary == "no blocks corrected"
