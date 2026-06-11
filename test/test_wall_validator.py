"""Unit tests for the wall layout validator."""

from concrete_block_assembly_planning.wall_validator import (
    check_collisions,
    check_reach,
    validate_layout,
)

REACH = {
    "base_xy": [0.0, 0.0],
    "reach_min": 1.0,
    "reach_max": 10.0,
    "z_min": -2.0,
    "z_max": 8.0,
}


def _block(bid, x, y, z, yaw=0.0):
    return {"id": bid, "x": x, "y": y, "z": z, "yaw_deg": yaw}


def test_reach_in_and_out():
    assert check_reach(_block("a", 5.0, 0.0, 0.0), REACH)[0] is True
    assert check_reach(_block("b", 0.5, 0.0, 0.0), REACH)[0] is False   # too close
    assert check_reach(_block("c", 11.0, 0.0, 0.0), REACH)[0] is False  # too far
    assert check_reach(_block("d", 5.0, 0.0, 9.0), REACH)[0] is False   # too high


def test_collision_detects_overlap_and_clears_gap():
    dims = [0.9, 0.6, 0.6]
    post = {"id": "post", "position": [5.0, 0.0, 0.0], "dimensions": [0.2, 0.2, 1.0]}
    # block centered on the post overlaps
    assert check_collisions(_block("x", 5.0, 0.0, 0.0), dims, [post]) == ["post"]
    # block well clear does not
    assert check_collisions(_block("y", 8.0, 0.0, 0.0), dims, [post]) == []


def test_yaw_changes_footprint():
    dims = [0.9, 0.6, 0.6]
    obj = {"id": "o", "position": [0.0, 0.42, 0.0], "dimensions": [0.1, 0.1, 0.6]}
    # at yaw=0 the y half-extent is 0.3 (+0.05 obj) -> clears object at 0.42
    assert check_collisions(_block("a", 0.0, 0.0, 0.0, yaw=0.0), dims, [obj]) == []
    # at yaw=90 the y half-extent is 0.45 (+0.05 obj) -> overlaps object at 0.42
    assert check_collisions(_block("b", 0.0, 0.0, 0.0, yaw=90.0), dims, [obj]) == ["o"]


def test_validate_layout_aggregate():
    meta = {"block_size": [0.9, 0.6, 0.6]}
    blocks = [
        _block("ok", 5.0, 0.0, 0.0),
        _block("far", 50.0, 0.0, 0.0),
    ]
    all_valid, results = validate_layout(blocks, meta, static_objects=[], reach=REACH)
    assert all_valid is False
    assert results["ok"]["valid"] is True
    assert results["far"]["valid"] is False
    assert results["far"]["reachable"] is False
