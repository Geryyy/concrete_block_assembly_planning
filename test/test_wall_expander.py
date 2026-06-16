"""Unit tests for the compact wall-spec expander (pure Python, no ROS)."""

import math

from concrete_block_assembly_planning.wall_expander import (
    build_support_graph,
    compute_layout,
    expand,
    to_wall_plan,
    topo_order,
)

SPEC = {
    "wall": {
        "name": "test_wall",
        "block_size": [0.9, 0.6, 0.6],
        "length_axis": "x",
        "clearance": {"horizontal": 0.1, "vertical": 0.0},
        "gripper_yaw_offset_deg": 90.0,
        "origin": {"x": 1.0, "y": 2.0, "z": 0.0, "yaw_deg": 0.0},
        "courses": [
            {"count": 3, "offset": 0.0},
            {"count": 2, "offset": 0.5},
        ],
    }
}


def test_block_count_and_ids():
    blocks, _ = compute_layout(SPEC)
    assert len(blocks) == 5
    assert [b["id"] for b in blocks] == ["c0_b0", "c0_b1", "c0_b2", "c1_b0", "c1_b1"]


def test_positions_pitch_and_height():
    blocks, meta = compute_layout(SPEC)
    by_id = {b["id"]: b for b in blocks}
    # pitch = block_len (0.9) + horizontal clearance (0.1) = 1.0
    assert meta["pitch"] == 1.0
    # bottom course along +x from origin (yaw=0)
    assert math.isclose(by_id["c0_b0"]["x"], 1.0)
    assert math.isclose(by_id["c0_b1"]["x"], 2.0)
    assert math.isclose(by_id["c0_b2"]["x"], 3.0)
    # second course raised by block height and shifted by its 0.5 offset
    assert math.isclose(by_id["c1_b0"]["z"], 0.6)
    assert math.isclose(by_id["c1_b0"]["x"], 1.5)


def test_yaw_rotates_layout():
    spec = {
        "wall": {
            "block_size": [0.9, 0.6, 0.6],
            "clearance": {"horizontal": 0.1},
            "origin": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 90.0},
            "courses": [{"count": 2, "offset": 0.0}],
        }
    }
    blocks, _ = compute_layout(spec)
    by_id = {b["id"]: b for b in blocks}
    # yaw=90 -> wall runs along +y
    assert math.isclose(by_id["c0_b1"]["x"], 0.0, abs_tol=1e-9)
    assert math.isclose(by_id["c0_b1"]["y"], 1.0)


def test_support_graph_running_bond():
    blocks, _ = compute_layout(SPEC)
    graph = build_support_graph(blocks)
    # bottom course rests on the ground -> no supports
    assert graph["c0_b0"] == []
    # running-bond top block straddles two lower blocks
    assert set(graph["c1_b0"]) == {"c0_b0", "c0_b1"}
    assert set(graph["c1_b1"]) == {"c0_b1", "c0_b2"}


def test_topo_order_is_bottom_up():
    blocks, _ = compute_layout(SPEC)
    order = [b["id"] for b in topo_order(blocks)]
    # every support precedes the block that rests on it
    pos = {bid: i for i, bid in enumerate(order)}
    graph = build_support_graph(blocks)
    for bid, supports in graph.items():
        for s in supports:
            assert pos[s] < pos[bid]


def test_to_wall_plan_absolute_positions():
    blocks, meta = compute_layout(SPEC)
    seq = to_wall_plan(blocks, meta)
    by_id = {item["id"]: item for item in seq}
    # every block carries an absolute position; none are relative
    assert all("absolute_position" in item for item in seq)
    assert all("relative_to" not in item for item in seq)
    # absolute positions match the layout (origin 1,2,0; pitch 1.0; course height 0.6)
    assert by_id["c0_b0"]["absolute_position"] == [1.0, 2.0, 0.0]
    assert by_id["c0_b2"]["absolute_position"] == [3.0, 2.0, 0.0]
    assert by_id["c1_b0"]["absolute_position"] == [1.5, 2.0, 0.6]
    # gripper offset propagated
    assert by_id["c0_b0"]["gripper_yaw_offset_deg"] == 90.0


def test_expand_structure():
    plan = expand(SPEC)
    assert "wall_plans" in plan
    assert "test_wall" in plan["wall_plans"]
    assert len(plan["wall_plans"]["test_wall"]["sequence"]) == 5
    assert plan["defaults"]["block_size"] == [0.9, 0.6, 0.6]
