"""A slot binds to the nearest pickable block when its bound one cannot be picked now."""

import math

from concrete_block_assembly_planning.wall_plan_server import (
    WallPlanServer,
    covered_reason,
    covering_blocks,
    free_block_for_slot,
    yaw_to_quaternion,
)
from concrete_block_world_model_interfaces.msg import Block
from geometry_msgs.msg import PoseStamped


def block(
    id_, x, y=0.0, z=0.0, yaw_deg=0.0, pose=Block.POSE_PRECISE, task=Block.TASK_FREE
):
    b = Block(id=id_, pose_status=pose, task_status=task)
    b.pose.position.x = float(x)
    b.pose.position.y = float(y)
    b.pose.position.z = float(z)
    qx, qy, qz, qw = yaw_to_quaternion(math.radians(yaw_deg))
    b.pose.orientation.x, b.pose.orientation.y = qx, qy
    b.pose.orientation.z, b.pose.orientation.w = qz, qw
    return b


def seibersdorf_storage(top_task=Block.TASK_FREE):
    """322's pile: two side by side along y, one crosswise on top (seed_seibersdorf)."""
    return [
        block("block_1", -11.0, 3.61, 0.3),
        block("block_2", -11.0, 2.99, 0.3),
        block("block_0", -11.0, 3.3, 0.9, yaw_deg=90.0, task=top_task),
    ]


def test_a_free_bound_block_is_kept():
    blocks = [block("block_0", -13.0), block("block_5", -14.0)]
    assert free_block_for_slot(blocks, "block_0", set(), (-14.0, 0.0)) == "block_0"


def test_a_block_in_the_jaws_is_never_rebound_away_from():
    # 327: rebound to block_5, the pick path opened the jaws at block_5's hover
    blocks = [block("block_0", -13.0, task=Block.TASK_MOVE), block("block_5", -14.0)]
    assert free_block_for_slot(blocks, "block_0", set(), (-14.0, 0.0)) == "block_0"


def test_a_goal_only_placeholder_gives_way_to_the_nearest_free_block():
    blocks = [
        block("block_0", 0.0, pose=Block.POSE_UNKNOWN),  # set_block_goal placeholder
        block("block_5", -11.0),
        block("block_6", -13.0),
        block("block_7", -14.0, task=Block.TASK_MOVE),  # in the jaws
        block("block_8", -15.0, task=Block.TASK_PLACED),
    ]
    assert free_block_for_slot(blocks, "block_0", set(), (-14.5, 0.0)) == "block_6"
    # bound to another slot: left for that slot
    assert (
        free_block_for_slot(blocks, "block_0", {"block_6"}, (-14.5, 0.0)) == "block_5"
    )


def test_a_coarse_block_is_not_picked():
    blocks = [
        block("block_0", 0.0, pose=Block.POSE_UNKNOWN),
        block("block_6", -14.0, pose=Block.POSE_COARSE),  # one face, half hidden
    ]
    assert free_block_for_slot(blocks, "block_0", set(), (-14.0, 0.0)) == "block_0"


def test_nothing_free_keeps_the_binding_so_the_server_waits():
    blocks = [block("block_0", 0.0, pose=Block.POSE_UNKNOWN)]
    assert free_block_for_slot(blocks, "block_0", set(), (-13.0, 0.0)) == "block_0"


def test_a_crosswise_top_block_covers_both_below_it():
    # yaw 90: the top block's long axis crosses theirs, and it still overlaps both footprints.
    # Side by side with a 20 mm gap, the two below cover nothing.
    assert covering_blocks(seibersdorf_storage()) == {
        "block_1": "block_0",
        "block_2": "block_0",
    }


def test_a_block_one_layer_up_but_beside_the_pile_covers_nothing():
    # same z as the top block, so only the footprints can tell these apart
    blocks = seibersdorf_storage()
    blocks[2].pose.position.x = -9.4  # 1.6 m along x: clear of both footprints
    assert covering_blocks(blocks) == {}
    blocks[
        2
    ].pose.position.x = -10.4  # 0.6 m: the crosswise footprint reaches block_1 again
    assert covering_blocks(blocks) == {"block_1": "block_0", "block_2": "block_0"}


def test_the_top_block_of_the_storage_goes_first():
    blocks = seibersdorf_storage()
    # wall_2_1's first slot is bound to a block pinned under the top one
    assert free_block_for_slot(blocks, "block_1", set(), (-15.0, -1.0)) == "block_0"
    assert free_block_for_slot(blocks, "block_0", set(), (-15.0, -1.0)) == "block_0"


def test_a_bottom_block_is_pickable_once_the_top_one_is_lifted():
    for task in (Block.TASK_MOVE, Block.TASK_PLACED):
        blocks = seibersdorf_storage(top_task=task)
        assert covering_blocks(blocks) == {}
        # block_2 is the nearer of the two to a target at y -1.0
        assert free_block_for_slot(blocks, "block_1", set(), (-15.0, -1.0)) == "block_1"
        assert free_block_for_slot(blocks, "x", set(), (-15.0, -1.0)) == "block_2"


def test_the_only_pickable_block_is_taken_from_the_slot_it_is_bound_to():
    # else a plan that binds the top block to its last slot never picks anything: every
    # earlier slot is pinned and the one block that can be picked is reserved.
    blocks = seibersdorf_storage()
    assert (
        free_block_for_slot(blocks, "block_1", {"block_0"}, (-15.0, -1.0)) == "block_0"
    )


def test_nothing_pickable_keeps_the_binding_and_names_the_coverer():
    # a coarse top block still covers, and cannot be picked itself: nothing is pickable
    blocks = seibersdorf_storage()
    blocks[2].pose_status = Block.POSE_COARSE
    assert free_block_for_slot(blocks, "block_1", set(), (-15.0, -1.0)) == "block_1"
    assert (
        covered_reason(covering_blocks(blocks)) == "block_1, block_2 covered by block_0"
    )
    assert covered_reason(covering_blocks(seibersdorf_storage(Block.TASK_MOVE))) == ""


class _StubServer:
    """What ``_rebind_to_a_free_block`` touches before it decides to wait."""

    _physical_block_id = WallPlanServer._physical_block_id
    _rebind_to_a_free_block = WallPlanServer._rebind_to_a_free_block

    def __init__(self, blocks, bindings):
        self._blocks = blocks
        self._block_bindings = {"wall_2_1": dict(bindings)}
        self._output_frame = "world"
        self._goal_client = self._clear_goals_client = None
        self._goal_service_name = self._clear_goals_service_name = ""

    def _get_world_model_blocks(self):
        return self._blocks

    def _make_pose(self, *args):
        return PoseStamped()

    def _call_service(self, *args):
        return None

    def get_logger(self):
        return self

    def info(self, *args):
        pass


def test_the_server_reports_the_reason_instead_of_the_pinned_block():
    blocks = seibersdorf_storage()
    blocks[2].pose_status = Block.POSE_COARSE  # covers, and is not pickable itself
    server = _StubServer(blocks, {"c0_b0": "block_1"})
    chosen, blocked = server._rebind_to_a_free_block(
        "wall_2_1", {"block_id": "c0_b0"}, (-15.0, -1.0, 0.3)
    )
    assert chosen == "block_1"
    assert blocked == "block_1, block_2 covered by block_0"


def test_a_slot_whose_block_was_taken_gets_the_takers_binding():
    # 327: c0_b0 took c0_b1's block_1; c0_b1 left on block_1 read as occupied once it was placed
    blocks = [block("block_0", 0.0, pose=Block.POSE_UNKNOWN), block("block_1", -14.0)]
    server = _StubServer(blocks, {"c0_b0": "block_0", "c0_b1": "block_1"})
    chosen, _ = server._rebind_to_a_free_block(
        "wall_2_1", {"block_id": "c0_b0", "yaw": 0.0}, (-15.0, -1.0, 0.3)
    )
    assert chosen == "block_1"
    assert server._block_bindings["wall_2_1"] == {
        "c0_b0": "block_1",
        "c0_b1": "block_0",
    }
