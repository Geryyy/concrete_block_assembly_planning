"""A slot binds to the nearest free discovered block when its bound one is not free."""

from concrete_block_world_model_interfaces.msg import Block

from concrete_block_assembly_planning.wall_plan_server import free_block_for_slot


def block(id_, x, pose=Block.POSE_PRECISE, task=Block.TASK_FREE):
    b = Block(id=id_, pose_status=pose, task_status=task)
    b.pose.position.x = float(x)
    return b


def test_a_free_bound_block_is_kept():
    blocks = [block("block_0", -13.0), block("block_5", -14.0)]
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
