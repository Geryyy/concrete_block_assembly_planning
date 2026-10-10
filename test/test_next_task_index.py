"""A handed-out task is served again until the world model reports its block placed."""

from concrete_block_assembly_planning.wall_plan_server import next_task_index
from concrete_block_world_model_interfaces.msg import Block

TASKS = [{"block_id": "c0_b0"}, {"block_id": "c0_b1"}, {"block_id": "c0_b2"}]
BINDINGS = {"c0_b0": "block_0", "c0_b1": "block_1", "c0_b2": "block_2"}


def blocks(**task_status):
    return [Block(id=id_, task_status=s) for id_, s in task_status.items()]


def test_a_cancelled_goal_gets_its_task_again():
    # fresh plan -> task 1; the hand-out leaves progress=1, handed_out=0
    assert next_task_index(TASKS, [], BINDINGS, 0, None) == 0
    # cancelled: block still in the jaws, or dropped back free -> task 1 again
    for status in (Block.TASK_MOVE, Block.TASK_FREE, Block.TASK_UNKNOWN):
        assert next_task_index(TASKS, blocks(block_0=status), BINDINGS, 1, 0) == 0
    # placed -> task 2
    assert (
        next_task_index(TASKS, blocks(block_0=Block.TASK_PLACED), BINDINGS, 1, 0) == 1
    )


def test_the_last_task_placed_runs_the_plan_off_the_end_so_it_reports_complete():
    placed = blocks(block_2=Block.TASK_PLACED)
    assert next_task_index(TASKS, placed, BINDINGS, 3, 2) == len(TASKS)
    assert next_task_index(TASKS, [], BINDINGS, 3, 2) == 2


def test_reset_plan_restarts_at_task_one_even_with_blocks_already_placed():
    # reset_plan / activate zero progress only; the stale hand-out must not pull it back
    placed = blocks(block_0=Block.TASK_PLACED, block_1=Block.TASK_PLACED)
    assert next_task_index(TASKS, placed, BINDINGS, 0, 1) == 0


def test_an_unbound_slot_falls_back_to_its_slot_id():
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_PLACED), {}, 1, 0) == 1
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_MOVE), {}, 1, 0) == 0
