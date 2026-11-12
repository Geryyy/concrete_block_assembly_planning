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


def test_a_re_activated_plan_continues_after_the_placed_slots():
    # 327: activate zeroes progress; task 1 went to a free block and into the occupied slot
    placed = blocks(block_0=Block.TASK_PLACED, block_1=Block.TASK_PLACED)
    assert next_task_index(TASKS, placed, BINDINGS, 0, 1) == 2


def test_a_slot_with_a_placed_block_on_its_target_is_occupied():
    # a restarted server lost the rebinding: block_7 went into c0_b0's slot
    tasks = [dict(t, position=[-15.0 + i, -1.0, 0.3]) for i, t in enumerate(TASKS)]
    placed = blocks(block_7=Block.TASK_PLACED)
    placed[0].pose_status = Block.POSE_PRECISE
    placed[0].pose.position.x, placed[0].pose.position.y = -14.98, -1.02
    placed[0].pose.position.z = 0.32
    assert next_task_index(tasks, placed, BINDINGS, 0, None) == 1


def test_the_block_in_the_jaws_is_served_before_anything_else():
    # cancelled mid-carry of task 2, plan re-activated: resume task 2, not task 1
    in_jaws = blocks(block_1=Block.TASK_MOVE)
    assert next_task_index(TASKS, in_jaws, BINDINGS, 0, None) == 1
    assert next_task_index(TASKS, in_jaws, BINDINGS, 3, 2) == 1


def test_an_unbound_slot_falls_back_to_its_slot_id():
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_PLACED), {}, 1, 0) == 1
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_MOVE), {}, 1, 0) == 0
