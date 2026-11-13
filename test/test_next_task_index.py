"""A handed-out task is served again until the world model reports its block placed."""

from concrete_block_assembly_planning.wall_plan_server import next_task_index
from concrete_block_world_model_interfaces.msg import Block

TASKS = [{"block_id": "c0_b0"}, {"block_id": "c0_b1"}, {"block_id": "c0_b2"}]
BINDINGS = {"c0_b0": "block_0", "c0_b1": "block_1", "c0_b2": "block_2"}


def blocks(**task_status):
    return [Block(id=id_, task_status=s) for id_, s in task_status.items()]


def test_a_cancelled_goal_gets_its_task_again():
    # fresh plan -> task 1; the hand-out leaves handed_out=0
    assert next_task_index(TASKS, [], BINDINGS, None) == 0
    # cancelled: block still in the jaws, or dropped back free -> task 1 again
    for status in (Block.TASK_MOVE, Block.TASK_FREE, Block.TASK_UNKNOWN):
        assert next_task_index(TASKS, blocks(block_0=status), BINDINGS, 0) == 0
    # placed -> task 2
    assert next_task_index(TASKS, blocks(block_0=Block.TASK_PLACED), BINDINGS, 0) == 1


def test_the_last_task_placed_runs_the_plan_off_the_end_so_it_reports_complete():
    placed = blocks(block_2=Block.TASK_PLACED)
    assert next_task_index(TASKS, placed, BINDINGS, 2) == len(TASKS)
    assert next_task_index(TASKS, [], BINDINGS, 2) == 2


def test_a_re_activated_plan_continues_after_the_placed_slots():
    # 327: activate drops the hand-out; task 1 went to a free block and into the occupied slot
    placed = blocks(block_0=Block.TASK_PLACED, block_1=Block.TASK_PLACED)
    assert next_task_index(TASKS, placed, BINDINGS, None) == 2


def test_a_slot_with_a_placed_block_on_its_target_is_occupied():
    # a restarted server lost the rebinding: block_7 went into c0_b0's slot
    tasks = [dict(t, position=[-15.0 + i, -1.0, 0.3]) for i, t in enumerate(TASKS)]
    placed = blocks(block_7=Block.TASK_PLACED)
    placed[0].pose_status = Block.POSE_PRECISE
    placed[0].pose.position.x, placed[0].pose.position.y = -14.98, -1.02
    placed[0].pose.position.z = 0.32
    assert next_task_index(tasks, placed, BINDINGS, None) == 1


def wall_blocks(tasks, count, err=(0.12, -0.09, 0.15)):
    """The first ``count`` slots filled, as a restarted world model reports them: rediscovered
    from scratch, so TASK_FREE and POSE_PRECISE, not TASK_PLACED. ``err`` is a placement plus
    re-fit error inside the half-block window (readiness A3 reads ground blocks up to 0.25 m
    high), so the test is not the zero-error case."""
    out = []
    for task, physical in zip(tasks[:count], (BINDINGS[t["block_id"]] for t in tasks)):
        b = Block(
            id=physical, task_status=Block.TASK_FREE, pose_status=Block.POSE_PRECISE
        )
        x, y, z = task["position"]
        b.pose.position.x = x + err[0]
        b.pose.position.y = y + err[1]
        b.pose.position.z = z + err[2]
        out.append(b)
    return out


def test_a_restart_does_not_re_serve_a_slot_that_already_holds_a_block():
    # 21: no TASK_PLACED survives a stack restart, so only the geometry says the slot is full.
    # Before this, slot 1 was served again and the nearest "pickable" block to its target was
    # the one standing in it.
    tasks = [dict(t, position=[-15.0 + i, -1.0, 0.3]) for i, t in enumerate(TASKS)]
    assert next_task_index(tasks, wall_blocks(tasks, 3), BINDINGS, None) == len(tasks)
    assert next_task_index(tasks, wall_blocks(tasks, 2), BINDINGS, None) == 2


def test_the_block_in_the_jaws_is_served_before_anything_else():
    # cancelled mid-carry of task 2, plan re-activated: resume task 2, not task 1
    in_jaws = blocks(block_1=Block.TASK_MOVE)
    assert next_task_index(TASKS, in_jaws, BINDINGS, None) == 1
    assert next_task_index(TASKS, in_jaws, BINDINGS, 2) == 1


def test_an_unbound_slot_falls_back_to_its_slot_id():
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_PLACED), {}, 0) == 1
    assert next_task_index(TASKS, blocks(c0_b0=Block.TASK_MOVE), {}, 0) == 0
