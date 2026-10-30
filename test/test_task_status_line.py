"""`/cbs/status/task` carries the served task line plus the poses it resolved."""

from concrete_block_assembly_planning.wall_plan_server import task_status


def test_the_line_carries_the_message_and_both_targets():
    line = task_status(
        "wall1",
        3,
        7,
        "c0_b0",
        "block_2",
        (4.1234, -1.5678, 0.2501),
        (6.0, 2.0, 1.05),
        (6.1736, 2.0),
        "Task 3/7: slot c0_b0 -> block_2",
    )
    assert line == (
        "plan=wall1 task=3/7 slot=c0_b0 block=block_2 "
        "pickup_x=4.123 pickup_y=-1.568 pickup_z=0.250 "
        "target_x=6.000 target_y=2.000 target_z=1.050 "
        "approach_x=6.174 approach_y=2.000 msg=Task 3/7: slot c0_b0 -> block_2"
    )
