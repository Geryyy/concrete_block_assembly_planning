"""Course 1 seats on the block below it, not on an absolute target (340, 356).

Absolute, the lower block's landing error went straight into the 10 mm stud clearance.
"""

from pathlib import Path

import pytest
import yaml
from concrete_block_assembly_planning.wall_plan_server import WallPlanServer

PLANS = Path(__file__).resolve().parents[1] / "config" / "wall_plans.yaml"


class _StubServer:
    """What ``_resolve_plan`` touches: it only logs, and only on an unusable plan item."""

    _resolve_plan = WallPlanServer._resolve_plan

    def get_logger(self):
        return self

    def error(self, *args):
        raise AssertionError(args)


def _tasks(plan_name):
    plans = yaml.safe_load(PLANS.read_text())["wall_plans"]
    resolved = _StubServer()._resolve_plan(plan_name, plans[plan_name])
    return {task["block_id"]: task for task in resolved}


@pytest.mark.parametrize(
    "plan, slot, below",
    [
        ("wall_2_1", "c1_b0", "c0_b0"),
        ("example_wall", "c1_b0", "c0_b0"),
        ("example_wall", "c1_b1", "c0_b1"),
    ],
)
def test_course_1_seats_on_the_block_it_rests_on(plan, slot, below):
    tasks = _tasks(plan)
    bridge = tasks[slot]
    assert bridge["target_mode"] == "relative_to_world_model"
    assert bridge["reference_block_id"] == below
    assert bridge["offset"] == [0.30, 0.0, 0.60]
    # The absolute point it used to carry is the fallback, and stays the plan's own geometry:
    # build direction and the slot bookkeeping read `position`, not the world model.
    ground = tasks[below]["position"]
    assert [round(ground[i] + bridge["offset"][i], 3) for i in range(3)] == bridge[
        "position"
    ]
