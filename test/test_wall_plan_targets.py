"""wall_2_1's bridge seats on the block below it, not on an absolute target (340).

Absolute, c0_b0's landing error went straight into the 10 mm stud clearance of the bridge.
"""

from pathlib import Path

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


def test_the_bridge_seats_on_the_block_it_rests_on():
    tasks = _tasks("wall_2_1")
    bridge = tasks["c1_b0"]
    assert bridge["target_mode"] == "relative_to_world_model"
    assert bridge["reference_block_id"] == "c0_b0"
    assert bridge["offset"] == [0.30, 0.0, 0.60]
    # The absolute point it used to carry is the fallback, and stays the plan's own geometry:
    # build direction and the slot bookkeeping read `position`, not the world model.
    assert bridge["position"] == [-14.7, -1.0, 0.9]
    ground = tasks["c0_b0"]["position"]
    assert [round(ground[i] + bridge["offset"][i], 3) for i in range(3)] == bridge[
        "position"
    ]
