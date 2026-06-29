# concrete_block_assembly_planning

Wall **task planning** for the concrete-block stack: expand a high-level wall specification into an ordered, per-block build plan, let the operator set and freeze the wall origin interactively in RViz, and serve the next block to place to the behavior tree. Everything works in the `world` frame; there is no PDDL — the plan is a deterministic expansion of `wall_plans.yaml`.

## Responsibilities

- **Expand** a wall spec (courses × blocks) into concrete per-block pickup/target poses (`wall_expander`, `wall_validator`).
- **Set the origin** interactively (ghost preview) and **freeze** it into a named plan (`wall_setup_node`).
- **Serve the build order** to the BT, one task at a time, and mirror each block's target into the world model (`wall_plan_server`).
- **Pre-flight check** reachability of the planned poses without Gazebo (`check_pose_feasibility`).

## Contents

```text
concrete_block_assembly_planning/
  wall_plan_server.py        Serves get_next_assembly_task; pushes goals to the world model
  wall_setup_node.py         Interactive origin marker; serves confirm_wall_origin
  wall_preview_node.py       Publishes the expanded wall as ghost markers
  wall_expander.py           Wall spec -> per-block plan (wall_plans.yaml schema)
  wall_validator.py          Reach / collision validation of resolved blocks
  check_pose_feasibility.py  Standalone A2B reachability PASS/FAIL table
config/   wall_plans.yaml, wall_spec_example.yaml, wall_plan_server.yaml
launch/   wall_setup, wall_preview, feasibility_check
rviz/     wall_setup.rviz, wall_preview.rviz
test/     test_wall_expander.py, test_wall_validator.py
```

## ROS interface

| Name | Type | Direction | Node |
|---|---|---|---|
| `~/get_next_assembly_task` | `concrete_block_assembly_interfaces/GetNextAssemblyTask` | **server** | `wall_plan_server` |
| `~/reload_plans` | `std_srvs/Trigger` | server | `wall_plan_server` |
| `/world_model_node/set_block_goal` | `concrete_block_world_model_interfaces/SetBlockGoal` | **client** | `wall_plan_server` |
| `~/confirm_wall_origin` | `concrete_block_assembly_interfaces/ConfirmWallOrigin` | **server** | `wall_setup_node` |
| `~/goal_wall_markers`, `~/goal_wall_blocks` | `MarkerArray`, `BlockArray` | publisher | `wall_setup_node`, `wall_preview_node` |
| `/a2b_movement` | `timber_crane_planning_interfaces` (A2B) | **client** | `check_pose_feasibility` |

## Dependencies & interactions

| Direction | Package | Via |
|---|---|---|
| **out** | [concrete_block_assembly_interfaces](../concrete_block_assembly_interfaces/) | serves `GetNextAssemblyTask`, `ConfirmWallOrigin` |
| **out** | [concrete_block_world_model_interfaces](../concrete_block_world_model_interfaces/) | client of `SetBlockGoal` (mirrors targets into the unified scene) |
| **out** | `timber_crane_planning_interfaces` | `check_pose_feasibility` calls the timber A2B server |
| **consumed by** | [concrete_block_behavior_tree](../concrete_block_behavior_tree/) | the `GetNextAssemblyTask` BT node drives the `wall_assembly.xml` loop |
| **viz** | `interactive_markers`, `visualization_msgs`, `rviz2`, `tf2_ros` | origin marker + ghost preview |

Note: this package **plans tasks**, it does not plan motion — trajectory generation is [concrete_block_motion_planning](../concrete_block_motion_planning/) (grip) and the timber A2B server (long moves).

## Build

```bash
colcon build --packages-select concrete_block_assembly_planning --symlink-install
source install/setup.bash
ros2 launch concrete_block_assembly_planning wall_setup.launch.py
```
