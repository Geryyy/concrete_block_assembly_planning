#!/usr/bin/env python3
"""Wall plan server.

Serves GetNextAssemblyTask. Reads wall_plans.yaml for target (place)
positions and the plan sequence. Queries the world model for actual block
pickup poses at runtime; falls back to YAML positions if the world model is
unavailable.

All poses are emitted in the configured ``output_frame`` (default ``world``).
The grip trajectory server handles the block-CoG -> TCP offset downstream.
"""

import math
import threading

import yaml

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ament_index_python.packages import get_package_share_directory

from concrete_block_assembly_interfaces.srv import GetNextAssemblyTask
from concrete_block_world_model_interfaces.msg import Block
from concrete_block_world_model_interfaces.srv import GetCoarseBlocks, SetBlockGoal
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger


def yaw_to_quaternion(yaw_rad: float):
    """Convert yaw angle to quaternion (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


def quaternion_to_yaw(q) -> float:
    """Extract yaw from quaternion (geometry_msgs/Quaternion)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle_rad: float) -> float:
    """Normalize angle to [-pi, pi]."""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


class WallPlanServer(Node):
    def __init__(self):
        super().__init__("wall_plan_server")

        # NOTE: No Z offset applied here. The wall plan sends raw block CoG
        # positions. The grip trajectory server handles the full offset from
        # block center to K8_tool_center_point via its tcp_z_offset parameter.

        self.declare_parameter(
            "world_model_service",
            "/world_model_node/get_coarse_blocks",
        )
        self.declare_parameter("world_model_timeout_s", 2.0)
        # Frame all emitted poses are expressed in. The whole stack standardizes
        # on `world`; conversion to K0_mounting_base happens at the MP boundary.
        self.declare_parameter("output_frame", "world")
        # Which plan's targets to publish into the world model as block goals
        # (empty = the only loaded plan). Makes the world model the unified
        # source of both actual and goal poses.
        self.declare_parameter("goal_plan_name", "")
        self.declare_parameter(
            "world_model_set_goal_service", "/world_model_node/set_block_goal"
        )
        # Pre-place approach: a point this far above the target, offset laterally
        # along the wall build direction so the descent comes in at this angle
        # (clearance from the already-placed neighbour).
        self.declare_parameter("place_approach_height_m", 0.30)
        self.declare_parameter("place_approach_angle_deg", 4.0)
        # Pre-pick point: this far straight above the pickup pose.
        self.declare_parameter("pickup_approach_height_m", 0.30)

        self._wm_service_name = self.get_parameter("world_model_service").value
        self._wm_timeout = self.get_parameter("world_model_timeout_s").value
        self._output_frame = self.get_parameter("output_frame").value
        self._approach_height = self.get_parameter("place_approach_height_m").value
        self._approach_angle = math.radians(
            self.get_parameter("place_approach_angle_deg").value
        )
        self._pickup_approach_height = self.get_parameter("pickup_approach_height_m").value
        self._cb_group = ReentrantCallbackGroup()

        # World model client (for live block poses)
        self._wm_client = self.create_client(
            GetCoarseBlocks,
            self._wm_service_name,
            callback_group=self._cb_group,
        )

        # Load wall plans
        self.declare_parameter("wall_plans_file", "")
        self._wall_plan_path = self.get_parameter("wall_plans_file").value
        if not self._wall_plan_path:
            pkg_share = get_package_share_directory("concrete_block_assembly_planning")
            self._wall_plan_path = f"{pkg_share}/config/wall_plans.yaml"

        self._wall_plans = {}
        self._progress = {}
        self._load_wall_plans()

        # Services
        self.create_service(
            GetNextAssemblyTask,
            "~/get_next_assembly_task",
            self._handle_get_next_task,
            callback_group=self._cb_group,
        )
        # Reload plans from disk (e.g. after the wall setup node persists a new
        # plan) without restarting the server. Progress is reset.
        self.create_service(
            Trigger,
            "~/reload_plans",
            self._handle_reload,
            callback_group=self._cb_group,
        )
        # Publish the plan's block goals into the world model (unified scene).
        self._goal_plan_name = self.get_parameter("goal_plan_name").value
        self._goal_service_name = self.get_parameter("world_model_set_goal_service").value
        self._goal_client = self.create_client(
            SetBlockGoal, self._goal_service_name, callback_group=self._cb_group
        )
        self._goal_timer = self.create_timer(
            1.0, self._try_publish_goals, callback_group=self._cb_group
        )

        self.get_logger().info(
            f"Wall plan server ready (output_frame={self._output_frame})"
        )

    def _load_wall_plans(self):
        """(Re)load wall plans from ``wall_plans_file``. Returns the plan count."""
        self.get_logger().info(f"Loading wall plans from {self._wall_plan_path}")
        # Re-arm goal publishing so a reload re-pushes goals to the world model.
        self._goals_published = False
        with open(self._wall_plan_path) as f:
            data = yaml.safe_load(f) or {}

        self._wall_plans = {}
        self._progress = {}
        for plan_name, plan_data in data.get("wall_plans", {}).items():
            tasks = self._resolve_plan(plan_name, plan_data)
            self._wall_plans[plan_name] = tasks
            self._progress[plan_name] = 0
            self.get_logger().info(
                f"Loaded wall plan '{plan_name}' with {len(tasks)} tasks"
            )
        return len(self._wall_plans)

    def _handle_reload(self, request, response):
        try:
            count = self._load_wall_plans()
            response.success = True
            response.message = f"Reloaded {count} wall plan(s) from {self._wall_plan_path}"
        except (OSError, yaml.YAMLError) as ex:
            response.success = False
            response.message = f"Reload failed: {ex}"
        self.get_logger().info(response.message)
        return response

    # ---- world-model goal publishing -------------------------------------

    def _resolve_goal_plan_name(self):
        """Which loaded plan's targets to publish as goals, or '' if ambiguous."""
        name = self._goal_plan_name
        if name:
            if name in self._wall_plans:
                return name
            if name.lower() in self._wall_plans:
                return name.lower()
            return ""
        if len(self._wall_plans) == 1:
            return next(iter(self._wall_plans))
        return ""

    def _try_publish_goals(self):
        """Push the plan's resolved targets into the world model as block goals."""
        if self._goals_published:
            return
        plan_name = self._resolve_goal_plan_name()
        if not plan_name:
            self._goals_published = True
            self.get_logger().warn(
                "goal_plan_name unset and not exactly one plan loaded; "
                "not publishing block goals to the world model"
            )
            return
        if not self._goal_client.service_is_ready():
            return  # world model not up yet; retry on the next tick

        tasks = self._wall_plans.get(plan_name, [])
        for task in tasks:
            req = SetBlockGoal.Request()
            req.block_id = task["block_id"]
            x, y, z = task["position"]
            req.goal_pose = self._make_pose(x, y, z, task["yaw"]).pose
            req.frame_id = self._output_frame
            self._goal_client.call_async(req)
        self._goals_published = True
        self.get_logger().info(
            f"Published {len(tasks)} block goal(s) for plan '{plan_name}' "
            f"to {self._goal_service_name}"
        )

    def _resolve_plan(self, plan_name, plan_data):
        """Resolve plan sequence into task dictionaries.

        Supported target definitions:
        - absolute_position: fixed target in the output frame
        - relative_to: target relative to a prior task in the same plan
        - relative_to_world_model: target relative to a live world-model block
        """
        resolved = {}
        tasks = []
        prev_pos = None
        for idx, item in enumerate(plan_data.get("sequence", [])):
            block_id = item["id"]
            yaw_rad = math.radians(item.get("yaw_deg", 0.0))
            gripper_yaw_offset = math.radians(item.get("gripper_yaw_offset_deg", 0.0))
            reference_block_id = item.get("relative_to_world_model", "")
            offset = item.get("offset", [0, 0, 0])
            target_mode = "absolute"

            if "absolute_position" in item:
                pos = item["absolute_position"]
            elif "relative_to" in item:
                ref = item["relative_to"]
                if ref not in resolved:
                    self.get_logger().error(
                        f"Plan '{plan_name}': block '{block_id}' references "
                        f"unknown block '{ref}'"
                    )
                    continue
                ref_pos = resolved[ref]
                offset = item.get("offset", [0, 0, 0])
                pos = [ref_pos[i] + offset[i] for i in range(3)]
                reference_block_id = ref
            elif "relative_to_world_model" in item:
                target_mode = "relative_to_world_model"
                pos = item.get("fallback_absolute_position")
                if pos is None:
                    self.get_logger().error(
                        f"Plan '{plan_name}': block '{block_id}' uses "
                        "'relative_to_world_model' but has no "
                        "'fallback_absolute_position'"
                    )
                    continue
            else:
                self.get_logger().error(
                    f"Plan '{plan_name}': block '{block_id}' has no position"
                )
                continue

            resolved[block_id] = pos
            task_id = f"{plan_name}_{idx + 1:02d}_{block_id}"
            # Build direction: from the previous block in the same course (same z)
            # toward this one — i.e. away from the just-placed neighbour. None at
            # course starts (no same-course neighbour placed yet -> straight down).
            build_dir = None
            if prev_pos is not None and abs(pos[2] - prev_pos[2]) < 1e-3:
                dx, dy = pos[0] - prev_pos[0], pos[1] - prev_pos[1]
                norm = math.hypot(dx, dy)
                if norm > 1e-6:
                    build_dir = [dx / norm, dy / norm]
            prev_pos = pos
            tasks.append({
                "task_id": task_id,
                "block_id": block_id,
                "position": pos,
                "yaw": yaw_rad,
                "gripper_yaw_offset": gripper_yaw_offset,
                "target_mode": target_mode,
                "reference_block_id": reference_block_id,
                "offset": offset,
                "build_dir": build_dir,
            })
        return tasks

    def _get_world_model_blocks(self):
        """Return the current world-model blocks list, or None on failure."""
        if not self._wm_client.service_is_ready():
            self.get_logger().warn(
                f"World model service '{self._wm_service_name}' not available, "
                "using YAML fallback"
            )
            return None

        req = GetCoarseBlocks.Request()
        req.force_refresh = False
        req.timeout_s = float(self._wm_timeout)
        future = self._wm_client.call_async(req)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())

        if not done.wait(timeout=float(self._wm_timeout) + 1.0) or future.result() is None:
            self.get_logger().warn("World model query timed out, using YAML fallback")
            return None

        result = future.result()
        if not result.success:
            self.get_logger().warn(
                f"World model query failed: {result.message}, using YAML fallback"
            )
            return None

        return result.blocks.blocks

    def _query_block_pose(self, block_id: str):
        """Query world model for a block's actual pose. Returns (x, y, z, yaw) or None."""
        blocks = self._get_world_model_blocks()
        if blocks is None:
            return None

        for block in blocks:
            if block.id == block_id:
                p = block.pose.position
                yaw = quaternion_to_yaw(block.pose.orientation)
                self.get_logger().info(
                    f"World model pose for '{block_id}': "
                    f"({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) yaw={math.degrees(yaw):.1f}deg"
                )
                return (p.x, p.y, p.z, yaw)

        self.get_logger().warn(
            f"Block '{block_id}' not found in world model, using YAML fallback"
        )
        return None

    def _query_block_goal(self, block_id: str):
        """Query world model for a block's goal pose. Returns (x, y, z, yaw) or None."""
        blocks = self._get_world_model_blocks()
        if blocks is None:
            return None

        for block in blocks:
            if block.id == block_id and block.goal_status == Block.GOAL_SET:
                p = block.goal_pose.position
                yaw = quaternion_to_yaw(block.goal_pose.orientation)
                return (p.x, p.y, p.z, yaw)
        return None

    def _make_pose(self, x, y, z, yaw_rad, frame=None):
        pose = PoseStamped()
        pose.header.frame_id = frame if frame is not None else self._output_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        qx, qy, qz, qw = yaw_to_quaternion(yaw_rad)
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        return pose

    def _handle_get_next_task(self, request, response):
        plan_name = request.wall_plan_name.lower()

        if request.reset_plan:
            self._progress[plan_name] = 0
            self.get_logger().info(f"Reset progress for plan '{plan_name}'")

        if plan_name not in self._wall_plans:
            response.success = False
            response.has_task = False
            response.message = f"Unknown wall plan: '{plan_name}'"
            self.get_logger().error(response.message)
            return response

        tasks = self._wall_plans[plan_name]
        idx = self._progress.get(plan_name, 0)

        if idx >= len(tasks):
            response.success = True
            response.has_task = False
            response.message = f"Plan '{plan_name}' complete ({len(tasks)} tasks done)"
            self.get_logger().info(response.message)
            return response

        task = tasks[idx]
        task_id = task["task_id"]
        block_id = task["block_id"]
        x, y, z = task["position"]
        block_target_yaw = task["yaw"]
        gripper_yaw_offset = task.get("gripper_yaw_offset", 0.0)
        reference_block_id = task.get("reference_block_id", "")
        self._progress[plan_name] = idx + 1

        # Query world model for actual pickup pose; fall back to YAML
        wm_pose = self._query_block_pose(block_id)
        if wm_pose is not None:
            px, py, pz, pickup_block_yaw = wm_pose
            pose_source = "world_model"
        else:
            px, py, pz, pickup_block_yaw = x, y, z, block_target_yaw
            pose_source = "yaml"

        pickup_tool_yaw = normalize_angle(pickup_block_yaw + gripper_yaw_offset)

        if task.get("target_mode") == "relative_to_world_model":
            ref_pose = self._query_block_pose(reference_block_id)
            if ref_pose is not None:
                rx, ry, rz, ryaw = ref_pose
                offset = task.get("offset", [0, 0, 0])
                x = rx + offset[0]
                y = ry + offset[1]
                z = rz + offset[2]
                block_target_yaw = normalize_angle(ryaw + task["yaw"])
                target_source = f"world_model:{reference_block_id}"
            else:
                target_source = "fallback_yaml"
        else:
            # Place pose comes from the world model goal (unified scene); the
            # plan's resolved YAML position is the fallback if no goal is set.
            goal_pose = self._query_block_goal(block_id)
            if goal_pose is not None:
                x, y, z, block_target_yaw = goal_pose
                target_source = "world_model_goal"
            else:
                target_source = "yaml"

        target_tool_yaw = normalize_angle(block_target_yaw + gripper_yaw_offset)

        # Pre-place point: above the target, offset laterally along the build
        # direction so the descent approaches at place_approach_angle_deg.
        lateral = self._approach_height * math.tan(self._approach_angle)
        build_dir = task.get("build_dir")
        ax = x + lateral * build_dir[0] if build_dir else x
        ay = y + lateral * build_dir[1] if build_dir else y
        az = z + self._approach_height

        response.success = True
        response.has_task = True
        response.task_id = task_id
        response.target_block_id = block_id
        response.reference_block_id = reference_block_id
        # All poses are raw block CoG — grip server handles TCP offset
        # Pose orientation carries the desired TCP yaw for the BT motion nodes.
        response.pickup_pose = self._make_pose(px, py, pz, pickup_tool_yaw)
        response.pickup_approach_pose = self._make_pose(
            px, py, pz + self._pickup_approach_height, pickup_tool_yaw)
        response.target_pose = self._make_pose(x, y, z, target_tool_yaw)
        response.reference_pose = self._make_pose(x, y, z, block_target_yaw)
        response.approach_pose = self._make_pose(ax, ay, az, target_tool_yaw)
        response.message = f"Task {idx + 1}/{len(tasks)}: {block_id}"

        self.get_logger().info(
            f"GetNextAssemblyTask | plan={plan_name} task={task_id} "
            f"block={block_id} pickup=({px:.2f}, {py:.2f}, {pz:.2f}) [{pose_source}] "
            f"target=({x:.2f}, {y:.2f}, {z:.2f}) [{target_source}] "
            f"block_yaw={math.degrees(block_target_yaw):.1f}deg "
            f"pickup_tcp_yaw={math.degrees(pickup_tool_yaw):.1f}deg "
            f"target_tcp_yaw={math.degrees(target_tool_yaw):.1f}deg"
        )
        return response


def main():
    rclpy.init()
    node = WallPlanServer()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
