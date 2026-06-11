#!/usr/bin/env python3
"""Interactive wall setup: place the origin, see the goal wall, freeze it.

This node owns the *compact* wall spec and the wall origin. It:

* publishes the goal wall as a latched ``MarkerArray`` ghost (recolored live by
  reach/collision validity) and as a latched ``BlockArray`` (data, for the
  compare node / other consumers),
* exposes an RViz ``InteractiveMarker`` for the origin (planar x, y, yaw); each
  move re-expands + re-validates + re-publishes,
* serves ``ConfirmWallOrigin`` to bake the origin into the plan, optionally
  persist the expanded ``wall_plans.yaml``, and lock the origin for the run.

Everything is in the ``world`` frame (conversion to K0_mounting_base happens at
the motion-planning boundary, not here).
"""

import math

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
    MarkerArray,
)
from interactive_markers import InteractiveMarkerServer

from concrete_block_world_model_interfaces.msg import Block, BlockArray
from concrete_block_assembly_interfaces.srv import ConfirmWallOrigin

from concrete_block_assembly_planning import wall_expander, wall_validator

# Goal-ghost color convention (kept distinct from live world-model blocks).
COLOR_VALID = ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.30)    # reachable & clear
COLOR_INVALID = ColorRGBA(r=0.9, g=0.1, b=0.1, a=0.45)  # unreachable / colliding


def yaw_to_quat(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))


def quat_to_yaw(q: Quaternion) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class WallSetupNode(Node):
    def __init__(self):
        super().__init__("wall_setup_node")

        self.declare_parameter("spec_file", "")
        self.declare_parameter("wall_plan_name", "wall")
        self.declare_parameter("world_frame", "world")
        self.declare_parameter("output_plan_file", "")
        self.declare_parameter("static_scene_file", "")
        # Crane reach annulus (world), for live validation feedback.
        self.declare_parameter("reach_base_xy", [0.0, 0.0])
        self.declare_parameter("reach_min", 2.0)
        self.declare_parameter("reach_max", 12.0)
        self.declare_parameter("reach_z_min", -2.0)
        self.declare_parameter("reach_z_max", 8.0)

        self._frame = self.get_parameter("world_frame").value
        self._plan_name = self.get_parameter("wall_plan_name").value
        self._output_plan_file = self.get_parameter("output_plan_file").value
        self._locked = False
        self._prev_marker_count = 0

        spec_file = self.get_parameter("spec_file").value
        if not spec_file:
            raise RuntimeError("wall_setup_node requires the 'spec_file' parameter")
        self._spec = wall_expander.load_spec(spec_file)

        self._reach = {
            "base_xy": list(self.get_parameter("reach_base_xy").value),
            "reach_min": self.get_parameter("reach_min").value,
            "reach_max": self.get_parameter("reach_max").value,
            "z_min": self.get_parameter("reach_z_min").value,
            "z_max": self.get_parameter("reach_z_max").value,
        }

        static_scene_file = self.get_parameter("static_scene_file").value
        self._static_objects = []
        if static_scene_file:
            try:
                self._static_objects = \
                    wall_validator.load_static_scene_from_world_model_yaml(static_scene_file)
                self.get_logger().info(
                    f"Loaded {len(self._static_objects)} static scene objects for validation"
                )
            except (OSError, yaml.YAMLError) as ex:
                self.get_logger().warn(f"Could not load static scene: {ex}")

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._marker_pub = self.create_publisher(MarkerArray, "~/goal_wall_markers", latched)
        self._blocks_pub = self.create_publisher(BlockArray, "~/goal_wall_blocks", latched)

        self.create_service(
            ConfirmWallOrigin, "~/confirm_wall_origin", self._handle_confirm
        )

        self._im_server = InteractiveMarkerServer(self, "wall_origin")
        self._add_origin_marker()

        self._refresh()
        self.get_logger().info("Wall setup node ready — drag the origin in RViz.")

    # ---- origin marker -----------------------------------------------------

    def _origin_pose(self) -> Pose:
        origin = wall_expander._wall_section(self._spec).get("origin", {}) or {}
        pose = Pose()
        pose.position.x = float(origin.get("x", 0.0))
        pose.position.y = float(origin.get("y", 0.0))
        pose.position.z = float(origin.get("z", 0.0))
        pose.orientation = yaw_to_quat(math.radians(float(origin.get("yaw_deg", 0.0))))
        return pose

    def _add_origin_marker(self):
        im = InteractiveMarker()
        im.header.frame_id = self._frame
        im.name = "wall_origin"
        im.description = "Wall origin (x, y, yaw)"
        im.scale = 1.0
        im.pose = self._origin_pose()

        # Visible handle.
        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = 0.3
        box.scale.z = 0.05
        box.color = ColorRGBA(r=0.2, g=0.6, b=1.0, a=0.8)
        handle = InteractiveMarkerControl()
        handle.always_visible = True
        handle.markers.append(box)
        im.controls.append(handle)

        # Planar translation (x, y) and yaw rotation about z.
        move = InteractiveMarkerControl()
        move.name = "move_xy"
        move.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)  # plane normal +z
        move.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        im.controls.append(move)

        rotate = InteractiveMarkerControl()
        rotate.name = "rotate_z"
        rotate.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        rotate.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(rotate)

        self._im_server.insert(im, feedback_callback=self._on_marker_feedback)
        self._im_server.applyChanges()

    def _on_marker_feedback(self, feedback):
        if self._locked:
            return
        if feedback.event_type != feedback.POSE_UPDATE:
            return
        self._set_origin_from_pose(feedback.pose)
        self._refresh()

    def _set_origin_from_pose(self, pose: Pose):
        wall = wall_expander._wall_section(self._spec)
        wall.setdefault("origin", {})
        wall["origin"]["x"] = float(pose.position.x)
        wall["origin"]["y"] = float(pose.position.y)
        wall["origin"]["z"] = float(pose.position.z)
        wall["origin"]["yaw_deg"] = math.degrees(quat_to_yaw(pose.orientation))

    # ---- expansion + validation + publishing -------------------------------

    def _refresh(self):
        blocks, meta = wall_expander.compute_layout(self._spec)
        all_valid, results = wall_validator.validate_layout(
            blocks, meta, self._static_objects, self._reach
        )
        self._publish_markers(blocks, meta, results)
        self._publish_blocks(blocks)
        return blocks, meta, all_valid, results

    def _publish_markers(self, blocks, meta, results):
        dims = meta["block_size"]
        ma = MarkerArray()
        for i, b in enumerate(blocks):
            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "goal_wall"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(x=b["x"], y=b["y"], z=b["z"])
            m.pose.orientation = yaw_to_quat(math.radians(b["yaw_deg"]))
            m.scale.x, m.scale.y, m.scale.z = dims[0], dims[1], dims[2]
            m.color = COLOR_VALID if results[b["id"]]["valid"] else COLOR_INVALID
            ma.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = "goal_wall_ids"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=b["x"], y=b["y"], z=b["z"] + 0.45)
            label.scale.z = 0.15
            label.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)
            label.text = b["id"]
            ma.markers.append(label)

        # Clear leftovers from a previously larger wall.
        for i in range(len(blocks), self._prev_marker_count):
            for ns in ("goal_wall", "goal_wall_ids"):
                d = Marker()
                d.ns = ns
                d.id = i
                d.action = Marker.DELETE
                ma.markers.append(d)
        self._prev_marker_count = len(blocks)
        self._marker_pub.publish(ma)

    def _publish_blocks(self, blocks):
        arr = BlockArray()
        arr.header.frame_id = self._frame
        arr.header.stamp = self.get_clock().now().to_msg()
        for b in blocks:
            blk = Block()
            blk.id = b["id"]
            blk.pose.position = Point(x=b["x"], y=b["y"], z=b["z"])
            blk.pose.orientation = yaw_to_quat(math.radians(b["yaw_deg"]))
            blk.pose_status = Block.POSE_UNKNOWN
            blk.task_status = Block.TASK_FREE
            blk.confidence = 1.0
            arr.blocks.append(blk)
        self._blocks_pub.publish(arr)

    # ---- confirm service ---------------------------------------------------

    def _handle_confirm(self, request, response):
        # Origin may be provided explicitly; otherwise keep the current one.
        if any((request.origin.position.x, request.origin.position.y,
                request.origin.position.z)) or (
                request.origin.orientation.w not in (0.0, 1.0) or
                request.origin.orientation.z != 0.0):
            self._set_origin_from_pose(request.origin)

        plan_name = request.wall_plan_name or self._plan_name
        blocks, meta, all_valid, _ = self._refresh()

        message = f"Origin confirmed: {len(blocks)} blocks, valid={all_valid}"
        if request.persist:
            ok, msg = self._persist_plan(plan_name)
            message += f"; {msg}"
            if not ok:
                response.success = False
                response.valid = all_valid
                response.num_blocks = len(blocks)
                response.message = message
                return response

        self._locked = True
        self.get_logger().info(message + " (origin locked)")
        response.success = True
        response.valid = all_valid
        response.num_blocks = len(blocks)
        response.message = message
        return response

    def _persist_plan(self, plan_name: str):
        if not self._output_plan_file:
            return False, "no output_plan_file configured"
        try:
            generated = wall_expander.expand(self._spec, name=plan_name)
            # Merge into any existing file so sibling plans are preserved.
            merged = {"defaults": generated["defaults"], "wall_plans": {}}
            try:
                with open(self._output_plan_file) as f:
                    existing = yaml.safe_load(f) or {}
                merged["defaults"] = existing.get("defaults", merged["defaults"])
                merged["wall_plans"].update(existing.get("wall_plans", {}))
            except FileNotFoundError:
                pass
            merged["wall_plans"].update(generated["wall_plans"])
            with open(self._output_plan_file, "w") as f:
                yaml.safe_dump(merged, f, sort_keys=False, default_flow_style=None)
            return True, f"plan '{plan_name}' written to {self._output_plan_file}"
        except (OSError, yaml.YAMLError, ValueError) as ex:
            return False, f"persist failed: {ex}"


def main():
    rclpy.init()
    node = WallSetupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
