#!/usr/bin/env python3
"""Expand a wall spec, save the full plan, and show it in RViz — no interaction.

Simple one-shot workflow:

1. read the compact spec (``spec_file``),
2. expand it into a full per-block plan and write it to ``output_plan_file``
   (merging into any plans already there, so siblings are preserved),
3. publish the blocks as a latched ``MarkerArray`` so RViz shows the wall.

To change the wall: edit the spec file and re-run. The absolute path the full
plan was written to is logged on startup. Everything is in the ``world`` frame.
"""

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from concrete_block_assembly_planning import wall_expander

BLOCK_COLOR = ColorRGBA(r=0.4, g=0.6, b=0.9, a=0.6)
LABEL_COLOR = ColorRGBA(r=1.0, g=1.0, b=1.0, a=0.9)


def yaw_to_quat(yaw_rad: float):
    """Return (x, y, z, w) for a yaw rotation about +z."""
    return (0.0, 0.0, math.sin(yaw_rad / 2.0), math.cos(yaw_rad / 2.0))


class WallPreviewNode(Node):
    def __init__(self):
        super().__init__("wall_preview_node")

        self.declare_parameter("spec_file", "")
        self.declare_parameter("wall_plan_name", "wall")
        self.declare_parameter("output_plan_file", "")
        self.declare_parameter("world_frame", "world")

        spec_file = self.get_parameter("spec_file").value
        if not spec_file:
            raise RuntimeError("wall_preview_node requires the 'spec_file' parameter")
        self._frame = self.get_parameter("world_frame").value
        plan_name = self.get_parameter("wall_plan_name").value
        out_file = self.get_parameter("output_plan_file").value

        spec = wall_expander.load_spec(spec_file)
        blocks, meta = wall_expander.compute_layout(spec)

        if out_file:
            self._write_plan(spec, plan_name, out_file)
        else:
            self.get_logger().warn("No 'output_plan_file' set — plan not saved, preview only")

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self._pub = self.create_publisher(MarkerArray, "~/goal_wall_markers", latched)
        self._pub.publish(self._make_markers(blocks, meta))
        self.get_logger().info(
            f"Showing {len(blocks)} blocks for plan '{plan_name}' (spec: {spec_file})"
        )

    def _write_plan(self, spec, plan_name, out_file):
        generated = wall_expander.expand(spec, name=plan_name)
        merged = {"defaults": generated["defaults"], "wall_plans": {}}
        try:
            with open(out_file) as f:
                existing = yaml.safe_load(f) or {}
            merged["defaults"] = existing.get("defaults", merged["defaults"])
            merged["wall_plans"].update(existing.get("wall_plans", {}))
        except FileNotFoundError:
            pass
        merged["wall_plans"].update(generated["wall_plans"])
        with open(out_file, "w") as f:
            yaml.safe_dump(merged, f, sort_keys=False, default_flow_style=None)
        n = len(generated["wall_plans"][plan_name]["sequence"])
        self.get_logger().info(
            f"Wrote full wall plan '{plan_name}' ({n} blocks) to {os.path.realpath(out_file)}"
        )

    def _make_markers(self, blocks, meta):
        dims = meta["block_size"]
        ma = MarkerArray()
        for i, b in enumerate(blocks):
            qx, qy, qz, qw = yaw_to_quat(math.radians(b["yaw_deg"]))

            m = Marker()
            m.header.frame_id = self._frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.ns = "goal_wall"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position = Point(x=b["x"], y=b["y"], z=b["z"])
            m.pose.orientation.x, m.pose.orientation.y = qx, qy
            m.pose.orientation.z, m.pose.orientation.w = qz, qw
            m.scale.x, m.scale.y, m.scale.z = dims[0], dims[1], dims[2]
            m.color = BLOCK_COLOR
            ma.markers.append(m)

            label = Marker()
            label.header = m.header
            label.ns = "goal_wall_ids"
            label.id = i
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = Point(x=b["x"], y=b["y"], z=b["z"] + 0.45)
            label.scale.z = 0.15
            label.color = LABEL_COLOR
            label.text = b["id"]
            ma.markers.append(label)
        return ma


def main():
    rclpy.init()
    node = WallPreviewNode()
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
