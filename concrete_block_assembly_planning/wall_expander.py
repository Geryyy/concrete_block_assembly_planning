#!/usr/bin/env python3
"""Expand a compact wall specification into a full per-block wall plan.

The compact spec describes a wall parametrically (block size, clearances,
origin, yaw, and a list of courses with a block count and a running-bond
offset). :func:`compute_layout` turns that into absolute per-block poses in the
world frame plus a support graph; :func:`to_wall_plan` re-expresses the same
blocks in the existing ``wall_plans.yaml`` schema (origin-relative chaining so
placement error does not accumulate across the wall).

This module is pure Python (no rclpy) so it can be unit tested directly.

Compact spec schema (YAML)::

    wall:
      block_size: [0.9, 0.6, 0.6]   # [X, Y, Z] in metres; X=depth, Z=height
      length_axis: x                # which block axis runs along the wall
      clearance:
        horizontal: 0.02            # gap between blocks within a course
        vertical: 0.0               # mortar gap between courses
      gripper_yaw_offset_deg: 90.0  # optional, copied onto every block
      origin: {x: 0.0, y: 0.0, z: 0.0, yaw_deg: 0.0}
      origin_frame: world
      courses:
        - {count: 3, offset: 0.0}
        - {count: 2, offset: 0.45}  # half-block running-bond offset
"""

from __future__ import annotations

import argparse
import math
from typing import Optional

import yaml

DEFAULT_BLOCK_SIZE = [0.9, 0.6, 0.6]


def _wall_section(spec: dict) -> dict:
    """Accept either a top-level ``wall:`` block or a bare wall dict."""
    return spec["wall"] if isinstance(spec, dict) and "wall" in spec else spec


def _block_length(block_size, length_axis: str) -> float:
    axis = {"x": 0, "y": 1, "z": 2}.get(length_axis.lower(), 0)
    return float(block_size[axis])


def compute_layout(spec: dict):
    """Return ``(blocks, meta)`` with absolute world poses and a support graph.

    Each block is a dict with: ``id, course, index, along, x, y, z, yaw_deg,
    supports`` (list of block ids in the course below that it rests on).
    """
    wall = _wall_section(spec)
    block_size = wall.get("block_size", DEFAULT_BLOCK_SIZE)
    length_axis = wall.get("length_axis", "x")
    block_len = _block_length(block_size, length_axis)
    block_h = float(block_size[2])

    clearance = wall.get("clearance", {}) or {}
    h_clear = float(clearance.get("horizontal", 0.0))
    v_clear = float(clearance.get("vertical", 0.0))

    origin = wall.get("origin", {}) or {}
    ox = float(origin.get("x", 0.0))
    oy = float(origin.get("y", 0.0))
    oz = float(origin.get("z", 0.0))
    yaw_deg = float(origin.get("yaw_deg", 0.0))
    yaw = math.radians(yaw_deg)
    dir_x, dir_y = math.cos(yaw), math.sin(yaw)

    pitch = block_len + h_clear
    vpitch = block_h + v_clear

    courses = wall.get("courses", []) or []
    blocks = []
    for i, course in enumerate(courses):
        count = int(course["count"])
        offset = float(course.get("offset", 0.0))
        for j in range(count):
            along = offset + j * pitch
            blocks.append({
                "id": f"c{i}_b{j}",
                "course": i,
                "index": j,
                "along": along,
                "x": ox + along * dir_x,
                "y": oy + along * dir_y,
                "z": oz + i * vpitch,
                "yaw_deg": yaw_deg,
                "supports": [],
            })

    _assign_supports(blocks, block_len)

    meta = {
        "block_size": list(block_size),
        "block_len": block_len,
        "block_h": block_h,
        "pitch": pitch,
        "vpitch": vpitch,
        "yaw_deg": yaw_deg,
        "dir": [dir_x, dir_y, 0.0],
        "origin_frame": wall.get("origin_frame", "world"),
        "gripper_yaw_offset_deg": wall.get("gripper_yaw_offset_deg"),
    }
    return blocks, meta


def _assign_supports(blocks, block_len: float) -> None:
    """Populate each block's ``supports`` from along-axis overlap with the course below."""
    by_course: dict[int, list] = {}
    for b in blocks:
        by_course.setdefault(b["course"], []).append(b)

    half = block_len / 2.0
    for i in sorted(by_course):
        if i == 0:
            continue
        below = by_course.get(i - 1, [])
        for b in by_course[i]:
            lo, hi = b["along"] - half, b["along"] + half
            for lb in below:
                llo, lhi = lb["along"] - half, lb["along"] + half
                if min(hi, lhi) - max(lo, llo) > 1e-6:
                    b["supports"].append(lb["id"])


def build_support_graph(blocks) -> dict:
    """Return ``{block_id: [supporting block ids]}``."""
    return {b["id"]: list(b["supports"]) for b in blocks}


def topo_order(blocks):
    """Return blocks in a build order that respects supports (bottom-up).

    Blocks are already produced in (course, index) order, which is a valid
    topological order because a block only depends on blocks in the course
    below. This is a thin validation wrapper that raises on any cycle / missing
    support reference rather than silently reordering.
    """
    placed: set[str] = set()
    order = []
    for b in blocks:
        for s in b["supports"]:
            if s not in placed:
                raise ValueError(
                    f"Block '{b['id']}' would be placed before its support '{s}'"
                )
        order.append(b)
        placed.add(b["id"])
    return order


def to_wall_plan(blocks, meta: dict) -> list:
    """Re-express blocks as a ``wall_plans.yaml`` sequence with relative chaining.

    - first block of the wall: ``absolute_position``
    - first block of each higher course: ``relative_to`` the first block below
    - every other block: ``relative_to`` the previous block in its course

    Offsets are world-axis deltas (matching the wall plan server's resolver,
    which adds offsets componentwise).
    """
    gripper_yaw = meta.get("gripper_yaw_offset_deg")
    seq = []
    prev_in_course: dict[int, dict] = {}
    first_of_course: dict[int, dict] = {}

    for b in blocks:
        i, j = b["course"], b["index"]
        item = {"id": b["id"], "yaw_deg": b["yaw_deg"]}
        if gripper_yaw is not None:
            item["gripper_yaw_offset_deg"] = gripper_yaw

        if i == 0 and j == 0:
            item["absolute_position"] = [round(v, 6) for v in (b["x"], b["y"], b["z"])]
        else:
            ref = prev_in_course[i] if j > 0 else first_of_course[i - 1]
            item["relative_to"] = ref["id"]
            item["offset"] = [
                round(b["x"] - ref["x"], 6),
                round(b["y"] - ref["y"], 6),
                round(b["z"] - ref["z"], 6),
            ]

        seq.append(item)
        prev_in_course[i] = b
        if j == 0:
            first_of_course[i] = b

    return seq


def expand(spec: dict, name: Optional[str] = None) -> dict:
    """Expand a compact spec into a wall_plans.yaml-compatible structure."""
    blocks, meta = compute_layout(spec)
    topo_order(blocks)  # validate support ordering
    plan_name = name or _wall_section(spec).get("name", "wall")
    return {
        "defaults": {"block_size": meta["block_size"]},
        "wall_plans": {
            plan_name: {"sequence": to_wall_plan(blocks, meta)},
        },
    }


def load_spec(path: str) -> dict:
    with open(path) as f:
        return yaml.safe_load(f)


def main():
    parser = argparse.ArgumentParser(description="Expand a compact wall spec.")
    parser.add_argument("spec", help="Path to the compact wall spec YAML")
    parser.add_argument("-o", "--output", help="Write expanded plan here (default: stdout)")
    parser.add_argument("-n", "--name", help="Wall plan name (default: spec 'name' or 'wall')")
    args = parser.parse_args()

    spec = load_spec(args.spec)
    plan = expand(spec, name=args.name)
    text = yaml.safe_dump(plan, sort_keys=False, default_flow_style=None)
    if args.output:
        with open(args.output, "w") as f:
            f.write(text)
    else:
        print(text)


if __name__ == "__main__":
    main()
