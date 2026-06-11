#!/usr/bin/env python3
"""Validate an expanded wall layout against crane reach and static obstacles.

Given the absolute per-block layout from :mod:`wall_expander` (world frame),
each block is checked for:

* **reach** — its center lies within the crane's horizontal annulus
  (``reach_min``..``reach_max`` around the base) and vertical range.
* **collision** — its (yaw-rotated, axis-aligned-bounded) box does not overlap
  any static scene object.

Pure Python (no rclpy) so it can be unit tested and called live while the user
drags the wall origin in RViz. Results drive the goal-ghost recoloring
(green = ok, red = unreachable / colliding).
"""

from __future__ import annotations

import math
import yaml

# Generous defaults; override per deployment via parameters.
DEFAULT_REACH = {
    "base_xy": [0.0, 0.0],   # crane slewing column in world (x, y)
    "reach_min": 2.0,
    "reach_max": 12.0,
    "z_min": -2.0,
    "z_max": 8.0,
}


def _rotated_aabb_half_extents(dims, yaw_deg: float):
    """World-axis half extents of a box with size ``dims`` rotated ``yaw`` about z."""
    hx, hy, hz = dims[0] / 2.0, dims[1] / 2.0, dims[2] / 2.0
    c, s = abs(math.cos(math.radians(yaw_deg))), abs(math.sin(math.radians(yaw_deg)))
    return (hx * c + hy * s, hx * s + hy * c, hz)


def _aabb_overlap(c1, h1, c2, h2) -> bool:
    """True if two axis-aligned boxes (center, half-extents) overlap on all axes."""
    return all(abs(c1[i] - c2[i]) < (h1[i] + h2[i]) for i in range(3))


def check_reach(block, reach: dict):
    """Return ``(ok, reason)`` for the crane-reach test of a single block."""
    bx, by = reach.get("base_xy", [0.0, 0.0])
    r = math.hypot(block["x"] - bx, block["y"] - by)
    if r < reach["reach_min"]:
        return False, f"inside min reach ({r:.2f} < {reach['reach_min']:.2f} m)"
    if r > reach["reach_max"]:
        return False, f"beyond max reach ({r:.2f} > {reach['reach_max']:.2f} m)"
    if block["z"] < reach["z_min"] or block["z"] > reach["z_max"]:
        return False, f"z {block['z']:.2f} outside [{reach['z_min']}, {reach['z_max']}]"
    return True, ""


def check_collisions(block, dims, static_objects):
    """Return a list of static object ids whose AABB overlaps the block."""
    c1 = (block["x"], block["y"], block["z"])
    h1 = _rotated_aabb_half_extents(dims, block.get("yaw_deg", 0.0))
    hits = []
    for obj in static_objects:
        c2 = tuple(obj["position"])
        d2 = obj["dimensions"]
        h2 = (d2[0] / 2.0, d2[1] / 2.0, d2[2] / 2.0)
        if _aabb_overlap(c1, h1, c2, h2):
            hits.append(obj["id"])
    return hits


def validate_layout(blocks, meta: dict, static_objects=None, reach: dict = None):
    """Validate every block. Returns ``(all_valid, results)``.

    ``results`` maps block id -> ``{reachable, collision_free, collisions,
    reasons, valid}``.
    """
    static_objects = static_objects or []
    reach = {**DEFAULT_REACH, **(reach or {})}
    dims = meta.get("block_size", [0.9, 0.6, 0.6])

    results = {}
    all_valid = True
    for b in blocks:
        reach_ok, reason = check_reach(b, reach)
        collisions = check_collisions(b, dims, static_objects)
        collision_free = not collisions
        valid = reach_ok and collision_free
        reasons = []
        if not reach_ok:
            reasons.append(reason)
        if collisions:
            reasons.append("collides with " + ", ".join(collisions))
        results[b["id"]] = {
            "reachable": reach_ok,
            "collision_free": collision_free,
            "collisions": collisions,
            "reasons": reasons,
            "valid": valid,
        }
        all_valid = all_valid and valid
    return all_valid, results


def load_static_scene_from_world_model_yaml(path: str):
    """Parse ``static_scene_objects`` (a nested YAML string) from a world_model config."""
    with open(path) as f:
        data = yaml.safe_load(f)
    params = data.get("world_model_node", {}).get("ros__parameters", {})
    raw = params.get("world_model", {}).get("static_scene_objects", "")
    if not raw:
        return []
    objects = yaml.safe_load(raw) or []
    # Keep only entries with both position and dimensions.
    return [o for o in objects if "position" in o and "dimensions" in o]
