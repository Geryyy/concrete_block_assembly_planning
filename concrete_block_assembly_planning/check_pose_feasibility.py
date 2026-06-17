#!/usr/bin/env python3
"""Check feasibility of staging (pickup) and wall (place) block poses.

Reads the world-model staging seed and the wall plan, builds each block's key
poses (block + pre-approach), converts them world -> K0_mounting_base, and asks
the A2B ``CalcMovement`` service whether the crane can reach each one
collision-free (same IK + collision gate the behavior tree hits). Prints a
PASS/FAIL table. Standalone (no Gazebo): needs only the a2b server +
collision_body_handler running. One-shot — exits after the report.
"""

import math
import sys

import yaml

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from timber_crane_planning_interfaces.srv import CalcMovement


def _normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


def _parse_seed_blocks(path):
    """Return {id: (x, y, z, yaw_deg)} from a world_model seed YAML."""
    with open(path) as f:
        top = yaml.safe_load(f) or {}
    raw = (
        top.get("world_model_node", {})
        .get("ros__parameters", {})
        .get("world_model", {})
        .get("initial_blocks", "")
    )
    blocks = yaml.safe_load(raw) if raw else []
    out = {}
    for b in blocks or []:
        p = b["position"]
        out[b["id"]] = (float(p[0]), float(p[1]), float(p[2]), float(b.get("yaw_deg", 0.0)))
    return out


def _parse_wall_plan(path, plan_name):
    """Return ordered list of (id, x, y, z, yaw_deg, gripper_yaw_offset_deg).

    Only absolute_position entries are resolved (the expander emits all-absolute).
    """
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    plans = data.get("wall_plans", {})
    plan = plans.get(plan_name) or plans.get(plan_name.lower())
    if plan is None:
        return []
    seq = []
    for item in plan.get("sequence", []):
        if "absolute_position" not in item:
            continue  # checker only handles absolute plans
        p = item["absolute_position"]
        seq.append((
            item["id"], float(p[0]), float(p[1]), float(p[2]),
            float(item.get("yaw_deg", 0.0)),
            float(item.get("gripper_yaw_offset_deg", 0.0)),
        ))
    return seq


class FeasibilityChecker(Node):
    def __init__(self):
        super().__init__("pose_feasibility_checker")

        self.declare_parameter("seed_file", "")
        self.declare_parameter("wall_plans_file", "")
        self.declare_parameter("wall_plan_name", "example_wall")
        self.declare_parameter("a2b_service", "/a2b_movement")
        # Start joint config (init pose). q0[8] = theta1,theta2,theta3,q4,theta6,theta7,theta8,theta10
        # q0[8] = theta1,theta2,theta3,q4,theta6,theta7,theta8,theta10. theta10
        # (gripper jaw) must be within the a2b limit [0.40, 1.42] — the init-pose
        # value 1.5708 is out of range, so default to a valid mid-range angle.
        self.declare_parameter(
            "q0", [0.785, 0.523599, 0.523602, 0.25, 0.546470, 1.570521, 0.0, 1.0])
        self.declare_parameter("pickup_approach_height_m", 0.30)
        self.declare_parameter("place_approach_height_m", 0.30)
        self.declare_parameter("place_approach_angle_deg", 4.0)
        # world -> K0 transform: K0 origin in world (xyz) and yaw (deg).
        self.declare_parameter("world_to_k0_xyz", [-6.939, 0.350, 1.141])
        self.declare_parameter("world_to_k0_yaw_deg", 180.0)

        self._seed_file = self.get_parameter("seed_file").value
        self._wall_plans_file = self.get_parameter("wall_plans_file").value
        self._plan_name = self.get_parameter("wall_plan_name").value
        self._q0 = [float(v) for v in self.get_parameter("q0").value]
        self._pickup_h = self.get_parameter("pickup_approach_height_m").value
        self._place_h = self.get_parameter("place_approach_height_m").value
        self._place_ang = math.radians(self.get_parameter("place_approach_angle_deg").value)
        self._k0_off = list(self.get_parameter("world_to_k0_xyz").value)
        self._k0_yaw = math.radians(self.get_parameter("world_to_k0_yaw_deg").value)

        svc = self.get_parameter("a2b_service").value
        self._cli = self.create_client(CalcMovement, svc)
        if not self._cli.wait_for_service(timeout_sec=30.0):
            self.get_logger().error(
                f"A2B service '{svc}' unavailable. The a2b server runs inside the "
                "sim/MP bringup (it can't run standalone). Launch 'Gazebo model with "
                "behavior tree (PZS100)', leave it up, then run this check.")
            return

        self._run()

    # ---- world -> K0 ------------------------------------------------------

    def _to_k0(self, x, y, z):
        ox, oy, oz = self._k0_off
        ca, sa = math.cos(self._k0_yaw), math.sin(self._k0_yaw)
        dx, dy, dz = x - ox, y - oy, z - oz
        # Rz(-yaw) applied (R^T of K0-in-world) — yaw=180 gives (-dx,-dy)
        return (ca * dx + sa * dy, -sa * dx + ca * dy, dz)

    def _phi_k0(self, tcp_yaw_world):
        return _normalize(tcp_yaw_world - self._k0_yaw)

    # ---- build the pose list ---------------------------------------------

    def _build(self):
        """Return [(label, x_w, y_w, z_w, tcp_yaw_w)] for all checked poses."""
        plan = _parse_wall_plan(self._wall_plans_file, self._plan_name)
        offsets = {b[0]: b[5] for b in plan}        # id -> gripper_yaw_offset_deg
        seed = _parse_seed_blocks(self._seed_file)
        poses = []

        # Pickup side: staging block + pre-pick (straight above).
        for bid, (x, y, z, yaw) in seed.items():
            tcp = math.radians(yaw + offsets.get(bid, 90.0))
            poses.append((f"{bid} pickup_approach", x, y, z + self._pickup_h, tcp))
            poses.append((f"{bid} pickup", x, y, z, tcp))

        # Place side: wall target + pre-place (above + lateral along build dir).
        prev = None
        lateral = self._place_h * math.tan(self._place_ang)
        for (bid, x, y, z, yaw, goff) in plan:
            tcp = math.radians(yaw + goff)
            bx, by = 0.0, 0.0
            if prev is not None and abs(z - prev[2]) < 1e-3:
                dx, dy = x - prev[0], y - prev[1]
                n = math.hypot(dx, dy)
                if n > 1e-6:
                    bx, by = dx / n, dy / n
            poses.append((f"{bid} place_approach", x + lateral * bx, y + lateral * by,
                          z + self._place_h, tcp))
            poses.append((f"{bid} place", x, y, z, tcp))
            prev = (x, y, z)
        return poses

    # ---- service call -----------------------------------------------------

    def _feasible(self, xk, yk, zk, phik):
        req = CalcMovement.Request()
        req.y_n = Point(x=float(xk), y=float(yk), z=float(zk))
        req.phi_tool_n = float(phik)
        req.t_end = 0.0
        req.carries_log = False
        req.slow_down = 1.0
        req.q0 = self._q0
        req.publish_path = False
        req.check_log_collision = False
        req.check_gripper_collision = True
        future = self._cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        res = future.result()
        return (res.success if res is not None else False)

    def _run(self):
        poses = self._build()
        self.get_logger().info(
            f"Checking {len(poses)} poses (plan='{self._plan_name}') via A2B...")
        n_pass = 0
        print(f"\n{'pose':<26}{'world (x,y,z)':<26}{'K0 (x,y,z)':<26}{'result'}")
        print("-" * 90)

        # CONTROL: the crane's own start TCP (~init pose 2 FK), phi = q0[0]-q0[6].
        # This is a near-zero-motion target and MUST be feasible; if it FAILs the
        # checker/request is broken (not the block positions).
        cphi = self._q0[0] - self._q0[6]
        cok = self._feasible(4.08, 4.06, 2.30, cphi)
        print(f"{'CONTROL (start TCP)':<26}"
              f"{'(reachable)':<26}"
              f"({4.08:+.2f},{4.06:+.2f},{2.30:+.2f})       "
              f"{'PASS' if cok else 'FAIL <-- checker/request broken!'}")
        print("-" * 90)

        for (label, xw, yw, zw, tcp) in poses:
            xk, yk, zk = self._to_k0(xw, yw, zw)
            ok = self._feasible(xk, yk, zk, self._phi_k0(tcp))
            n_pass += int(ok)
            print(f"{label:<26}"
                  f"({xw:+.2f},{yw:+.2f},{zw:+.2f})       "
                  f"({xk:+.2f},{yk:+.2f},{zk:+.2f})       "
                  f"{'PASS' if ok else 'FAIL'}")
        print("-" * 90)
        print(f"{n_pass}/{len(poses)} feasible\n")


def main():
    rclpy.init()
    node = FeasibilityChecker()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


if __name__ == "__main__":
    main()
