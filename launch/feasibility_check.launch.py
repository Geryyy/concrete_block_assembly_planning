"""Feasibility check for staging (pickup) and wall (place) block poses.

Runs the pose feasibility checker: it calls the A2B ``CalcMovement`` service for
each block's key poses (pickup, place, and their pre-approach points) and prints
a PASS/FAIL table, then exits.

IMPORTANT: this runs the checker ONLY and talks to an already-running A2B server.
The a2b_ilqr_jerk_server is tightly coupled to the full crane bringup (it parses
the PZS100 robot/tool description and segfaults without it), so it cannot be
started standalone. Launch the sim ("Gazebo model with behavior tree (PZS100)")
or an MP bringup FIRST, leave it up, then run this. The checker reads the YAMLs
fresh each run, so tweak block positions and re-run in seconds without
relaunching the sim.

    ros2 launch concrete_block_assembly_planning feasibility_check.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    seed_default = PathJoinSubstitution(
        [FindPackageShare("concrete_block_world_model"), "config",
         "world_model_seed_pick_place.yaml"])
    wall_plans_default = PathJoinSubstitution(
        [FindPackageShare("concrete_block_assembly_planning"), "config", "wall_plans.yaml"])

    return LaunchDescription([
        DeclareLaunchArgument("wall_plan_name", default_value="example_wall"),
        DeclareLaunchArgument("seed_file", default_value=seed_default),
        DeclareLaunchArgument("wall_plans_file", default_value=wall_plans_default),
        DeclareLaunchArgument("a2b_service", default_value="/a2b_movement"),

        # init pose 2 = initialization_outside.yaml:
        #   q0[8] = theta1,theta2,theta3,q4,theta6,theta7,theta8,theta10
        Node(
            package="concrete_block_assembly_planning",
            executable="check_pose_feasibility",
            output="screen",
            parameters=[{
                "seed_file": LaunchConfiguration("seed_file"),
                "wall_plans_file": LaunchConfiguration("wall_plans_file"),
                "wall_plan_name": LaunchConfiguration("wall_plan_name"),
                "a2b_service": LaunchConfiguration("a2b_service"),
                # q0[7]=theta10 (gripper jaw) must be within the a2b limit
                # [0.40, 1.42]; the init-pose value 1.5708 is out of range, so use
                # a valid mid-range jaw angle (negligible effect on arm reach).
                "q0": [0.785, 0.523599, 0.523602, 0.25, 0.546470, 1.570521, 0.0, 1.0],
                "pickup_approach_height_m": 0.30,
                "place_approach_height_m": 0.30,
                "place_approach_angle_deg": 4.0,
                "world_to_k0_xyz": [-6.939, 0.350, 1.141],
                "world_to_k0_yaw_deg": 180.0,
            }],
        ),
    ])
