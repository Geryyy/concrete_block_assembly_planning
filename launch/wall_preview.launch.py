"""Expand a wall spec, save the full plan, and show it in RViz — no interaction.

Edit the spec, run this, look at RViz. The full per-block plan is written to
``output_plan_file`` and the absolute path is logged by the node.

    ros2 launch concrete_block_assembly_planning wall_preview.launch.py

A static ``world -> base`` TF is published so RViz has a fixed frame to anchor to.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("concrete_block_assembly_planning")

    default_spec = PathJoinSubstitution([pkg, "config", "wall_spec_example.yaml"])
    default_output_plan = PathJoinSubstitution([pkg, "config", "wall_plans.yaml"])
    default_rviz = PathJoinSubstitution([pkg, "rviz", "wall_preview.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spec_file",
                default_value=default_spec,
                description="Compact wall spec to expand (edit this to change the wall).",
            ),
            DeclareLaunchArgument(
                "wall_plan_name",
                default_value="example_wall",
                description="Name the expanded plan is stored under.",
            ),
            DeclareLaunchArgument(
                "output_plan_file",
                default_value=default_output_plan,
                description="Where the full per-block plan is written.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="world",
                description="Frame the markers are published in.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz with the wall-preview view.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for the wall-preview view.",
            ),
            # Give RViz a 'world' frame to anchor to (node broadcasts no TF).
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="world_to_base",
                arguments=[
                    "--frame-id",
                    LaunchConfiguration("world_frame"),
                    "--child-frame-id",
                    "base",
                ],
                output="screen",
            ),
            Node(
                package="concrete_block_assembly_planning",
                executable="wall_preview_node",
                name="wall_preview_node",
                parameters=[
                    {
                        "spec_file": LaunchConfiguration("spec_file"),
                        "wall_plan_name": LaunchConfiguration("wall_plan_name"),
                        "output_plan_file": LaunchConfiguration("output_plan_file"),
                        "world_frame": LaunchConfiguration("world_frame"),
                    }
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", LaunchConfiguration("rviz_config")],
                condition=IfCondition(LaunchConfiguration("rviz")),
                output="screen",
            ),
        ]
    )
