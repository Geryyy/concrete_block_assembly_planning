"""Interactive wall setup + RViz visualization.

Brings up the goal-wall ghost (recolored live by reach/collision validity) with a
draggable origin InteractiveMarker, plus an RViz view preconfigured to show it.

The setup node publishes everything in the ``world`` frame but broadcasts no TF,
so a static ``world -> base`` transform is published here to give RViz a fixed
frame to anchor to.

Typical use::

    ros2 launch concrete_block_assembly_planning wall_setup.launch.py

Drag the blue origin handle in RViz; the wall re-expands and recolors live.
Freeze + persist the plan via the ``/wall_setup_node/confirm_wall_origin`` service.
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
    default_rviz = PathJoinSubstitution([pkg, "rviz", "wall_setup.rviz"])
    default_static_scene = PathJoinSubstitution(
        [FindPackageShare("concrete_block_world_model"), "config", "world_model.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "spec_file",
                default_value=default_spec,
                description="Compact wall spec to expand.",
            ),
            DeclareLaunchArgument(
                "wall_plan_name",
                default_value="example_wall",
                description="Name the expanded plan is persisted under.",
            ),
            DeclareLaunchArgument(
                "output_plan_file",
                default_value=default_output_plan,
                description="Where ConfirmWallOrigin(persist=true) writes the plan.",
            ),
            DeclareLaunchArgument(
                "static_scene_file",
                default_value=default_static_scene,
                description="world_model.yaml whose objects are used for collision validation.",
            ),
            DeclareLaunchArgument(
                "world_frame",
                default_value="world",
                description="Frame all markers/blocks are published in.",
            ),
            DeclareLaunchArgument(
                "rviz",
                default_value="true",
                description="Launch RViz with the wall-setup view.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz config for the wall-setup view.",
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
                executable="wall_setup_node",
                name="wall_setup_node",
                parameters=[
                    {
                        "spec_file": LaunchConfiguration("spec_file"),
                        "wall_plan_name": LaunchConfiguration("wall_plan_name"),
                        "output_plan_file": LaunchConfiguration("output_plan_file"),
                        "static_scene_file": LaunchConfiguration("static_scene_file"),
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
