from glob import glob

from setuptools import find_packages, setup

package_name = "concrete_block_assembly_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vscode",
    maintainer_email="gerald.ebmer.fl@ait.ac.at",
    description="Assembly/task planning layer for the concrete-block stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wall_plan_server = concrete_block_assembly_planning.wall_plan_server:main",
            "wall_expander = concrete_block_assembly_planning.wall_expander:main",
            "wall_setup_node = concrete_block_assembly_planning.wall_setup_node:main",
            "wall_preview_node = concrete_block_assembly_planning.wall_preview_node:main",
            "check_pose_feasibility = concrete_block_assembly_planning.check_pose_feasibility:main",
        ],
    },
)
