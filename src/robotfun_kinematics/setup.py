import os
from glob import glob

from setuptools import find_packages, setup

package_name = "robotfun_kinematics"

setup(
    name=package_name,
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yholi",
    maintainer_email="yholicomun@gmail.com",
    description="Cinemática directa/inversa (DH estándar, 5 juntas + gripper; "
                "modos joint_4 bloqueado/activo) del brazo RobotFun.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ik_node = robotfun_kinematics.nodes.ik_node:main",
            "fk_check_node = robotfun_kinematics.nodes.fk_check_node:main",
            "trajectory_node = robotfun_kinematics.nodes.trajectory_node:main",
            "joint_state_relay = robotfun_kinematics.nodes.joint_state_relay:main",
        ],
    },
)
