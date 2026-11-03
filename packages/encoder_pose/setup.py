import os
from glob import glob

from setuptools import setup

package_name = "encoder_pose"

setup(
    name=package_name,
    version="2.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config", "encoder_pose_node"), glob("config/encoder_pose_node/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Vincenzo",
    maintainer_email="vincenzo.polizzi@student.ethz.ch",
    description="Pose estimation (dead reckoning) from the wheel encoders.",
    license="GPLv3",
    entry_points={
        "console_scripts": [
            "encoder_pose_node = encoder_pose.encoder_pose_node:main",
        ],
    },
)
