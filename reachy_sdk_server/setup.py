import os
from glob import glob

from setuptools import find_packages, setup

package_name = "reachy_sdk_server"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="pierre",
    maintainer_email="pierre.rouanet@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "reachy_grpc_joint_sdk_server = reachy_sdk_server.grpc_server.joint_server:main",
            "reachy_grpc_audio_sdk_server = reachy_sdk_server.grpc_server.audio_server:main",
        ],
    },
)
