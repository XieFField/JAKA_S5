from glob import glob
from setuptools import find_packages, setup


package_name = "massage_mujoco"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/scenarios", glob("scenarios/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xieffield",
    maintainer_email="3432674139@qq.com",
    description="MuJoCo simulation runtime for the JAKA S5 massage robot.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_smoke = massage_mujoco.cli:main",
            "mujoco_node = massage_mujoco.ros_node:main",
            "mujoco_regression = massage_mujoco.regression:main",
        ],
    },
)
