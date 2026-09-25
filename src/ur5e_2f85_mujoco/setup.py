import os
from glob import glob

from setuptools import setup

package_name = "ur5e_2f85_mujoco"
share = os.path.join("share", package_name)


def tree(root):
    """data_files entries that reproduce ``root`` under the package share dir."""
    out = []
    for dirpath, _, files in os.walk(root):
        if files:
            out.append((os.path.join(share, dirpath), [os.path.join(dirpath, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (share, ["package.xml"]),
        (
            os.path.join(share, package_name),
            glob(package_name + "/*.xml") + glob(package_name + "/*.yaml"),
        ),
    ]
    + tree("assets"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotic",
    maintainer_email="roboticai@eagleprojects.tn",
    description="UR5e + 2F-85 MuJoCo scene and kinematics (no ROS deps)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": []},
)
