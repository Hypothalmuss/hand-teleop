from setuptools import setup

package_name = "mujoco_sim_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robotic",
    maintainer_email="roboticai@eagleprojects.tn",
    description="MuJoCo simulation node (physics, cameras, reset service)",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mujoco_sim = mujoco_sim_ros.sim_node:main",
            "scripted_trajectory = mujoco_sim_ros.scripted_trajectory:main",
        ],
    },
)
