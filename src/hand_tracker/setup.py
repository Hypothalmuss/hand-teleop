from setuptools import setup

package_name = "hand_tracker"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    package_data={package_name: ["models/hand_landmarker.task"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="robotic",
    maintainer_email="roboticai@eagleprojects.tn",
    description="MediaPipe hand tracking node",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["hand_tracker = hand_tracker.node:main"]},
)
