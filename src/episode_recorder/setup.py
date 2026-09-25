from setuptools import setup

package_name = "episode_recorder"

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
    description="Keyboard console and episode recorder",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "keyboard_console = episode_recorder.keyboard_console:main",
    ]},
)
