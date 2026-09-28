from setuptools import setup

package_name = "teleop_console"

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
    description="Keyboard console: e-stop, clear, scene reset",
    license="MIT",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "keyboard_console = teleop_console.keyboard_console:main",
    ]},
)
