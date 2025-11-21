from os.path import join
from glob import glob
from setuptools import find_packages, setup

package_name = "minigun_controller2"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (join("share", package_name, "launch"), glob("launch/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ubuntu",
    maintainer_email="ubuntu@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "scan_move = minigun_controller2.scan_move:main",
            "depth_move = minigun_controller2.depth_move:main",
        ],
    },
)
