from glob import glob

from setuptools import find_packages, setup

package_name = "franzi_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phl",
    maintainer_email="puheliang122@gmail.com",
    description="Pick-and-place / machine-tending demo for the Franzi robot.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pick_place_task = franzi_pick_place.pick_place_node:main",
            "reach_map = franzi_pick_place.reach_map:main",
            "dock_tolerance = franzi_pick_place.dock_tolerance:main",
            "teach_docks = franzi_pick_place.teach_docks:main",
            "handeye_calibrate = franzi_pick_place.handeye:main",
        ],
    },
)
