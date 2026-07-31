from pathlib import Path

from setuptools import find_packages, setup

package_name = "franzi_skills"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [str(path) for path in Path("launch").glob("*.launch.py")],
        ),
        (
            f"share/{package_name}/config",
            [str(path) for path in Path("config").glob("*.yaml")],
        ),
        (
            f"share/{package_name}/maps",
            [str(path) for path in Path("maps").glob("*.*")],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phl",
    maintainer_email="puheliang122@gmail.com",
    description="The engraving cell's skills as ROS 2 action servers.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "skill_server = franzi_skills.skill_server:main",
            "camera_recorder = franzi_skills.camera_recorder:main",
            "camera_stamp_relay = franzi_skills.camera_stamp_relay:main",
            "mapping_drive = franzi_skills.mapping_drive:main",
        ],
    },
)
