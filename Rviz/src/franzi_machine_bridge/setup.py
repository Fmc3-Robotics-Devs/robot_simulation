from setuptools import find_packages, setup

package_name = "franzi_machine_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="phl",
    maintainer_email="puheliang122@gmail.com",
    description="Engraving machine bridge and safety interlocks.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "machine_bridge = franzi_machine_bridge.bridge_node:main",
        ],
    },
)
