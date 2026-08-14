from glob import glob

from setuptools import find_packages, setup


package_name = "brov_mission"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description=(
        "Validate pool-frame waypoint drafts and resolve immutable missions "
        "into a localization odometry frame."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "mission_manager_node = brov_mission.mission_manager_node:main",
        ],
    },
)

