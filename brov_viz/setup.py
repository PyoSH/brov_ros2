from glob import glob

from setuptools import find_packages, setup


package_name = "brov_viz"


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
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description="Pool, raw vision, and aligned odometry visualization for BlueROV2.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "pool_scene_node = brov_viz.pool_scene_node:main",
        ],
    },
)
