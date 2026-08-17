from glob import glob

from setuptools import find_packages, setup


package_name = "brov_bringup"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description="Launch composition for BlueROV2 sim-to-real experiments.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "demo_orchestrator_node = "
            "brov_bringup.demo_orchestrator_node:main",
        ],
    },
)
