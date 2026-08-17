from glob import glob

from setuptools import find_packages, setup


package_name = "brov_localization"


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
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest", "PyYAML"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description="One-shot pool-frame alignment for BROV local odometry.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "pool_alignment_node = brov_localization.localization_node:main",
        ],
    },
)
