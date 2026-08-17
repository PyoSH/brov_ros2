from glob import glob

from setuptools import find_packages, setup


package_name = "cmg_deploy"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description=(
        "Standalone hover RL policy integrated with brov_ros2 through "
        "topics only."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "cmg_policy_node = cmg_deploy.cmg_policy_node:main",
        ],
    },
)
