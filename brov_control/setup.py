from glob import glob

from setuptools import find_packages, setup


package_name = "brov_control"


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
    ],
    install_requires=["setuptools"],
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description="RL and model-based controllers for the BlueROV2 runtime.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "policy_node = brov_control.policy_node:main",
            "policy_wrench_node = brov_control.policy_wrench_node:main",
            "policy_node_mk2 = brov_control.policy_node_mk2:main",
            (
                "model_based_controller_node = "
                "brov_control.model_based_controller_node:main"
            ),
            "drag_test_node = brov_control.drag_test_node:main",
            "dvl_record_node = brov_control.dvl_record_node:main",
        ],
    },
)
