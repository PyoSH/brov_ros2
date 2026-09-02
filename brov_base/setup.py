from glob import glob

from setuptools import find_packages, setup


package_name = "brov_base"


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
        (
            f"share/{package_name}/config",
            glob("config/*.yaml") + ["brov_base/vendor/brov2_heavy.yaml"],
        ),
    ],
    package_data={"brov_base.vendor": ["brov2_heavy.yaml", "t200_table.npz"]},
    install_requires=["PyYAML", "numpy", "pymavlink", "setuptools", "torch"],
    # ``extras_require`` remains visible to colcon with modern setuptools;
    # colcon uses the ``test`` extra to select pytest instead of unittest.
    extras_require={"test": ["pytest"]},
    zip_safe=False,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description=(
        "BlueROV2 MAVLink, observation, guidance, and actuator integration "
        "entry points."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "obs_node = brov_base.obs_node:main",
            "base_node = brov_base.base_node:main",
            "guidance_node = brov_base.guidance_node:main",
            "observation_node = brov_base.observation_node:main",
            "diag_thruster_map = brov_base.diag_thruster_map:main",
            "diag_loop_delay = brov_base.diag_loop_delay:main",
            "diag_link_rtt = brov_base.diag_link_rtt:main",
            "diag_depth_gate = brov_base.diag_depth_gate:main",
        ],
    },
)
