from glob import glob

from setuptools import find_packages, setup


package_name = "brov_perception"


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
    extras_require={"test": ["pytest"]},
    zip_safe=True,
    maintainer="Pyo Seunghyeon",
    maintainer_email="jeongmok99@koreatech.ac.kr",
    description=(
        "BlueROV2 camera streaming, intrinsic calibration, and ArUco pose "
        "estimation."
    ),
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "camera_stream_node = brov_perception.camera_stream_node:main",
            (
                "checkerboard_calibration_node = "
                "brov_perception.checkerboard_calibration_node:main"
            ),
            "aruco_pose_node = brov_perception.aruco_pose_node:main",
        ],
    },
)
