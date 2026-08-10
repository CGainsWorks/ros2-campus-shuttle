from setuptools import setup

package_name = "shuttle_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="CGainsWorks",
    maintainer_email="114424105+CGainsWorks@users.noreply.github.com",
    description="Camera-to-LiDAR extrinsic calibration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "calibrate_extrinsics = "
            "shuttle_perception.calibrate_extrinsics:main",
            "synth_calib_data = shuttle_perception.synth_calib_data:main",
            "collect_calib_data = shuttle_perception.collect_calib_data:main",
        ],
    },
)
