from setuptools import setup

package_name = "shuttle_localisation"

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
    description="EKF ground-truth evaluation and reference-path publishing.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ekf_evaluator = shuttle_localisation.ekf_evaluator:main",
            "path_publisher = shuttle_localisation.path_publisher:main",
        ],
    },
)
