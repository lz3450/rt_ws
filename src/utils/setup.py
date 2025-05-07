from setuptools import find_packages, setup

package_name = "utils"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kzl",
    maintainer_email="lz3450@outlook.com",
    description="Utils for RoboTrace robotic arm project",
    license="GPL-3.0-only",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "csv_logger = utils.csv_logger:main",
        ],
    },
)
