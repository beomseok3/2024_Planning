import os
from setuptools import setup, find_packages
from glob import glob

package_name = "parking"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="ps",
    maintainer_email="ps@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "parking_path = parking.parking_path:main",
            "db_write = parking.db_write:main",
            "db_read = parking.db_read:main",
            "db_local_write = parking.db_local_write:main",
            "line_detection_area_addition = parking.line_detection_area_addition:main",
            # "detection_area_addition = parking.detection_area_addition:main",
            "parallel_parking = parking.parallel_parking:main",
            "search_parking_line = parking.search_parking_line:main",
            "db_read_db2txt = parking.db_read_db2txt:main",
            "parking_bs = parking.parking_bs:main",
            "avoidance = parking.avoidance:main",
            "astar = parking.astar:main",
            
        ],
    },
)
