from glob import glob
from setuptools import find_packages, setup


package_name = "drone_auto_bag_record"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "LICENSE"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    scripts=[
        "scripts/record.sh",
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Chanjoon Park",
    maintainer_email="chanjoon.park@kaist.ac.kr",
    author="Eungchang Mason Lee",
    author_email="eungchang_mason@kaist.ac.kr",
    description="Arming-triggered rosbag2 recorder for PX4/MAVROS flight tests.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "auto_bag_recorder = drone_auto_bag_record.auto_bag_recorder:main",
        ],
    },
)
