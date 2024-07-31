from setuptools import setup
from setuptools import find_packages

package_name = "tree_gardener"

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
    author="Óscar Martínez",
    author_email="oscar.robotics@tutanota.com",
    maintainer="Óscar Martínez",
    maintainer_email="oscar.robotics@tutanota.com",
    keywords=["ROS2", "py_trees"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="BT execution engine for BT Studio",
    license="GPLv3",
    tests_require=["pytest"],
)
