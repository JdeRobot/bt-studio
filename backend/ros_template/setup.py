from setuptools import setup
from setuptools import find_packages

package_name = "ros_template"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/resource",
            [
                "resource/app_tree.xml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    author="",
    author_email="",
    maintainer="",
    maintainer_email="",
    keywords=["ROS2", "py_trees"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="A ROS2 app generated with tree_translator",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "executor = ros_template.execute:main",
        ],
    },
)
