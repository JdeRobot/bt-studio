import JSZip from "jszip"

const executeDocker = 
`import functools
import sys
import rclpy
import py_trees
from rclpy.node import Node
import tree_factory
import os
from tree_tools import ascii_bt_to_json


def start_console():
    # Get all the file descriptors and choose the latest one
    fds = os.listdir("/dev/pts/")
    console_fd = str(max(map(int, fds[:-1])))

    sys.stderr = open("/dev/pts/" + console_fd, "w")
    sys.stdout = open("/dev/pts/" + console_fd, "w")
    sys.stdin = open("/dev/pts/" + console_fd, "r")


def close_console():
    sys.stderr.close()
    sys.stdout.close()
    sys.stdin.close()


class TreeExecutor(Node):
    def __init__(self):
        super().__init__("tree_executor_node")
        # Get the path to the root of the package
        ws_path = "/workspace/code"
        tree_path = os.path.join(ws_path, "self_contained_tree.xml")
        actions_path = os.path.join(ws_path, "actions")

        factory = tree_factory.TreeFactory()
        self.tree = factory.create_tree_from_file(tree_path, actions_path)
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.tick_tock(period_ms=50)

    def post_tick_handler(self, snapshot_visitor, behaviour_tree):
        with open("/tmp/tree_state", "w") as f:
            ascii_bt_to_json(
                py_trees.display.ascii_tree(
                    behaviour_tree.root,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.visited,
                ),
                py_trees.display.ascii_blackboard(),
                f,
            )

    def spin_tree(self):

        try:
            rclpy.spin(self.tree.node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            self.tree.shutdown()
        finally:
            print("Shutdown completed")


def main():
    # Start the console
    start_console()
    # Init the components
    rclpy.init()
    executor = TreeExecutor()
    # Spin the tree
    executor.spin_tree()
    # Close the console
    close_console()


main()
`

const thirdparty = `repositories:
  ThirdParty/tb4_sim:
    type: git
    url: https://github.com/OscarMrZ/tb4_sim.git
    version: main
  ThirdParty/py_trees_ros_viewer:
    type: git
    url: https://github.com/splintered-reality/py_trees_ros_viewer.git
    version: devel
`

const setupConfig = (name:string) => {
  return `[develop]
script_dir=$base/lib/ros_template
[install]
install_scripts=$base/lib/`+name+`
`
}

const setupPython = (name:string) => {
  return `from setuptools import setup
from setuptools import find_packages

package_name = "`+name+`"

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
  `
}

const executeLocal = (name:string) => {
  return `import rclpy
from rclpy.node import Node
from tree_gardener import tree_factory
from ament_index_python.packages import get_package_share_directory
import os


class TreeExecutor(Node):
    def __init__(self):

        super().__init__("tree_executor_node")

        # Get the path to the root of the package
        pkg_share_dir = get_package_share_directory("`+name+`")
        tree_path = os.path.join(pkg_share_dir, "resource", "app_tree.xml")

        factory = tree_factory.TreeFactory()
        self.tree = factory.create_tree_from_file(tree_path)
        self.tree.tick_tock(period_ms=50)

    def spin_tree(self):

        try:
            rclpy.spin(self.tree.node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            self.tree.shutdown()
        finally:
            print("Shutdown completed")


def main():

    # Init the components
    rclpy.init()
    executor = TreeExecutor()

    # Spin the tree
    executor.spin_tree()
  `
}

const packageInfo = (name:string, extraDependencies: string[]) => {
  //TODO: add also more user info
  var dependStr = `<exec_depend>rclpy</exec_depend>`

  extraDependencies.forEach(depend => {
    dependStr += `
  <exec_depend>`+ depend +`</exec_depend>`
  }); 

  return `<?xml version="1.0"?>
<package format="3">
  <name>`+name+`</name>
  <version>0.1.0</version>
  <description>A ROS2 app generated BT Studio</description>
  <maintainer email="mantainer@mail.com">Mantainer</maintainer>
  <license>GPLv3</license>

  `
  + dependStr +
  `

  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
  `
}

namespace RosTemplates {
  export function addDockerFiles(zip:JSZip ) {
    zip.file("execute_docker.py", executeDocker);
  }

  export function addLocalFiles(zip:JSZip, project_name: string, tree_xml:string, depend: string[]) {
    const loc = zip.folder(project_name);

    if (loc == null) {
        return
    } 

    loc.file("resource/"+project_name, "");
    loc.file("resource/app_tree.xml", tree_xml);

    loc.file(project_name + "/execute.py", executeLocal(project_name));
    loc.file(project_name + "/__init__.py", "");

    loc.file("package.xml", packageInfo(project_name, depend));
    loc.file("setup.cfg", setupConfig(project_name));
    loc.file("setup.py", setupPython(project_name));
    loc.file("thirdparty.repos", thirdparty);
  }
}

export default RosTemplates;