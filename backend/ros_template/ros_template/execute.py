import rclpy
from rclpy.node import Node
from tree_gardener import tree_factory
from ament_index_python.packages import get_package_share_directory
import os


class TreeExecutor(Node):
    def __init__(self):

        super().__init__("tree_executor_node")

        # Get the path to the root of the package
        pkg_share_dir = get_package_share_directory("ros_template")
        tree_path = os.path.join(pkg_share_dir, "resource", "app_tree.xml")
        actions_path = os.path.join(pkg_share_dir, "actions")

        factory = tree_factory.TreeFactory()
        self.tree = factory.create_tree_from_file(tree_path, actions_path)
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
