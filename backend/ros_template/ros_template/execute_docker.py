import functools
import rclpy
import py_trees
from rclpy.node import Node
import tree_factory
import os


class TreeExecutor(Node):
    def __init__(self):

        super().__init__("tree_executor_node")

        # Get the path to the root of the package
        ws_path = "/workspace/code"
        tree_path = os.path.join(ws_path, "self_contained_tree.xml")

        factory = tree_factory.TreeFactory()
        self.tree = factory.create_tree_from_file(tree_path)
        snapshot_visitor = py_trees.visitors.SnapshotVisitor()
        self.tree.add_post_tick_handler(
            functools.partial(self.post_tick_handler, snapshot_visitor)
        )
        self.tree.visitors.append(snapshot_visitor)
        self.tree.tick_tock(period_ms=50)

    def post_tick_handler(self, snapshot_visitor, behaviour_tree):
        with open("/tmp/tree_state", "w") as f:
            f.write(
                py_trees.display.ascii_tree(
                    behaviour_tree.root,
                    visited=snapshot_visitor.visited,
                    previously_visited=snapshot_visitor.visited,
                )
            )
            f.write(py_trees.display.ascii_blackboard())
            f.close()

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


main()
