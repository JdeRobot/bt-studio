import rclpy
from rclpy.node import Node
import tree_factory
import os

class TreeExecutor(Node):

    def __init__(self):
        
        super().__init__('tree_executor_node')
        
        # Get the path to the root of the package
        ws_path = "/workspace/code"
        tree_path = os.path.join(ws_path, 'self_contained_tree.xml')

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

main()