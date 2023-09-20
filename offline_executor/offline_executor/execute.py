import rclpy
from rclpy.node import Node
from tree_gardener import tree_factory

class TreeExecutor(Node):

    def __init__(self):
        
        super().__init__('tree_executor_node')
        
        # Declare the parameter and its default value
        self.declare_parameter("tree_path", "final_tree.xml")

        # Retrieve the parameter value
        tree_path = self.get_parameter("tree_path").value

        factory = tree_factory.TreeFactory()
        self.tree = factory.create_tree_from_file(tree_path)
        self.tree.tick_tock(period_ms=1000)
        
    def spin_tree(self):

        try:
            rclpy.spin(self.tree.node)
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            pass
        finally:
            self.tree.shutdown()

def main():
    
    # Init the components
    rclpy.init()
    executor = TreeExecutor()

    # Spin the tree
    # executor.spin_tree()
    rclpy.shutdown()