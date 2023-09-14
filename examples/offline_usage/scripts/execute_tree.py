import rclpy
from tree_translator import factory

def main():

    # Init ros
    rclpy.init(args=None)

    # Generate the executable tree using the factory
    tree_file_path = "self_contained_tree.xml"
    tree_factory = factory.TreeFactory()
    tree = tree_factory.create_tree_from_file(tree_file_path)

    # Init the execution loop
    tree.tick_tock(period_ms=1000)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
    
main()