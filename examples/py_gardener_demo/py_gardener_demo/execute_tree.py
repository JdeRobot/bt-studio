import rclpy
from tree_translator import factory
from ament_index_python.packages import get_package_share_directory
import os

def execute_tree():

    # Init ros
    rclpy.init(args=None)

    # Get the path to the root of the package
    demo_root_dir = get_package_share_directory('py_gardener_demo')

    # Now, you can build paths relative to the root of the package
    tree_file_path = os.path.join(demo_root_dir, 'resource', 'self_contained_file.xml')

    # Generate the executable tree using the factory
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