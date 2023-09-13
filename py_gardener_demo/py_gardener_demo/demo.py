import rclpy
from ament_index_python.packages import get_package_share_directory
from py_gardener import gn_factory
import os

##############################################################################
# Entry point
##############################################################################

def demo_main():

    # Init ros
    rclpy.init(args=None)

    # Get the path to the root of the package
    demo_root_dir = get_package_share_directory('py_gardener_demo')

    # Now, you can build paths relative to the root of the package
    tree_file_path = os.path.join(demo_root_dir, 'resource', 'bumpgo_ports.xml')
    actions_dir_path = os.path.join(demo_root_dir, 'actions/')

    gardener = gn_factory.Gardener()
    tree = gardener.create_tree_from_file(tree_file_path, actions_dir_path)

    # Init the execution loop
    tree.tick_tock(period_ms=1000)
    try:
        rclpy.spin(tree.node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        tree.shutdown()
        rclpy.try_shutdown()
