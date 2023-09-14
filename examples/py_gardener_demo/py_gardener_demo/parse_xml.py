import sys
import os
from tree_translator import parser
from ament_index_python.packages import get_package_share_directory, get_package_prefix

def main():
    # Get the path to the root of the package
    demo_share_dir = get_package_share_directory('py_gardener_demo')
    demo_root_dir = get_package_prefix('py_gardener_demo')

    # Init tree and action paths
    init_tree_path = os.path.join(demo_share_dir, 'resource', 'basic_tree.xml')
    action_path = os.path.join(demo_share_dir, 'actions')

    # Get a formatted self-contained tree string
    formatted_xml = parser.parse_tree(init_tree_path, action_path + "/")

    # Path where the xml will be stored for later access
    result_tree_path = os.path.join(demo_share_dir, 'resource', 'final_tree.xml')

    # Adjust the path to point to the src/resource directory
    src_resource_dir = os.path.join(demo_root_dir, '..', '..', 'src', 'BehaviorTrees', 'py_gardener_demo', 'resource')
    readable_tree_path = os.path.join(src_resource_dir, 'final_tree.xml')

    # Store the string in a temp xml file
    with open(result_tree_path, "w") as result_file:
        result_file.write(formatted_xml)
    with open(readable_tree_path, "w") as readable_file:
        readable_file.write(formatted_xml)

main()
