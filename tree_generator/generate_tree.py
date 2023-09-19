import os
import argparse
from tree_translator import parser

def main(tree_path, action_path):

    # Ensure the provided tree and action paths exist
    if not os.path.exists(tree_path):
        raise FileNotFoundError(f"Tree path '{tree_path}' does not exist!")
    if not os.path.exists(action_path):
        raise FileNotFoundError(f"Action path '{action_path}' does not exist!")

    # Get a formatted self-contained tree string
    formatted_xml = parser.parse_tree(tree_path, action_path)

    # Path where the xml will be stored for later access
    result_tree_path = "self_contained_tree.xml"

    # Store the string in a temp xml file
    with open(result_tree_path, "w") as result_file:
        result_file.write(formatted_xml)


if __name__ == "__main__":

    # Use argparse to handle command line arguments
    parser = argparse.ArgumentParser(description="Generate self contained xml tree files from a basic xml and the actions")
    parser.add_argument('tree_path', type=str, help='Path to the tree file.')
    parser.add_argument('action_path', type=str, help='Path to the actions directory.')

    args = parser.parse_args()
    main(args.tree_path, args.action_path)
