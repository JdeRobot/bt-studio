import sys
import os
from tree_translator import parser

def main():

    # Get a formatted self contained tree string
    tree_file_path = "../resource/basic_tree.xml"
    action_path = "../actions/"
    formatted_xml = parser.parse_tree(tree_file_path, action_path)

    # Store the string in a temp xml file
    self_contained_tree_file = open("../resource/self_contained_file.xml", "w")
    self_contained_tree_file.write(formatted_xml)
    self_contained_tree_file.close()

main()