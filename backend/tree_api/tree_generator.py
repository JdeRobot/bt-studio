import os
import argparse
import xml.etree.ElementTree as ET
from .json_translator import prettify_xml

##############################################################################
# Parser functions
##############################################################################


# Get the indentation of a given line
def get_line_indentation(line) -> int:

    indent = len(line) - len(line.strip())

    return indent


# Fix the indentation in a xml string
def fix_indentation(xml_string, actions):

    lines = xml_string.split("\n")
    processed_lines = list()

    code_section = False

    for line in lines:

        if "Code>" in line:
            code_section = not code_section

        new_line = line
        if code_section and "Code>" not in line:
            if all(action + ">" not in line for action in actions):
                new_line = " " * 6 + new_line

        processed_lines.append(new_line)

    pretty_str = "\n".join(line for line in processed_lines)
    return pretty_str


# Extract a BT structure from a XML file
def get_bt_structure(xml_string) -> str:

    root = ET.fromstring(xml_string)
    behavior_tree_element = root.find(".//BehaviorTree")

    if behavior_tree_element is None:
        print("No BehaviorTree found in the XML")
        return None

    return root


# Get a list of properly named actions to search in the nodes directory
def get_action_set(tree, possible_actions) -> set:
    actions = set()
    for leaf in tree:

        if leaf.tag not in actions and leaf.tag in possible_actions:
            actions.add(leaf.tag)
        child_actions = get_action_set(leaf, possible_actions)
        actions.update(a for a in child_actions if a not in actions)

    return actions


# Get a list of properly named subtrees to substitute later in the tree
def get_subtree_set(tree, possible_subtrees) -> set:

    subtrees = set()

    for leaf in tree:

        if leaf.tag not in subtrees and leaf.tag in possible_subtrees:
            subtrees.add(leaf.tag)
        child_subtrees = get_subtree_set(leaf, possible_subtrees)
        subtrees.update(a for a in child_subtrees if a not in subtrees)

    return subtrees


# Add the code of the different actions
def add_actions_code_(tree, actions, all_actions):

    code_section = ET.SubElement(tree, "Code")

    # Add each action code to the tree
    for action_name in actions:
        action_code = None
        for action in all_actions:
            if action_name == action["name"]:
                action_code = action["content"]

        # Verify if action code exists
        if action_code == None:
            continue

        # Add a new subelement to the code_section
        action_section = ET.SubElement(code_section, action_name)
        action_section.text = "\n" + action_code + "\n"


# Replaces all the subtrees in a given tree depth
def replace_subtrees_in_tree_(tree, subtrees):

    for subtree in subtrees:
        subtree_name = subtree["name"]
        subtree_xml = subtree["content"]
        subtree_tree = ET.fromstring(subtree_xml)

        # Find the content inside the <BehaviorTree> tag in the subtree
        subtree_behavior_tree = subtree_tree.find(".//BehaviorTree")
        if subtree_behavior_tree is not None:
            # Locate tags in the main tree that refer to this subtree
            subtree_parents = tree.findall(".//" + subtree_name + "/..")
            for parent in subtree_parents:
                subtree_tags = parent.findall(subtree_name)
                for subtree_tag in subtree_tags:
                    # Get the index of the original subtree tag
                    index = list(parent).index(subtree_tag)

                    # Insert new elements from the subtree at the correct position
                    for i, subtree_elem in enumerate(subtree_behavior_tree):
                        try:
                            parent.insert(index + i, subtree_elem)
                        except Exception as e:
                            print(str(e))
                    # Remove the original subtree tag
                    parent.remove(subtree_tag)


# Recursively replace all subtrees in a given tree
def replace_all_subtrees_(tree, all_subtrees, depth=0, max_depth=15):

    # Avoid infinite recursion
    if depth > max_depth:
        return

    # Get the subtrees that are present in the tree
    possible_trees = [x["name"] for x in all_subtrees]
    subtrees = get_subtree_set(tree, possible_trees)

    # If no subtrees are found, stop the recursion
    if not subtrees:
        return

    # Replace subtrees in the main tree
    replace_subtrees_in_tree_(tree, all_subtrees)

    # Recursively call the function to replace subtrees in the newly added subtrees
    replace_all_subtrees_(tree, all_subtrees, depth + 1, max_depth)


def parse_tree_(tree_xml, subtrees, all_actions):
    # Parse the tree file
    tree = get_bt_structure(tree_xml)

    # Obtain the defined subtrees recursively
    replace_all_subtrees_(tree, subtrees)

    # Obtain the defined actions
    possible_actions = [x["name"] for x in all_actions]
    actions = get_action_set(tree, possible_actions)
    actions = sorted(actions)

    # Add subsections for the action code
    add_actions_code_(tree, actions, all_actions)

    # Serialize the modified XML to a properly formatted string
    formatted_tree = prettify_xml(tree)
    formatted_tree = fix_indentation(formatted_tree, actions)

    return formatted_tree


##############################################################################
# Main section
##############################################################################


def generate(tree_xml, subtrees, actions):

    # Get a formatted self-contained tree string
    formatted_xml = parse_tree_(tree_xml, subtrees, actions)

    # Return the xml string
    return formatted_xml
