from xml.etree.ElementTree import Element, SubElement, ElementTree, tostring
import json
from xml.dom import minidom
import os


def prettify_xml(element):
    """Return a pretty-printed XML string for the Element."""
    rough_string = tostring(element, "utf-8")
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


def get_tree_structure(link_models, node_models):

    # Create a dictionary to store the tree connections
    tree_structure = {}

    # Build the tree structure
    for node_id, node_info in node_models.items():

        node_type = node_info["type"]
        node_ports = node_info["ports"]

        # If the node is a tag, there are other construction mechanisms
        if node_type == "tag":
            continue

        # Search for the children port
        for port in node_ports:

            if port["type"] == "children":

                # The current node_id is always the source
                if node_id not in tree_structure:
                    tree_structure[node_id] = []

                # Get all the links
                children_links = port["links"]
                for link in children_links:

                    link_info = link_models[link]

                    # The user may have connected the node in both directions
                    child_node = (
                        link_info["target"]
                        if link_info["target"] != node_id
                        else link_info["source"]
                    )
                    tree_structure[node_id].append(child_node)

    return tree_structure


def get_data_ports(node_models, link_models, node_id):

    data_ports = {}

    # Obtain a list of ports and iterate
    ports = node_models[node_id]["ports"]

    for port in ports:

        if port["type"] == "input" or port["type"] == "output":

            # Get the link connecting the port to the value tag
            tag_port_link = link_models[port["links"][0]]

            # The current node name is needed for checking which end of the link is the tag
            node_name = node_models[node_id]["name"]

            # Get the nodes in both ends of the link
            target_name = node_models[tag_port_link["target"]]["name"]
            source_name = node_models[tag_port_link["source"]]["name"]

            # Store the port name-value pair
            port_value = target_name if target_name != node_name else source_name
            data_ports[port["name"]] = port_value

    return data_ports


def build_xml(node_models, link_models, tree_structure, node_id, xml_parent, order):
    print(order)
    node_name = node_models[node_id]["name"]
    data_ports = get_data_ports(node_models, link_models, node_id)

    # Add data_ports as attributes to current_element
    attributes = {"name": node_name}
    attributes.update(data_ports)
    current_element = SubElement(xml_parent, node_name, **attributes)

    # Apply recursion to all its children
    if node_id in tree_structure:
        tree_structure[node_id] = sorted(
            tree_structure[node_id],
            key=lambda item: node_models[item]["y"],
            reverse=order,
        )  # Fixed: issue #73
        for child_id in tree_structure[node_id]:
            build_xml(
                node_models,
                link_models,
                tree_structure,
                child_id,
                current_element,
                order,
            )


def build_tree_structure(node_models, link_models, tree_structure, node_id, order):

    node_name = node_models[node_id]["name"]
    data_ports = get_data_ports(node_models, link_models, node_id)

    # Add data_ports as attributes to current_element
    attributes = {"name": node_name, "id": node_id, "childs": []}

    # Apply recursion to all its children
    if node_id in tree_structure:
        tree_structure[node_id] = sorted(
            tree_structure[node_id],
            key=lambda item: node_models[item]["y"],
            reverse=order,
        )  # Fixed: issue #73
        for child_id in tree_structure[node_id]:
            attributes["childs"].append(
                build_tree_structure(
                    node_models,
                    link_models,
                    tree_structure,
                    child_id,
                    order,
                )
            )

    return attributes


def get_start_node_id(node_models, link_models):

    start_node_id = ""

    for node_id in node_models:

        node_name = node_models[node_id]["name"]
        if node_name == "Tree Root":

            # Obtain the link to the next children (tree root must only have one children)
            root_first_link_id = node_models[node_id]["ports"][0]["links"][0]
            root_first_link = link_models[root_first_link_id]

            start_node_id = root_first_link["target"]

    return start_node_id


def translate(content, tree_path, raw_order):

    # Parse the JSON data
    try:
        parsed_json = json.loads(content)
    except json.JSONDecodeError as e:
        raise ValueError(f"Invalid JSON content: {e}")

    try:
        # Extract nodes and links information
        node_models = parsed_json["layers"][1]["models"]
        link_models = parsed_json["layers"][0]["models"]

        # Get the tree structure
        tree_structure = get_tree_structure(link_models, node_models)

        # Get the order of bt: True = Ascendent; False = Descendent
        order = raw_order == "bottom-to-top"

        # Generate XML
        root = Element("Root", name="Tree Root")
        behavior_tree = SubElement(root, "BehaviorTree")
        start_node_id = get_start_node_id(node_models, link_models)
        build_xml(
            node_models,
            link_models,
            tree_structure,
            start_node_id,
            behavior_tree,
            order,
        )
    except Exception as e:
        tree_name = os.path.splitext(os.path.basename(tree_path))[0]
        raise RuntimeError(f"Failed to translate tree '{tree_name}': {e}")

    # Save the xml in the specified route
    xml_string = prettify_xml(root)
    print("ree before sub substitution: ", xml_string)
    f = open(tree_path, "w")
    f.write(xml_string)
    f.close()


def translate_tree_structure(content):
    # Parse the JSON data
    parsed_json = content

    # Extract nodes and links information
    node_models = parsed_json["layers"][1]["models"]
    link_models = parsed_json["layers"][0]["models"]

    # Get the tree structure
    tree_structure = get_tree_structure(link_models, node_models)
    # Get the order of bt: True = Ascendent; False = Descendent
    # order = raw_order == "bottom-to-top"

    # Generate XML
    start_node_id = get_start_node_id(node_models, link_models)
    root = build_tree_structure(
        node_models, link_models, tree_structure, start_node_id, False
    )

    return root
