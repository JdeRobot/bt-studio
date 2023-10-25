from xml.etree.ElementTree import Element, SubElement, ElementTree, tostring
import json
from xml.dom import minidom

def prettify_xml(element):
    """Return a pretty-printed XML string for the Element."""
    rough_string = tostring(element, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")

def get_tree_structure(link_models):
    
  # Create a dictionary to store the tree connections
  tree_structure = {}

  # Build the tree structure
  for link_id, link_info in link_models.items():
      
      source = link_info['source']
      target = link_info['target']
      
      if source not in tree_structure:
        tree_structure[source] = []
      
      tree_structure[source].append(target)

  return tree_structure

def get_data_ports(node_models, link_models, node_id):
    
  data_ports = {}

  # Obtain a list of ports and iterate
  ports = node_models[node_id]['ports']

  for port in ports:

    if port['type'] == "input port":
      
      tag_port_link = link_models[port['links'][0]]

      node_name = node_models[node_id]['name']

      target_name = node_models[tag_port_link['target']]['name']
      source_name = node_models[tag_port_link['source']]['name']
      port_value = target_name if target_name != node_name else source_name

      data_ports[port['name']] = port_value
    
  return data_ports

def build_xml(node_models, link_models, tree_structure, node_id, xml_parent):

  node_name = node_models[node_id]['name']
  data_ports = get_data_ports(node_models, link_models, node_id)

  # Add data_ports as attributes to current_element
  attributes = {'name': node_name}
  attributes.update(data_ports)
  current_element = SubElement(xml_parent, node_name, **attributes)

  # Apply recursion to all its children
  if node_id in tree_structure:
    for child_id in tree_structure[node_id]:

      build_xml(node_models, link_models, tree_structure, child_id, current_element)

def translate(json_file):

  # Parse the JSON data
  parsed_json = json.load(json_file)

  # Extract nodes and links information
  node_models = parsed_json['layers'][1]['models']
  link_models = parsed_json['layers'][0]['models']

  # Get the tree structure
  tree_structure = get_tree_structure(link_models)

  # Generate XML
  root = Element("Root", name="Tree Root")
  behavior_tree = SubElement(root, "BehaviorTree")
  build_xml(node_models, link_models, tree_structure, '497b70be-7cf9-4a79-b658-983970be4c06', behavior_tree)
  xml_string = prettify_xml(root)
  print(xml_string)

json_file = open('json_test.json')
translate(json_file)