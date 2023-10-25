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

    if port['type'] == "input port" or port['type'] == "output port":

      # Get the link connecting the port to the value tag
      tag_port_link = link_models[port['links'][0]]

      # The current node name is needed for checking which end of the link is the tag
      node_name = node_models[node_id]['name']
      
      # Get the nodes in both ends of the link
      target_name = node_models[tag_port_link['target']]['name']
      source_name = node_models[tag_port_link['source']]['name']

      # Store the port name-value pair
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

def get_start_node_id(node_models, link_models):

  start_node_id = ''

  for node_id in node_models:
    
    node_name = node_models[node_id]['name']
    if node_name == "Tree Root":
      
      # Obtain the link to the next children (tree root must only have one children)
      root_first_link_id = node_models[node_id]['ports'][0]['links'][0]
      root_first_link = link_models[root_first_link_id]

      start_node_id = root_first_link['target']
  
  return start_node_id
      

def translate(content):

  # Parse the JSON data
  parsed_json = json.loads(content)

  # Extract nodes and links information
  node_models = parsed_json['layers'][1]['models']
  link_models = parsed_json['layers'][0]['models']

  # Get the tree structure
  tree_structure = get_tree_structure(link_models)

  # Generate XML
  root = Element("Root", name="Tree Root")
  behavior_tree = SubElement(root, "BehaviorTree")
  start_node_id = get_start_node_id(node_models, link_models)
  build_xml(node_models, link_models, tree_structure, start_node_id, behavior_tree)
  
  # Debug the XML
  xml_string = prettify_xml(root)
  
  return xml_string