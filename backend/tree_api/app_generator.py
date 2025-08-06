import json
import os
import re
from . import tree_generator
from . import json_translator

##############################################################################
# Helper functions
##############################################################################


# Collect unique imports from files
def get_unique_imports(fal, project_name):
    action_path = fal.actions_path(project_name)
    unique_imports = set()

    actions_list = fal.listfiles(action_path)
    actions_list.sort()
    for action_file in actions_list:
        if action_file.endswith(".py"):
            path = fal.path_join(action_path, action_file)
            action_content = fal.read(path)

            lines = action_content.splitlines()

            for line in lines:
                # Using regex to find lines that don't start with '#' and have 'import ...' or 'from ... import ...'
                match = re.search(r"^(?!#.*)(?:import|from)\s+(\w+)", line)
                if match:
                    unique_imports.add(match.group(1))

    return list(unique_imports)

def add_actions_in_tree(actions,actions_list, subtrees, subtrees_list, tree):
    for entry in tree["childs"]:
        if entry["name"] in actions_list:
            actions.add(entry["name"])
        elif entry["name"] in subtrees_list:
            subtrees.add(entry["name"])
        else:
            add_actions_in_tree(actions, actions_list, subtrees, subtrees_list, entry)

def get_tree_data(fal, project, graph_data, bt_order, actions = set(), subtrees = set()):
    subtree_path = fal.subtrees_path(project)
    actions_path = fal.actions_path(project)
    subtrees_inside = set()

    # Check if the project exists
    # 1. Generate a basic tree from the JSON definition
    main_tree = json_translator.translate_tree_structure(json.loads(graph_data), bt_order)

    try:
        actions_list = fal.listfiles(actions_path)
        actions_list = [os.path.splitext(x)[0] for x in actions_list]
    except:
        actions_list = []

    try:
        subtrees_list = fal.listfiles(subtree_path)
        subtrees_list = [os.path.splitext(x)[0] for x in subtrees_list]
    except:
        subtrees_list = []

    if main_tree["name"] in actions_list:
        actions.add(main_tree["name"])
    elif main_tree["name"] in subtrees_list:
        subtrees_inside.add(main_tree["name"])

    add_actions_in_tree(actions, actions_list, subtrees_inside, subtrees_list, main_tree)

    for subtree in subtrees_inside:
        if subtree not in subtrees:
            subtrees.add(subtree)
            entry_path = fal.subtrees_path(project)
            graph_path = fal.path_join(entry_path, subtree + ".json")
            graph_data = fal.read(graph_path)
            get_tree_data(fal, project, graph_data, bt_order, actions, subtrees)
    
    return {"actions": list(actions), "subtrees": list(subtrees)}

def generate_app(fal, project_name, bt_order):
    # Make folder path relative to Django app
    trees_path = fal.trees_path(project_name)
    tree_path = fal.path_join(trees_path, "main.json")
    subtree_path = fal.subtrees_path(project_name)

    subtrees = []

    # Check if the project exists
    graph_data = fal.read(tree_path)
    # 1. Generate a basic tree from the JSON definition
    main_tree = json_translator.translate_raw(graph_data, bt_order)

    # 2. Get all possible subtrees name and content
    try:
        subtrees_list = fal.listfiles(subtree_path)
        subtrees_list.sort()
        for subtree_file in subtrees_list:
            if subtree_file.endswith(".json"):
                subtree_name = fal.filename(subtree_file)
                path = fal.path_join(subtree_path, subtree_file)
                subtree_json = fal.read(path)

                subtree = json_translator.translate_raw(subtree_json, bt_order)
                subtrees.append({"name": subtree_name, "content": subtree})
    except:
        print("No subtrees")

    # 3. Generate a self-contained tree
    return tree_generator.generate(main_tree, subtrees)
