import re
from . import tree_generator
from . import json_translator

##############################################################################
# Helper functions
##############################################################################


# Collect unique imports from files
def get_unique_imports(actions):

    unique_imports = set()

    for action in actions:
        lines = action["content"].splitlines()

        for line in lines:
            # Using regex to find lines that don't start with '#' and have 'import ...' or 'from ... import ...'
            match = re.search(r"^(?!#.*)(?:import|from)\s+(\w+)", line)
            if match:
                unique_imports.add(match.group(1))

    return list(unique_imports)


def generate_app(fal, project_name, bt_order):
    # Make folder path relative to Django app
    action_path = fal.actions_path(project_name)
    trees_path = fal.trees_path(project_name)
    tree_path = fal.path_join(trees_path, "main.json")
    subtree_path = fal.subtrees_path(project_name)

    subtrees = []
    actions = []

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

    # 3. Get all possible actions name and content
    actions_list = fal.listfiles(action_path)
    actions_list.sort()
    for action_file in actions_list:
        if action_file.endswith(".py"):
            action_name = fal.filename(action_file)
            path = fal.path_join(action_path, action_file)
            action_content = fal.read(path)

            actions.append({"name": action_name, "content": action_content})

    # 4. Generate a self-contained tree
    return tree_generator.generate(main_tree, subtrees, actions), actions
