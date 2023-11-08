import os
import argparse
import shutil
import zipfile
import re
import subprocess

##############################################################################
# Helper functions
##############################################################################

# Zip a directory
def zipdir(path, ziph):

    for root, dirs, files in os.walk(path):
        for file in files:
            ziph.write(os.path.join(root, file), os.path.relpath(os.path.join(root, file), path))

# Rename all the necessary files in the template
def rename_template_files(root_path, original_str, replacement_str):

    for dirpath, dirnames, filenames in os.walk(root_path, topdown=False):
        
        # Rename directories
        for dirname in dirnames:
            if original_str in dirname:
                src_dir = os.path.join(dirpath, dirname)
                dst_dir = os.path.join(dirpath, dirname.replace(original_str, replacement_str))
                os.rename(src_dir, dst_dir)
        
        # Rename files
        for filename in filenames:
            if original_str in filename:
                src_file = os.path.join(dirpath, filename)
                dst_file = os.path.join(dirpath, filename.replace(original_str, replacement_str))
                os.rename(src_file, dst_file)

# Replace a str in a file for another
def replace_contents_in_file(file_path, original_str, replacement_str):

    with open(file_path, 'r') as file:
        file_data = file.read()
    
    new_data = file_data.replace(original_str, replacement_str)
    
    with open(file_path, 'w') as file:
        file.write(new_data)

def get_actions_paths(dir_to_scan):

    files_to_scan = []
    for root, _, filenames in os.walk(dir_to_scan):
        for filename in filenames:
            if filename.endswith('.py'):
                files_to_scan.append(os.path.join(root, filename))

    return files_to_scan

# Collect unique imports from files
def get_unique_imports(file_paths):

    unique_imports = set()
    
    for file_path in file_paths:
        if not os.path.exists(file_path):
            print(f"Warning: File {file_path} does not exist. Skipping import collection for this file.")
            continue

        with open(file_path, 'r') as file:
            lines = file.readlines()

        for line in lines:
            # Using regex to find lines that don't start with '#' and have 'import ...' or 'from ... import ...'
            match = re.search(r'^(?!#.*)(?:import|from)\s+(\w+)', line)
            if match:
                unique_imports.add(match.group(1))
    
    f = open("/tmp/imports.txt", 'w')
    f.write(f"Action paths: {unique_imports}.")
    f.close()

    return list(unique_imports)

def update_package_xml(package_xml_path, unique_imports):
    # Mapping from Python import names to ROS package names
    special_imports = {
        'cv2': 'python3-opencv',
        # Add more mappings here as needed
    }

    with open(package_xml_path, 'r') as file:
        content = file.read()

    # Finding the position of the last </exec_depend> tag
    last_exec_depend_index = content.rfind('</exec_depend>') + len('</exec_depend>')

    # Replacing special import names and generating new <exec_depend> entries
    new_exec_depends = '\n'.join(
        [f'  <exec_depend>{special_imports.get(imp, imp)}</exec_depend>' for imp in unique_imports]
    )

    # Inserting the new dependencies after the last </exec_depend>
    updated_content = content[:last_exec_depend_index] + '\n' + new_exec_depends + content[last_exec_depend_index:]

    # Writing the updated content back to package.xml
    with open(package_xml_path, 'w') as file:
        file.write(updated_content)

# Setup the package with the user data
def setup_package(temp_path, action_path, user_data):

    app_name = user_data['app_name']
    template_str = "ros_template"
    
    # 1. Rename directories and files recursively
    rename_template_files(temp_path, template_str, app_name)
    
    # 2. Replace the original_str with app_name in the content of relevant files
    files_to_edit = ["package.xml", "setup.py", "setup.cfg", app_name + "/execute.py"]
    for file_name in files_to_edit:
        file_path = os.path.join(temp_path, file_name)
        if os.path.exists(file_path):
            replace_contents_in_file(file_path, template_str, app_name)
        else:
            print(f"Warning: {file_name} not found in {temp_path}. Skipping content replacement for this file.")

    # 3. Get a list of unique imports from the user-defined actions
    action_paths = get_actions_paths(action_path)
    imports = get_unique_imports(action_paths)

    # 4. Update the template package xml so the dependencies can be installed with rosdep
    package_xml_path = os.path.join(temp_path, "package.xml")
    update_package_xml(package_xml_path, imports)


##############################################################################
# Main section
##############################################################################

def generate(app_tree, app_name, template_path, action_path, tree_gardener_src):

    app_path = "/tmp/" + app_name
    executor_path = app_path + "/" + app_name
    tree_gardener_dst = app_path + "/tree_gardener"

    # Ensure the files exist
    if not os.path.exists(app_tree):
        raise FileNotFoundError(f"Tree path '{app_tree}' does not exist!")

    # 1. Copy the template to a temporary directory
    if os.path.exists(executor_path):
        shutil.rmtree(executor_path)  # Delete if it already exists
    shutil.copytree(template_path, executor_path)
    print(f"Template copied to {executor_path}")

    # 2. Copy the tree to the template directory
    tree_location = executor_path + "/resource/app_tree.xml"
    shutil.copy(app_tree, tree_location)

    # 3. Edit some files in the template
    user_data = {"app_name": app_name}
    setup_package(executor_path, action_path, user_data)

    # 4. Copy the tree_gardener package to the app
    if os.path.exists(tree_gardener_dst):
        shutil.rmtree(tree_gardener_dst)  # Delete if it already exists
    shutil.copytree(tree_gardener_src, tree_gardener_dst)
    print(f"Tree gardener copied to {tree_gardener_dst}")

    # 3. Generate a zip file in the destination folder with a name specified by the user
    dest_path = app_path + ".zip"
    with zipfile.ZipFile(dest_path, 'w') as zipf:
        zipdir(app_path, zipf)
    print(f"Directory compressed to {dest_path}")

    return dest_path
