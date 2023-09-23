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

    return list(unique_imports)

def update_package_xml(package_xml_path, unique_imports):

    valid_imports = []
    
    for imp in unique_imports:
        try:
            result = subprocess.run(['rosdep', 'where-defined', imp], capture_output=True, text=True, check=True)
            if "ERROR" not in result.stdout:
                valid_imports.append(imp)
        except subprocess.CalledProcessError:
            print(f"Error while running rosdep for {imp}. Skipping this package.")

    with open(package_xml_path, 'r') as file:
        content = file.read()

    # Finding the position of the last </exec_depend> tag
    last_exec_depend_index = content.rfind('</exec_depend>') + len('</exec_depend>')

    # Generating new <exec_depend> entries from the valid imports
    new_exec_depends = '\n'.join([f'  <exec_depend>{imp}</exec_depend>' for imp in valid_imports])

    # Inserting the new dependencies after the last </exec_depend>
    updated_content = content[:last_exec_depend_index] + '\n' + new_exec_depends + content[last_exec_depend_index:]

    # Writing the updated content back to package.xml
    with open(package_xml_path, 'w') as file:
        file.write(updated_content)

# Setup the package with the user data
def setup_package(temp_path, user_data):

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
    action_paths = get_actions_paths("actions/")
    imports = get_unique_imports(action_paths)

    # 4. Update the template package xml so the dependencies can be installed with rosdep
    package_xml_path = os.path.join(temp_path, "package.xml")
    update_package_xml(package_xml_path, imports)


##############################################################################
# Main section
##############################################################################

def main(app_tree, app_name):

    src_path = "ros_template"
    temp_path = "/tmp/ros_template"

    # Ensure the files exist
    if not os.path.exists(app_tree):
        raise FileNotFoundError(f"Tree path '{app_tree}' does not exist!")

    # 1. Copy the template to a temporary directory
    if os.path.exists(temp_path):
        shutil.rmtree(temp_path)  # Delete if it already exists
    shutil.copytree(src_path, temp_path)
    print(f"Template copied to {temp_path}")

    # 2. Copy the tree to the template directory
    tree_location = temp_path + "/resource/app_tree.xml"
    shutil.copy(app_tree, tree_location)

    # 3. Edit some files in the template
    user_data = {"app_name": app_name}
    setup_package(temp_path, user_data)

    # 3. Generate a zip file in the destination folder with a name specified by the user
    dest_zip = f"{app_name}.zip"
    with zipfile.ZipFile(dest_zip, 'w') as zipf:
        zipdir(temp_path, zipf)
    print(f"Directory compressed to {dest_zip}")

if __name__ == "__main__":

    # Use argparse to handle command line arguments
    parser = argparse.ArgumentParser(description="Generate a ROS2 app with a self-contained tree")
    parser.add_argument('app_tree', type=str, help='Path to the tree file to use in the app')
    parser.add_argument('app_name', type=str, help='Name of the generated app')

    args = parser.parse_args()
    main(args.app_tree, args.app_name)
