# File Abstraction Layer

import os
import shutil
from .project_view import list_dir
from .exceptions import InvalidPath, ResourceNotExists, ResourceAlreadyExists


class FAL:
    """File Abstraction Layer"""

    def __init__(self, base):
        self.base = base

    def base_path(self) -> str:
        return self.path_join(self.base, "filesystem")

    def project_path(self, project_name) -> str:
        return self.path_join(self.base_path(), project_name)

    def universes_path(self, project_name) -> str:
        return self.path_join(self.project_path(project_name), "universes")

    def code_path(self, project_name) -> str:
        return self.path_join(self.project_path(project_name), "code")

    def actions_path(self, project_name) -> str:
        return self.path_join(self.code_path(project_name), "actions")

    def trees_path(self, project_name) -> str:
        return self.path_join(self.code_path(project_name), "trees")

    def subtrees_path(self, project_name) -> str:
        return self.path_join(self.trees_path(project_name), "subtrees")

    def path_join(self, a: str, b: str) -> str:
        return os.path.join(a, b)

    def exists(self, path: str) -> bool:
        return os.path.exists(path)

    def isdir(self, path: str) -> bool:
        return os.path.isdir(path)

    def isfile(self, path: str) -> bool:
        return os.path.isfile(path)

    def create(self, path: str, content):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path):
            raise ResourceAlreadyExists(path)

        with open(path, "w") as f:
            f.write(content)

    def create_binary(self, path: str, content):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path):
            raise ResourceAlreadyExists(path)

        with open(path, "wb") as f:
            f.write(content)

    def write(self, path: str, content):
        if not self.exists(path):
            raise ResourceNotExists(path)

        with open(path, "w") as f:
            f.write(content)

    def read(self, path: str) -> str:
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)
        with open(path, "r") as f:
            return f.read()

    def listdirs(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

        return [d for d in os.listdir(path) if self.isdir(self.path_join(path, d))]

    def listfiles(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

        return [d for d in os.listdir(path) if self.isfile(self.path_join(path, d))]

    def list_formatted(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

        return list_dir(path, path)

    def get_base_tree_template(self):
        init_graph_path = self.path_join(self.base, "templates/graph.json")
        return self.read(init_graph_path)

    def get_base_subtree_template(self):
        init_graph_path = self.path_join(self.base, "templates/graph.json")
        return self.read(init_graph_path)

    def get_action_template(self, filename, template):
        templates_folder_path = self.path_join(self.base, "templates")
        template_path = self.path_join(templates_folder_path, template)
        file_data = self.read(template_path)
        new_data = file_data.replace("ACTION", filename)
        return new_data

    def get_universe_template(self, universe):
        contents = []
        templates_folder_path = self.path_join(self.base, "templates/universe")
        launch_path = self.path_join(templates_folder_path, "launch/universe.launch.py")
        world_path = self.path_join(templates_folder_path, "worlds/universe.world")
        cmake_path = self.path_join(templates_folder_path, "CMakeLists.txt")
        vis_config_path = self.path_join(templates_folder_path, "gz.config")
        package_path = self.path_join(templates_folder_path, "package.xml")

        file_data = self.read(launch_path)
        new_data = file_data.replace("REPLACE", universe)
        contents.append({"path": "launch/universe.launch.py", "content": new_data})

        file_data = self.read(world_path)
        contents.append({"path": "worlds/universe.world", "content": file_data})

        file_data = self.read(vis_config_path)
        contents.append({"path": "gz.config", "content": file_data})

        file_data = self.read(cmake_path)
        new_data = file_data.replace("REPLACE", universe)
        contents.append({"path": "CMakeLists.txt", "content": new_data})

        file_data = self.read(package_path)
        new_data = file_data.replace("REPLACE", universe)
        contents.append({"path": "package.xml", "content": new_data})
        return contents

    def mkdir(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path):
            raise ResourceAlreadyExists(path)

        os.makedirs(path)

    def renamefile(self, old_path: str, new_path: str):
        if ".." in new_path:
            raise InvalidPath(new_path)

        if not self.exists(old_path):
            raise ResourceNotExists(old_path)

        if self.exists(new_path):
            raise ResourceAlreadyExists(new_path)

        os.rename(old_path, new_path)

    def renamedir(self, old_path: str, new_path: str):
        if ".." in new_path:
            raise InvalidPath(new_path)

        if not self.exists(old_path):
            raise ResourceNotExists(old_path)

        if self.exists(new_path):
            raise ResourceAlreadyExists(new_path)

        os.rename(old_path, new_path)

    def removefile(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)

        if not self.isfile(path):
            raise ResourceNotExists(path)

        os.remove(path)

    def removedir(self, path: str):
        if ".." in path:
            raise InvalidPath(path)

        if not self.exists(path):
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

        shutil.rmtree(path)

    def filename(self, path: str) -> str:
        return os.path.splitext(os.path.basename(path))[0]
