# File Abstraction Layer

from abc import ABC, abstractmethod
import os
import shutil

from backend.tree_api.models import Project, get_user_projects_size
from .project_view import list_dir
from .exceptions import (
    BinaryNotSupported,
    InvalidPath,
    ResourceNotExists,
    ResourceAlreadyExists,
)


class FAL(ABC):
    """
    Abstract base class defining the File Abstraction Layer (FAL) interface.
    Provides a unified API for file and directory operations used by
    RoboticsAcademy exercises. Concrete implementations handle different
    storage backends (local filesystem, S3).
    """

    def __init__(self, projects=""):
        self.projects = projects
        self.user = None
        self.project = None

    def __copy__(self):
        return type(self)(self.projects)

    def set_user(self, user):
        """Set the current user context for file operations."""
        self.user = user

    def set_project(self, project):
        self.project = project

    @abstractmethod
    def projects_path(self) -> str:
        pass

    def project_path(self, project_id, change_proj=True) -> str:
        if change_proj:
            self.set_project(Project.objects.get(id=project_id, creator=self.user))
        return self.path_join(self.projects_path(), project_id + "/")

    @abstractmethod
    def library_path(self) -> str:
        pass

    def library_entry_path(self, entry) -> str:
        return self.path_join(self.library_path(), entry)

    def library_actions_path(self, entry) -> str:
        return self.path_join(self.library_entry_path(entry), "actions")

    def library_subtrees_path(self, entry) -> str:
        return self.path_join(self.library_entry_path(entry), "subtrees")

    def world_path(self, project_id) -> str:
        return self.path_join(self.project_path(project_id), "worlds/")

    def code_path(self, project_id) -> str:
        return self.path_join(self.project_path(project_id), "code/")

    def actions_path(self, project_id) -> str:
        return self.path_join(self.code_path(project_id), "actions/")

    def trees_path(self, project_id) -> str:
        return self.path_join(self.code_path(project_id), "trees/")

    def subtrees_path(self, project_id) -> str:
        return self.path_join(self.trees_path(project_id), "subtrees/")

    @abstractmethod
    def path_join(self, a: str, b: str) -> str:
        """Join two paths and return the result."""
        pass

    @abstractmethod
    def exists(self, path: str) -> bool:
        """Return file size if exists, 0 if directory, -1 if not found."""
        pass

    @abstractmethod
    def isdir(self, path: str) -> bool:
        """Return True if path is an existing directory."""
        pass

    @abstractmethod
    def isfile(self, path: str) -> bool:
        """Return True if path is an existing file."""
        pass

    @abstractmethod
    def create(self, path: str, content):
        """Create a new text file at path with given content. Raises InvalidPath or ResourceAlreadyExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) != -1:
            raise ResourceAlreadyExists(path)

        size = len(content.encode("utf-8"))

        if self.project is not None:
            self.project.update_size(self, size)
        self.user.update_size(self, size, project_callback=get_user_projects_size)

    @abstractmethod
    def create_binary(self, path: str, content):
        """Create a new binary file at path with given content. Raises InvalidPath or ResourceAlreadyExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) != -1:
            raise ResourceAlreadyExists(path)

        size = len(content)

        if self.project is not None:
            self.project.update_size(self, size)
        self.user.update_size(self, size, project_callback=get_user_projects_size)

    @abstractmethod
    def write(self, path: str, content):
        """Write content to an existing file. Raises ResourceNotExists if missing."""
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

        new_size = len(content.encode("utf-8"))

        if self.project is not None:
            self.project.update_size(self, new_size, size)
        self.user.update_size(
            self,
            new_size,
            size,
            project_callback=get_user_projects_size,
        )

    @abstractmethod
    def write_binary(self, path: str, content):
        """Write binary content to an existing file. Raises ResourceNotExists if missing."""
        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

        new_size = len(content)

        if self.project is not None:
            self.project.update_size(self, new_size, size)
        self.user.update_size(
            self,
            new_size,
            size,
            project_callback=get_user_projects_size,
        )

    @abstractmethod
    def read(self, path: str):
        """Read and return text content of file. Raises InvalidPath or ResourceNotExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def read_binary(self, path: str):
        """Read and return binary content of file. Raises InvalidPath or ResourceNotExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

    @abstractmethod
    def listdirs(self, path: str):
        """Return list of subdirectory names at path. Raises InvalidPath or ResourceNotExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def listfiles(self, path: str):
        """Return list of file names at path. Raises InvalidPath or ResourceNotExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def list_formatted(self, path: str, base_group: str):
        """Return formatted directory listing for the file explorer UI."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def get_base_tree_template(self):
        pass

    @abstractmethod
    def get_base_subtree_template(self):
        pass

    @abstractmethod
    def get_action_template(self, filename, template):
        pass

    @abstractmethod
    def get_world_template(self, world):
        pass

    @abstractmethod
    def mkdir(self, path: str):
        """Create a new directory at path. Raises InvalidPath or ResourceAlreadyExists."""
        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) >= 0:
            raise ResourceAlreadyExists(path)

    @abstractmethod
    def renamefile(self, old_path: str, new_path: str):
        """Rename a file from old path to new path. Raises InvalidPath, ResourceNotExists, or ResourceAlreadyExists."""
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def renamedir(self, old_path: str, new_path: str):
        """Rename a directory from old path to new path. Raises InvalidPath, ResourceNotExists, or ResourceAlreadyExists."""
        if ".." in new_path:
            raise InvalidPath(new_path)

        if self.exists(old_path) < 0:
            raise ResourceNotExists(old_path)

        if self.exists(new_path) >= 0:
            raise ResourceAlreadyExists(new_path)

    @abstractmethod
    def removefile(self, path: str):
        """Delete a file. Raises InvalidPath or ResourceNotExists."""

        if ".." in path:
            raise InvalidPath(path)

        size = self.exists(path)
        if size < 0:
            raise ResourceNotExists(path)

        if not self.isfile(path):
            raise ResourceNotExists(path)
        if self.project is not None:
            self.project.update_size(self, 0, size)
        self.user.update_size(self, 0, size, project_callback=get_user_projects_size)

    @abstractmethod
    def removedir(self, path: str):
        """Delete a directory and all its contents. Raises InvalidPath or ResourceNotExists."""

        if ".." in path:
            raise InvalidPath(path)

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    @abstractmethod
    def dir_size(self, path):
        """Return total size in bytes of all files under path. Raises InvalidPath or ResourceNotExists."""

        if ".." in path:
            raise InvalidPath(path)

        path = self.path_join(path, "")

        if self.exists(path) < 0:
            raise ResourceNotExists(path)

        if not self.isdir(path):
            raise ResourceNotExists(path)

    def filename(self, path: str) -> str:
        return os.path.splitext(os.path.basename(path))[0]


class FAL_BT(FAL):
    """File Abstraction Layer"""

    def __init__(self, projects=""):
        super().__init__(projects)

    def projects_path(self) -> str:
        return self.path_join(self.projects, "filesystem")

    def library_path(self) -> str:
        return self.path_join(self.projects, "library")

    def path_join(self, a: str, b: str) -> str:
        return os.path.join(a, b)

    def exists(self, path: str) -> int:
        """Returns -1 if path does not exist, 0 if it is a directory, file's size if it is an existing file."""
        if not os.path.exists(path):
            return -1

        if os.path.isdir(path):  # It is a dir
            return 0

        return os.path.getsize(path)

    def isdir(self, path: str) -> bool:
        return os.path.isdir(path)

    def isfile(self, path: str) -> bool:
        return os.path.isfile(path)

    def create(self, path: str, content):
        super().create(path, content)

        with open(path, "w") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def create_binary(self, path: str, content):
        super().create_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def write(self, path: str, content):
        super().write(path, content)

        with open(path, "w") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def write_binary(self, path: str, content):
        super().write_binary(path, content)

        with open(path, "wb") as f:
            f.write(content)
        os.chmod(path, 0o777)

    def read(self, path: str) -> str:
        super().read(path)

        try:
            with open(path, "r") as f:
                return f.read()
        except Exception:
            raise BinaryNotSupported(path)

    def read_binary(self, path: str) -> bytes:
        super().read_binary(path)

        with open(path, "rb") as f:
            return f.read()

    def listdirs(self, path: str):
        super().listdirs(path)

        return [d for d in os.listdir(path) if self.isdir(self.path_join(path, d))]

    def listfiles(self, path: str):
        super().listfiles(path)

        return [d for d in os.listdir(path) if self.isfile(self.path_join(path, d))]

    def list_formatted(self, path: str, base_group: str):
        super().list_formatted(path, base_group)

        return list_dir(path, path, base_group=base_group)

    def get_base_tree_template(self):
        init_graph_path = self.path_join(self.projects, "templates/graph.json")
        return self.read(init_graph_path)

    def get_base_subtree_template(self):
        init_graph_path = self.path_join(self.projects, "templates/graph.json")
        return self.read(init_graph_path)

    def get_action_template(self, filename, template):
        templates_folder_path = self.path_join(self.projects, "templates")
        template_path = self.path_join(templates_folder_path, template)
        file_data = self.read(template_path)
        new_data = file_data.replace("ACTION", filename)
        return new_data

    def get_world_template(self, world):
        contents = []
        templates_folder_path = self.path_join(self.projects, "templates/world")
        launch_path = self.path_join(templates_folder_path, "launch/scene.launch.py")
        world_path = self.path_join(templates_folder_path, "scene/scene.world")
        cmake_path = self.path_join(templates_folder_path, "CMakeLists.txt")
        vis_config_path = self.path_join(templates_folder_path, "gz.config")
        package_path = self.path_join(templates_folder_path, "package.xml")

        file_data = self.read(launch_path)
        new_data = file_data.replace("REPLACE", world)
        contents.append({"path": "launch/scene.launch.py", "content": new_data})

        file_data = self.read(world_path)
        contents.append({"path": "scene/scene.world", "content": file_data})

        file_data = self.read(vis_config_path)
        contents.append({"path": "gz.config", "content": file_data})

        file_data = self.read(cmake_path)
        new_data = file_data.replace("REPLACE", world)
        contents.append({"path": "CMakeLists.txt", "content": new_data})

        file_data = self.read(package_path)
        new_data = file_data.replace("REPLACE", world)
        contents.append({"path": "package.xml", "content": new_data})
        return contents

    def mkdir(self, path: str):
        super().mkdir(path)

        os.makedirs(path)
        os.chmod(path, mode=0o777)

    def renamefile(self, old_path: str, new_path: str):
        super().renamefile(old_path, new_path)

        os.rename(old_path, new_path)

    def renamedir(self, old_path: str, new_path: str):
        super().renamedir(old_path, new_path)

        os.rename(old_path, new_path)

    def removefile(self, path: str):
        super().removefile(path)
        os.remove(path)

    def removedir(self, path: str):
        super().removedir(path)
        size = self.dir_size(path)
        shutil.rmtree(path)

        self.project.update_size(self, 0, size)
        self.user.update_size(self, 0, size, project_callback=get_user_projects_size)

    def dir_size(self, path):
        super().dir_size(path)

        total_size = 0
        for dirpath, dirnames, filenames in os.walk(path):
            for f in filenames:
                fp = os.path.join(dirpath, f)
                # skip if it is symbolic link
                if not os.path.islink(fp):
                    total_size += os.path.getsize(fp)

        return total_size
