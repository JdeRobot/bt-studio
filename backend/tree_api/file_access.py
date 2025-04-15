# File Abstraction Layer

import os


class FAL:
    """File Abstraction Layer"""

    def __init__(self, base):
        self.base = base

    def base_path(self) -> str:
        return self.path_join(self.base, "filesystem")

    def path_join(self, a: str, b: str) -> str:
        return os.path.join(a, b)

    def exists(self, path: str) -> bool:
        return os.path.exists(path)

    def isdir(self, path: str) -> bool:
        return os.path.isdir(path)

    def isfile(self, path: str) -> bool:
        return os.path.isfile(path)

    def relpath(self, path: str, start: str) -> str:
        return os.path.relpath(path, start)
    
    def write(self, path: str, content):
        with open(path, "w") as f:
            f.write(content)

    def write_binary(self, path: str, content):
        with open(path, "wb") as f:
            f.write(content)

    def read(self, path: str) -> str:
        with open(path, "r") as f:
            return f.read()
