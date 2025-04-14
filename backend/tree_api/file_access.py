# File Abstraction Layer

import os


class FAL:
    """File Abstraction Layer"""

    def __init__(self):
        pass

    def path_join(a: str, b: str) -> str:
        return os.path.join(a, b)

    def exists(path: str) -> bool:
        return os.path.exists(path)

    def isdir(path: str) -> bool:
        return os.path.isdir(path)

    def isfile(path: str) -> bool:
        return os.path.isfile(path)

    def relpath(path: str, start: str) -> str:
        return os.path.relpath(path, start)
