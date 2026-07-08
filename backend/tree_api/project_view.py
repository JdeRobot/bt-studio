"""Utilities for project file tree representation."""

from json import JSONEncoder
import mimetypes
import chardet
import os


class Entry:
    """Represents a file or directory node."""

    def __init__(
        self,
        is_dir=False,
        name="",
        path="/",
        group="",
        access=True,
        files=[],
    ):
        """Initialize an Entry.

        Args:
            is_dir: Boolean indicating directory status.
            name: Name of the entry.
            path: Relative path.
            group: Group identifier.
            access: Boolean access flag.
            files: List of children if directory.
        """
        self.is_dir = is_dir
        self.name = name
        self.path = path
        self.group = group
        self.access = access
        self.files = files
        self.binary = is_binary_mimetype(name)

    def __str__(self):
        """Return the string representation of the entry."""
        if self.is_dir:
            return self.name + " [%s]" % (", ".join(map(str, self.files)))
        else:
            return self.name


class EntryEncoder(JSONEncoder):
    """JSON encoder for Entry objects."""

    def default(self, o):
        """Return the dictionary representation of the object."""
        return o.__dict__


def is_binary_mimetype(file_path):
    """Check if a file is binary based on mimetype.

    Args:
        file_path: Path or filename to check.

    Returns:
        True if binary, False otherwise.
    """
    mime_type, _ = mimetypes.guess_type(file_path)
    return mime_type is None or mime_type.startswith(
        ("application/", "image/", "video/", "audio/")
    )


def list_dir(base_dir, directory, access_old=True, base_group=""):
    """Recursively list directory contents.

    Args:
        base_dir: Root directory for relative paths.
        directory: Target directory to list.
        access_old: Access flag for children.
        base_group: Group label for children.

    Returns:
        A list of Entry objects.
    """
    entries = os.listdir(directory)
    values = []
    for entry in entries:
        access = access_old
        group = base_group
        entry_path = os.path.join(directory, entry)
        rel_path = os.path.relpath(entry_path, base_dir)
        if os.path.isfile(entry_path):
            values.append(
                Entry(
                    False,
                    entry,
                    rel_path,
                    group,
                    access,
                )
            )
        else:
            if entry == "trees":
                group = "Trees"
                # access = False
            elif entry == "Actions":
                group = "Actions"
            values.append(
                Entry(
                    True,
                    entry,
                    rel_path,
                    group,
                    access,
                    list_dir(base_dir, entry_path, access, group),
                )
            )
    values.sort(key=lambda x: (not x.is_dir, x.name.lower()))
    return values
