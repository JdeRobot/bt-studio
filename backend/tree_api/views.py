import glob
import os
from django.conf import settings
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .serializers import FileContentSerializer
from . import app_generator
from . import tree_generator
from . import json_translator
from . import templates
from .models import Universe
from .project_view import list_dir, EntryEncoder
from django.http import HttpResponse
from django.http import JsonResponse
import mimetypes
import json
import shutil
import zipfile
from distutils.dir_util import copy_tree
from rest_framework import status
from django.core.files.storage import default_storage
import base64
import xml.etree.ElementTree as ET
from .file_access import FAL
from .exceptions import ResourceNotExists, ResourceAlreadyExists

# PROJECT MANAGEMENT

CUSTOM_EXCEPTIONS = (ResourceNotExists, ResourceAlreadyExists)

fal = FAL(settings.BASE_DIR)


@api_view(["POST"])
def create_project(request):
    if "project_name" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.data.get("project_name")
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code/actions")
    universes_path = fal.path_join(project_path, "universes")
    config_path = fal.path_join(project_path, "config.json")

    tree_path = fal.path_join(project_path, "code/trees")
    subtree_path = fal.path_join(tree_path, "subtrees")
    init_graph_path = fal.path_join(settings.BASE_DIR, "templates/graph.json")

    # Default cfg values
    default_cfg = {
        "name": project_name,
        "config": {
            "editorShowAccentColors": True,
            "theme": "dark",
            "btOrder": "bottom-to-top",
        },
    }

    if not fal.exists(project_path):

        # Create folders
        os.mkdir(project_path)
        os.makedirs(action_path)
        os.mkdir(universes_path)
        os.mkdir(tree_path)
        os.mkdir(subtree_path)

        # Create default config
        with open(config_path, "w") as cfg:
            json.dump(default_cfg, cfg)

        # Copy default graph from templates
        shutil.copy(init_graph_path, fal.path_join(tree_path, "main.json"))

        return Response(
            {"success": True, "message": "Project created successfully"},
            status=status.HTTP_201_CREATED,
        )
    else:
        return Response(
            {"success": False, "message": "Project already exists"}, status=409
        )


@api_view(["POST"])
def delete_project(request):
    if "project_name" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    project_name = request.data.get("project_name")
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)

    if fal.exists(project_path):
        shutil.rmtree(project_path)
        return Response({"success": True}, status=200)
    else:
        return Response(
            {"success": False, "message": "Project does not exist"}, status=404
        )


@api_view(["GET"])
def get_project_list(request):

    folder_path = fal.base_path()

    try:
        project_list = fal.listdirs(folder_path)
        return Response({"project_list": project_list})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def save_base_tree(request):
    if "project_name" not in request.data or "graph_json" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the app name and the graph
    project_name = request.data.get("project_name")
    graph_json = request.data.get("graph_json")

    # Generate the paths
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    graph_path = fal.path_join(project_path, "code/trees/main.json")

    try:
        # Obtain pretty json
        graph = json.loads(graph_json)
        graph_formated = json.dumps(graph, indent=4)

        with open(graph_path, "w") as f:
            f.write(graph_formated)

        return JsonResponse({"success": True})

    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error deleting file: {str(e)}"},
            status=422,
        )


@api_view(["GET"])
def get_project_graph(request):

    project_name = request.GET.get("project_name")

    if "project_name" not in request.GET:
        return Response(
            {"error": "Project Name is required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Generate the paths
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    graph_path = fal.path_join(project_path, "code/trees/main.json")

    # Check if the project exists
    try:
        graph_data = json.loads(fal.read(graph_path))
        return JsonResponse({"success": True, "graph_json": graph_data})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error reading file: {str(e)}"},
            status=422,
        )


@api_view(["GET"])
def get_project_configuration(request):

    project_name = request.GET.get("project_name")

    if "project_name" not in request.GET:
        return Response(
            {"error": "Project Name is required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    folder_path = fal.base_path()

    project_path = fal.path_join(folder_path, project_name)
    config_path = fal.path_join(project_path, "config.json")
    try:
        content = fal.read(config_path)
        return Response(content)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)


@api_view(["GET"])
def get_tree_structure(request):

    project_name = request.GET.get("project_name")
    bt_order = request.GET.get("bt_order")

    if "project_name" not in request.GET or "bt_order" not in request.GET:
        return Response(
            {"error": "Project Name is required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Generate the paths
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    graph_path = fal.path_join(project_path, "code/trees/main.json")

    # Check if the project exists
    try:
        graph_data = json.loads(fal.read(graph_path))
        # Get the tree structure
        tree_structure = json_translator.translate_tree_structure(graph_data, bt_order)
        return JsonResponse({"success": True, "tree_structure": tree_structure})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error reading file: {str(e)}"},
            status=500,
        )


@api_view(["GET"])
def get_subtree_structure(request):

    project_name = request.GET.get("project_name")
    subtree_name = request.GET.get("subtree_name")
    bt_order = request.GET.get("bt_order")

    if (
        "project_name" not in request.GET
        or "subtree_name" not in request.GET
        or "bt_order" not in request.GET
    ):
        return Response(
            {"error": "Project Name is required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Generate the paths
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    subtree_path = fal.path_join(project_path, "code/trees/subtrees")
    graph_path = fal.path_join(subtree_path, subtree_name + ".json")

    # Check if the project exists
    try:
        graph_data = json.loads(fal.read(graph_path))
        # Get the tree structure
        tree_structure = json_translator.translate_tree_structure(graph_data, bt_order)
        return JsonResponse({"success": True, "tree_structure": tree_structure})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error reading file: {str(e)}"},
            status=500,
        )


@api_view(["POST"])
def save_project_configuration(request):
    if "project_name" not in request.data or "settings" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.data.get("project_name")
    content = request.data.get("settings")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    config_path = fal.path_join(project_path, "config.json")

    if content is None or len(content) == 0:
        return Response(
            {"success": False, "message": "Settings are missing"}, status=400
        )
    try:

        d = json.loads(content)

        with open(config_path, "w") as f:
            json.dump(d, f, indent=4)

        return Response({"success": True})
    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=422)


# SUBTREE MANAGEMENT


@api_view(["POST"])
def create_subtree(request):
    if "project_name" not in request.data or "subtree_name" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.data.get("project_name")
    subtree_name = request.data.get("subtree_name")

    if not project_name:
        return Response(
            {"success": False, "message": "Project parameter is missing"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    if not subtree_name:
        return Response(
            {"success": False, "message": "Subtree parameter is missing"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    project_actions_path = fal.path_join(project_path, "code", "actions")
    library_path = fal.path_join(settings.BASE_DIR, "library", subtree_name)
    library_actions_path = fal.path_join(library_path, "actions")
    template_path = fal.path_join(settings.BASE_DIR, "templates")
    src_path = template_path
    project_subtree_path = fal.path_join(project_path, "code", "trees", "subtrees")

    # Check if the subtree is already implemented on the library
    if fal.exists(library_path):
        src_path = library_path
        if fal.exists(library_actions_path):
            shutil.copytree(
                library_actions_path, project_actions_path, dirs_exist_ok=True
            )

    # Setup init and copy paths
    init_json_path = fal.path_join(src_path, "graph.json")
    project_json_path = fal.path_join(project_subtree_path, f"{subtree_name}.json")

    # Create subtree directory if it does not exist
    if not fal.exists(project_subtree_path):
        os.mkdir(project_subtree_path)

    # Copy the subtree to the project
    if not fal.exists(project_json_path):
        shutil.copy(init_json_path, project_json_path)
        return Response({"success": True}, status=status.HTTP_201_CREATED)
    else:
        return Response(
            {"success": False, "message": "Subtree already exists"},
            status=409,
        )


@api_view(["POST"])
def save_subtree(request):

    # Check if 'project_name', 'subtree_name', and 'subtree_json' are in the request data
    if (
        "project_name" not in request.data
        or "subtree_name" not in request.data
        or "subtree_json" not in request.data
    ):
        return Response(
            {"success": False, "message": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the project name, subtree name, and subtree JSON
    project_name = request.data.get("project_name")
    subtree_name = request.data.get("subtree_name")
    subtree_json = request.data.get("subtree_json")

    # Generate the paths
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    json_path = fal.path_join(
        project_path, "code", "trees", "subtrees", f"{subtree_name}.json"
    )

    try:
        # Write the subtree JSON to the file
        with open(json_path, "w") as f:
            f.write(subtree_json)

        return JsonResponse({"success": True}, status=status.HTTP_200_OK)

    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error saving subtree: {str(e)}"},
            status=status.HTTP_500_INTERNAL_SERVER_ERROR,
        )


@api_view(["GET"])
def get_subtree(request):
    if "project_name" not in request.GET or "subtree_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    subtree_name = request.GET.get("subtree_name")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    subtree_path = fal.path_join(
        project_path, "code/trees/subtrees", f"{subtree_name}.json"
    )

    try:
        subtree = json.loads(fal.read(subtree_path))
        return Response({"subtree": subtree}, status=status.HTTP_200_OK)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)


@api_view(["GET"])
def get_subtree_list(request):

    if "project_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    tree_path = fal.path_join(project_path, "code", "trees", "subtrees")

    try:
        # List all files in the directory removing the .json extension
        subtree_list = fal.listfiles(tree_path)
        subtree_list = [f.split(".")[0] for f in subtree_list]
        return Response({"subtree_list": subtree_list})

    except Exception as e:
        return Response({"subtree_list": []})


# UNIVERSE MANAGEMENT


@api_view(["POST"])
def delete_universe(request):
    if "project_name" not in request.data or "universe_name" not in request.data:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )
    project_name = request.data.get("project_name")
    universe_name = request.data.get("universe_name")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    universes_path = fal.path_join(project_path, "universes/")
    universe_path = fal.path_join(universes_path, universe_name)

    if fal.exists(universe_path):
        shutil.rmtree(universe_path)
        return Response({"success": True})
    else:
        return Response(
            {"success": False, "message": "Project does not exist"}, status=404
        )


@api_view(["GET"])
def get_universes_list(request):

    if "project_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    universes_path = fal.path_join(project_path, "universes/")

    try:
        universes_list = fal.listdirs(universes_path)
        return Response({"universes_list": universes_list})
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=404)


@api_view(["GET"])
def get_universe_configuration(request):
    if "project_name" not in request.GET or "universe_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    universe_name = request.GET.get("universe_name")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    universes_path = fal.path_join(project_path, "universes/")

    universe_path = fal.path_join(universes_path, universe_name)
    config_path = fal.path_join(universe_path, "config.json")

    try:
        content = json.loads(fal.read(config_path))
        return Response(
            {"success": True, "config": content}, status=200
        )  # Return as JSON
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except json.JSONDecodeError:
        return Response(
            {"success": False, "message": "Invalid JSON format in config file"},
            status=422,
        )


# FILE MANAGEMENT


@api_view(["GET"])
def get_file_list(request):
    if "project_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")

    try:
        file_list = fal.list_formatted(action_path)

        # Return the list of files
        return Response({"file_list": EntryEncoder().encode(file_list)})

    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=404)


@api_view(["GET"])
def get_actions_list(request):
    if "project_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code/actions")

    try:
        actions_list = fal.listfiles(action_path)
        return Response({"actions_list": actions_list})
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=404)


@api_view(["GET"])
def get_file(request):
    if "project_name" not in request.GET or "filename" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name", None)
    filename = request.GET.get("filename", None)

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")

    file_path = fal.path_join(action_path, filename)
    try:
        content = fal.read(file_path)
        serializer = FileContentSerializer({"content": content})
        return Response(serializer.data)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)


@api_view(["POST"])
def create_action(request):
    if (
        "project_name" not in request.data
        or "template" not in request.data
        or "filename" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the file info
    project_name = request.data.get("project_name")
    filename = request.data.get("filename")
    template = request.data.get("template")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code/actions")
    file_path = fal.path_join(action_path, filename + ".py")

    templates_folder_path = fal.path_join(settings.BASE_DIR, "templates")
    template_path = fal.path_join(templates_folder_path, template)

    if not fal.exists(file_path):
        try:
            with open(file_path, "w") as f:
                f.write(
                    templates.get_action_template(filename, template, template_path)
                )
            return Response({"success": True})
        except:
            return Response(
                {"success": False, "message": "Template does not exist"}, status=400
            )
    else:
        return Response(
            {"success": False, "message": "File already exists"}, status=409
        )


@api_view(["POST"])
def create_file(request):
    if (
        "project_name" not in request.data
        or "location" not in request.data
        or "file_name" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the file info
    project_name = request.data.get("project_name")
    location = request.data.get("location")
    filename = request.data.get("file_name")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    create_path = fal.path_join(action_path, location)
    file_path = fal.path_join(create_path, filename)

    if not fal.exists(file_path):
        with open(file_path, "w") as f:
            f.write("")
        return Response({"success": True})
    else:
        return Response(
            {"success": False, "message": "File already exists"}, status=409
        )


@api_view(["POST"])
def create_folder(request):
    if (
        "project_name" not in request.data
        or "location" not in request.data
        or "folder_name" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the file info
    project_name = request.data.get("project_name")
    location = request.data.get("location")
    folder_name = request.data.get("folder_name")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    create_path = fal.path_join(action_path, location)
    folder_path = fal.path_join(create_path, folder_name)

    if not fal.exists(folder_path):
        try:
            os.makedirs(folder_path)
            return Response({"success": True})
        except Exception as e:
            return Response({"success": False, "message": "Invalid name"}, status=400)
    else:
        return Response(
            {"success": False, "message": "Folder already exists"}, status=409
        )


@api_view(["POST"])
def rename_file(request):
    if (
        "project_name" not in request.data
        or "path" not in request.data
        or "rename_to" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the file info
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    file_path = fal.path_join(action_path, path)
    new_path = fal.path_join(action_path, rename_path)

    if fal.exists(file_path):
        try:
            os.rename(file_path, new_path)
            return JsonResponse({"success": True})
        except Exception as e:
            return JsonResponse(
                {"success": False, "message": f"Error deleting file: {str(e)}"},
                status=500,
            )
    else:
        return JsonResponse(
            {"success": False, "message": "File does not exist"}, status=404
        )


@api_view(["POST"])
def rename_folder(request):
    if (
        "project_name" not in request.data
        or "path" not in request.data
        or "rename_to" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )
    # Get the folder info
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    file_path = fal.path_join(action_path, path)
    new_path = fal.path_join(action_path, rename_path)

    if fal.exists(file_path):
        try:
            os.rename(file_path, new_path)
            return JsonResponse({"success": True})
        except Exception as e:
            return JsonResponse(
                {"success": False, "message": f"Error deleting folder: {str(e)}"},
                status=500,
            )
    else:
        return JsonResponse(
            {"success": False, "message": "File does not exist"}, status=404
        )


@api_view(["POST"])
def delete_file(request):
    if "project_name" not in request.data or "path" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the file info
    project_name = request.data.get("project_name")
    path = request.data.get("path")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    file_path = fal.path_join(action_path, path)

    if fal.exists(file_path) and not fal.isdir(file_path):
        try:
            os.remove(file_path)
            return JsonResponse({"success": True})
        except Exception as e:
            return JsonResponse(
                {"success": False, "message": f"Error deleting file: {str(e)}"},
                status=500,
            )
    else:
        return JsonResponse(
            {"success": False, "message": "File does not exist"}, status=404
        )


@api_view(["POST"])
def delete_folder(request):
    if "project_name" not in request.data or "path" not in request.data:
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the folder info
    project_name = request.data.get("project_name")
    path = request.data.get("path")

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    file_path = fal.path_join(action_path, path)

    if fal.exists(file_path) and fal.isdir(file_path):
        try:
            shutil.rmtree(file_path)
            return JsonResponse({"success": True})
        except Exception as e:
            return JsonResponse(
                {"success": False, "message": f"Error deleting file: {str(e)}"},
                status=500,
            )
    else:
        return JsonResponse(
            {"success": False, "message": "File does not exist"}, status=404
        )


@api_view(["POST"])
def save_file(request):
    if (
        "project_name" not in request.data
        or "filename" not in request.data
        or "content" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.data.get("project_name")
    filename = request.data.get("filename")
    content = request.data.get("content")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    action_path = fal.path_join(project_path, "code")
    file_path = fal.path_join(action_path, filename)

    # If file doesn't exist simply return
    if not fal.exists(file_path):
        return JsonResponse(
            {"success": False, "message": "File does not exist"}, status=404
        )

    try:
        with open(file_path, "w") as f:
            f.write(content)
        return Response({"success": True})
    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=400)


@api_view(["POST"])
def generate_local_app(request):
    # Check if 'app_name', 'main_tree_graph', and 'bt_order' are in the request data
    if "app_name" not in request.data or "bt_order" not in request.data:
        return Response(
            {"success": False, "message": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the request parameters
    app_name = request.data.get("app_name")
    bt_order = request.data.get("bt_order")

    # Make folder path relative to Django app
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, app_name)
    action_path = fal.path_join(project_path, "code/actions")
    tree_path = fal.path_join(project_path, "code/trees/main.json")
    subtree_path = fal.path_join(project_path, "code/trees/subtrees")

    subtrees = []
    actions = []

    try:

        # Check if the project exists
        graph_data = fal.read(tree_path)
        # 1. Generate a basic tree from the JSON definition
        main_tree = json_translator.translate_raw(graph_data, bt_order)

        # 2. Get all possible subtrees name and content
        try:
            subtrees_list = os.listdir(subtree_path)
            subtrees_list.sort()
            for subtree_file in subtrees_list:
                if subtree_file.endswith(".json"):
                    subtree_name = os.path.splitext(os.path.basename(subtree_file))[0]
                    path = fal.path_join(subtree_path, subtree_file)
                    subtree_json = fal.read(path)

                    subtree = json_translator.translate_raw(subtree_json, bt_order)
                    subtrees.append({"name": subtree_name, "content": subtree})
        except:
            print("No subtrees")

        # 3. Get all possible actions name and content
        actions_list = os.listdir(action_path)
        actions_list.sort()
        for action_file in actions_list:
            if action_file.endswith(".py"):
                action_name = os.path.splitext(os.path.basename(action_file))[0]
                path = fal.path_join(action_path, action_file)
                action_content = fal.read(path)

                actions.append({"name": action_name, "content": action_content})

        # 4. Generate a self-contained tree
        final_tree = tree_generator.generate(main_tree, subtrees, actions)

        unique_imports = app_generator.get_unique_imports(actions)

        return JsonResponse(
            {
                "success": True,
                "tree": final_tree,
                "dependencies": sorted(unique_imports),
            }
        )
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response(
            {"success": False, "message": str(e)},
            status=status.HTTP_500_INTERNAL_SERVER_ERROR,
        )


@api_view(["POST"])
def generate_dockerized_app(request):
    # Check if 'app_name', 'tree_graph', and 'bt_order' are in the request data
    if "app_name" not in request.data or "bt_order" not in request.data:
        return Response(
            {"success": False, "message": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the request parameters
    app_name = request.data.get("app_name")
    bt_order = request.data.get("bt_order")

    # Make folder path relative to Django app
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, app_name)
    action_path = fal.path_join(project_path, "code/actions")
    tree_path = fal.path_join(project_path, "code/trees/main.json")
    subtree_path = fal.path_join(project_path, "code/trees/subtrees")

    subtrees = []
    actions = []

    try:

        # Check if the project exists
        graph_data = fal.read(tree_path)

        # 1. Generate a basic tree from the JSON definition
        main_tree = json_translator.translate_raw(graph_data, bt_order)

        # 2. Get all possible subtrees name and content
        try:
            for subtree_file in os.listdir(subtree_path):
                if subtree_file.endswith(".json"):
                    subtree_name = os.path.splitext(os.path.basename(subtree_file))[0]
                    path = fal.path_join(subtree_path, subtree_file)
                    subtree_json = fal.read(path)

                    subtree = json_translator.translate_raw(subtree_json, bt_order)
                    subtrees.append({"name": subtree_name, "content": subtree})
        except:
            print("No subtrees")

        # 3. Get all possible actions name and content
        for action_file in os.listdir(action_path):
            if action_file.endswith(".py"):
                action_name = os.path.splitext(os.path.basename(action_file))[0]
                path = fal.path_join(action_path, action_file)
                action_content = fal.read(path)

                actions.append({"name": action_name, "content": action_content})

        # 4. Generate a self-contained tree
        final_tree = tree_generator.generate(main_tree, subtrees, actions)

        # 6. Return the files as a response
        return JsonResponse({"success": True, "tree": final_tree})

    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response(
            {"success": False, "message": str(e)},
            status=status.HTTP_500_INTERNAL_SERVER_ERROR,
        )


@api_view(["GET"])
def get_universe_file_list(request):
    if "project_name" not in request.GET or "universe_name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name")
    universe_name = request.GET.get("universe_name")

    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    universes_path = fal.path_join(project_path, "universes")
    universe_path = fal.path_join(universes_path, universe_name)

    try:
        file_list = fal.list_formatted(universe_path)

        # Return the list of files
        return Response({"file_list": EntryEncoder().encode(file_list)})

    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=404)


@api_view(["GET"])
def get_universe_file(request):
    if (
        "project_name" not in request.GET
        or "universe_name" not in request.GET
        or "filename" not in request.GET
    ):
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    project_name = request.GET.get("project_name", None)
    universe_name = request.GET.get("universe_name")
    filename = request.GET.get("filename", None)

    # Make folder path relative to Django app
    folder_path = fal.base_path()
    project_path = fal.path_join(folder_path, project_name)
    universes_path = fal.path_join(project_path, "universes")
    universe_path = fal.path_join(universes_path, universe_name)

    file_path = fal.path_join(universe_path, filename)
    try:
        content = fal.read(file_path)
        serializer = FileContentSerializer({"content": content})
        return Response(serializer.data)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)


@api_view(["POST"])
def upload_universe(request):

    # Check if 'name' and 'zipfile' are in the request data
    if (
        "universe_name" not in request.data
        or "app_name" not in request.data
        or "zip_file" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the name and the zip file from the request
    universe_name = request.data.get("universe_name")
    app_name = request.data.get("app_name")
    zip_file = request.data.get("zip_file")

    # Make folder path relative to Django app
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, app_name)
    universes_path = fal.path_join(project_path, "universes")
    universe_path = fal.path_join(universes_path, universe_name)

    # Create the folder if it doesn't exist
    if not fal.exists(universe_path):
        os.makedirs(universe_path)

    try:
        zip_file_data = base64.b64decode(zip_file)
    except (TypeError, ValueError):
        return Response({"error": "Invalid zip file data."}, status=422)

    # Save the zip file temporarily
    temp_zip_path = fal.path_join(universe_path, "temp.zip")
    with open(temp_zip_path, "wb") as temp_zip_file:
        temp_zip_file.write(zip_file_data)

    # Unzip the file
    try:
        with zipfile.ZipFile(temp_zip_path, "r") as zip_ref:
            zip_ref.extractall(universe_path)
    except zipfile.BadZipFile:
        return Response(
            {"error": "Invalid zip file."}, status=status.HTTP_400_BAD_REQUEST
        )
    finally:
        # Clean up the temporary zip file
        if fal.exists(temp_zip_path):
            os.remove(temp_zip_path)

    # Fill the config dictionary of the universe
    ram_launch_path = "/workspace/worlds/" + universe_name + "/universe.launch.py"
    universe_config = {
        "name": universe_name,
        "type": "custom",
        "ram_config": {
            "ros_version": "ROS2",
            "world": "gazebo",
            "launch_file_path": ram_launch_path,
        },
    }

    # Generate the json config
    config_path = fal.path_join(universe_path, "config.json")
    with open(config_path, "w") as config_file:
        json.dump(universe_config, config_file, ensure_ascii=False, indent=4)

    return Response(
        {"success": True, "message": "Universe uploaded successfully"},
        status=status.HTTP_200_OK,
    )


@api_view(["POST"])
def add_docker_universe(request):

    # Check if 'universe_name', 'app_name' and 'id' are in the request data
    if (
        "universe_name" not in request.data
        or "app_name" not in request.data
        or "id" not in request.data
    ):
        return Response(
            {"error": "Name and id are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the name and the id file from the request
    universe_name = request.data.get("universe_name")
    app_name = request.data.get("app_name")
    id = request.data.get("id")

    # Make folder path relative to Django app
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, app_name)
    universes_path = fal.path_join(project_path, "universes")
    universe_path = fal.path_join(universes_path, universe_name)

    # Create the folder if it doesn't exist
    if not fal.exists(universe_path):
        os.makedirs(universe_path)

    # Fill the config dictionary of the universe
    universe_config = {"name": universe_name, "id": id, "type": "robotics_backend"}

    # Generate the json config
    config_path = fal.path_join(universe_path, "config.json")
    with open(config_path, "w") as config_file:
        json.dump(universe_config, config_file, ensure_ascii=False, indent=4)

    return Response(
        {"success": True, "message": "Universe uploaded successfully"},
        status=status.HTTP_200_OK,
    )


@api_view(["POST"])
def upload_code(request):

    # Check if 'name' and 'zipfile' are in the request data
    if (
        "project_name" not in request.data
        or "file_name" not in request.data
        or "location" not in request.data
        or "content" not in request.data
    ):
        return Response(
            {"error": "Name and zip file are required."},
            status=status.HTTP_400_BAD_REQUEST,
        )

    # Get the name and the zip file from the request
    project_name = request.data.get("project_name")
    file_name = request.data.get("file_name")
    location = request.data.get("location")
    content = request.data.get("content")

    # Make folder path relative to Django app
    base_path = fal.base_path()
    project_path = fal.path_join(base_path, project_name)
    code_path = fal.path_join(project_path, "code")
    create_path = fal.path_join(code_path, location)
    file_path = fal.path_join(create_path, file_name)

    # If file exist simply return
    if fal.exists(file_path):
        return JsonResponse(
            {"success": False, "message": "File already exists"}, status=409
        )

    try:
        with open(file_path, "wb") as f:
            f.write(base64.b64decode(content))
        return Response({"success": True})
    except Exception as e:
        return Response({"success": False, "message": str(e)}, status=422)


@api_view(["GET"])
def list_docker_universes(request):
    try:
        universes = Universe.objects.all()
        universes_docker_list = [x.name for x in universes]
        # Return the list of projects
        return Response({"universes": universes_docker_list})

    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_docker_universe_path(request):
    # Check if 'name' is in the request data
    if "name" not in request.GET:
        return Response(
            {"error": "Missing required parameters"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    name = request.GET.get("name")

    try:
        universe = Universe.objects.get(name=name)

        config = {
            "name": universe.name,
            "world": {
                "name": universe.world.name,
                "launch_file_path": universe.world.launch_file_path,
                "ros_version": universe.world.ros_version,
                "world": universe.world.world,
            },
            "robot": {
                "name": universe.robot.name,
                "launch_file_path": universe.robot.launch_file_path,
                "ros_version": universe.world.ros_version,
                "world": universe.world.world,
                "start_pose": universe.world.start_pose,
            },
            "visualization": universe.world.visualization,
            "visualization_config_path": universe.world.visualization_config_path,
        }

        # Return the list of projects
        return Response(
            {
                "success": True,
                "universe": config,
            }
        )

    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)
