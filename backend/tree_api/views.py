from django.conf import settings
from rest_framework.decorators import api_view
from rest_framework.response import Response
from .serializers import FileContentSerializer
from . import app_generator
from . import tree_generator
from . import json_translator
from .models import Universe
from .project_view import EntryEncoder
from django.http import HttpResponse
from django.http import JsonResponse
import json
import shutil
import zipfile
from rest_framework import status
import base64
from .file_access import FAL
from .exceptions import ResourceNotExists, ResourceAlreadyExists, ParameterInvalid

# PROJECT MANAGEMENT

CUSTOM_EXCEPTIONS = (ResourceNotExists, ResourceAlreadyExists, ParameterInvalid)

fal = FAL(settings.BASE_DIR)


def check_post_parameters(request, param: list[str | tuple]):
    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data == None or len(data) <= min_len:
            raise ParameterInvalid(p)


def check_get_parameters(request, param: list[str | tuple]):
    for p in param:
        min_len = 0
        if type(p) is tuple:
            min_len = p[1]
            p = p[0]
        if p not in request:
            raise ParameterInvalid(p)
        data = request.get(p)
        if data == None or len(data) <= min_len:
            raise ParameterInvalid(p)


@api_view(["POST"])
def create_project(request):
    try:
        check_post_parameters(request.data, ["project_name"])

        project_name = request.data.get("project_name")

        project_path = fal.project_path(project_name)
        action_path = fal.actions_path(project_name)
        universes_path = fal.universes_path(project_name)
        tree_path = fal.trees_path(project_name)
        subtree_path = fal.subtrees_path(project_name)

        config_path = fal.path_join(project_path, "config.json")
        base_tree_path = fal.path_join(tree_path, "main.json")
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

        # Create folders
        fal.mkdir(project_path)
        fal.mkdir(action_path)
        fal.mkdir(universes_path)
        fal.mkdir(tree_path)
        fal.mkdir(subtree_path)

        # Create default config
        config_formated = json.dumps(default_cfg, indent=4)
        fal.create(config_path, config_formated)

        # Copy default graph from templates
        graph_data = fal.read(init_graph_path)
        fal.create(base_tree_path, graph_data)
        return Response(
            {"success": True, "message": "Project created successfully"},
            status=status.HTTP_201_CREATED,
        )
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def delete_project(request):
    try:
        check_post_parameters(request.data, ["project_name"])

        project_name = request.data.get("project_name")

        project_path = fal.project_path(project_name)

        fal.removedir(project_path)
        return Response({"success": True}, status=200)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


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
    try:
        check_post_parameters(request.data, ["project_name", "graph_json"])

        # Get the app name and the graph
        project_name = request.data.get("project_name")
        graph_json = request.data.get("graph_json")

        # Generate the paths
        trees_path = fal.trees_path(project_name)
        graph_path = fal.path_join(trees_path, "main.json")

        # Obtain pretty json
        graph = json.loads(graph_json)
        graph_formated = json.dumps(graph, indent=4)
        fal.write(graph_path, graph_formated)
        return JsonResponse({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return JsonResponse(
            {"success": False, "message": f"Error deleting file: {str(e)}"},
            status=422,
        )


@api_view(["GET"])
def get_project_graph(request):
    try:
        check_get_parameters(request.GET, ["project_name"])
        project_name = request.GET.get("project_name")

        # Generate the paths
        trees_path = fal.trees_path(project_name)
        graph_path = fal.path_join(trees_path, "main.json")

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
    try:
        check_get_parameters(request.GET, ["project_name"])
        project_name = request.GET.get("project_name")

        project_path = fal.project_path(project_name)
        config_path = fal.path_join(project_path, "config.json")
        content = fal.read(config_path)
        return Response(content)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_tree_structure(request):
    try:
        check_get_parameters(request.GET, ["project_name", "bt_order"])
        project_name = request.GET.get("project_name")
        bt_order = request.GET.get("bt_order")

        # Generate the paths
        trees_path = fal.trees_path(project_name)
        graph_path = fal.path_join(trees_path, "main.json")

        # Check if the project exists
        graph_data = json.loads(fal.read(graph_path))
        # Get the tree structure
        tree_structure = json_translator.translate_tree_structure(graph_data, bt_order)
        return JsonResponse({"success": True, "tree_structure": tree_structure})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_subtree_structure(request):
    try:
        check_get_parameters(request.GET, ["project_name", "subtree_name", "bt_order"])
        project_name = request.GET.get("project_name")
        subtree_name = request.GET.get("subtree_name")
        bt_order = request.GET.get("bt_order")

        # Generate the paths
        subtrees_path = fal.subtrees_path(project_name)
        graph_path = fal.path_join(subtrees_path, subtree_name + ".json")

        # Check if the project exists
        graph_data = json.loads(fal.read(graph_path))
        # Get the tree structure
        tree_structure = json_translator.translate_tree_structure(graph_data, bt_order)
        return JsonResponse({"success": True, "tree_structure": tree_structure})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def save_project_configuration(request):
    try:
        check_post_parameters(request.data, ["project_name", "settings"])

        project_name = request.data.get("project_name")
        content = request.data.get("settings")

        project_path = fal.project_path(project_name)
        config_path = fal.path_join(project_path, "config.json")

        if content is None or len(content) == 0:
            return Response(
                {"success": False, "message": "Settings are missing"}, status=400
            )
        graph = json.loads(content)
        graph_formated = json.dumps(graph, indent=4)
        fal.write(config_path, graph_formated)
        return Response({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


# SUBTREE MANAGEMENT


@api_view(["POST"])
def create_subtree(request):
    try:
        check_post_parameters(request.data, ["project_name", "subtree_name"])

        project_name = request.data.get("project_name")
        subtree_name = request.data.get("subtree_name")

        project_actions_path = fal.actions_path(project_name)
        project_subtree_path = fal.subtrees_path(project_name)

        library_base_path = fal.path_join(settings.BASE_DIR, "library")
        library_path = fal.path_join(library_base_path, subtree_name)
        library_actions_path = fal.path_join(library_path, "actions")
        template_path = fal.path_join(settings.BASE_DIR, "templates")
        src_path = template_path

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
            fal.mkdir(project_subtree_path)

        subtree = json.loads(fal.read(init_json_path))
        subtree_formated = json.dumps(subtree, indent=4)
        fal.create(project_json_path, subtree_formated)
        return JsonResponse({"success": True}, status=status.HTTP_201_CREATED)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def save_subtree(request):
    try:
        check_post_parameters(
            request.data, ["project_name", "subtree_name", "subtree_json"]
        )

        # Get the project name, subtree name, and subtree JSON
        project_name = request.data.get("project_name")
        subtree_name = request.data.get("subtree_name")
        subtree_json = request.data.get("subtree_json")

        # Generate the paths
        subtrees_path = fal.subtrees_path(project_name)
        subtree_path = fal.path_join(subtrees_path, f"{subtree_name}.json")

        fal.write(subtree_path, subtree_json)
        return JsonResponse({"success": True}, status=status.HTTP_200_OK)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_subtree(request):
    try:
        check_get_parameters(request.GET, ["project_name", "subtree_name"])

        project_name = request.GET.get("project_name")
        subtree_name = request.GET.get("subtree_name")

        subtrees_path = fal.subtrees_path(project_name)
        subtree_path = fal.path_join(subtrees_path, f"{subtree_name}.json")

        subtree = json.loads(fal.read(subtree_path))
        return Response({"subtree": subtree}, status=status.HTTP_200_OK)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_subtree_list(request):
    try:
        check_get_parameters(request.GET, ["project_name"])

        project_name = request.GET.get("project_name")

        subtrees_path = fal.subtrees_path(project_name)

        # List all files in the directory removing the .json extension
        subtree_list = fal.listfiles(subtrees_path)
        subtree_list = [f.split(".")[0] for f in subtree_list]
        return Response({"subtree_list": subtree_list})

    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


# UNIVERSE MANAGEMENT


@api_view(["POST"])
def delete_universe(request):
    try:
        check_post_parameters(request.data, ["project_name", "universe_name"])
        project_name = request.data.get("project_name")
        universe_name = request.data.get("universe_name")

        universes_path = fal.universes_path(project_name)
        universe_path = fal.path_join(universes_path, universe_name)

        fal.removedir(universe_path)
        return Response({"success": True}, status=200)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_universes_list(request):
    try:
        check_get_parameters(request.GET, ["project_name"])

        project_name = request.GET.get("project_name")

        universes_path = fal.universes_path(project_name)

        universes_list = fal.listdirs(universes_path)
        return Response({"universes_list": universes_list})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_universe_configuration(request):
    try:
        check_get_parameters(request.GET, ["project_name", "universe_name"])

        project_name = request.GET.get("project_name")
        universe_name = request.GET.get("universe_name")

        universes_path = fal.universes_path(project_name)
        universe_path = fal.path_join(universes_path, universe_name)
        config_path = fal.path_join(universe_path, "config.json")

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
    try:
        check_get_parameters(request.GET, ["project_name"])

        project_name = request.GET.get("project_name")

        code_path = fal.code_path(project_name)

        file_list = fal.list_formatted(code_path)

        # Return the list of files
        return Response({"file_list": EntryEncoder().encode(file_list)})

    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_actions_list(request):
    try:
        check_get_parameters(request.GET, ["project_name"])

        project_name = request.GET.get("project_name")

        action_path = fal.actions_path(project_name)

        actions_list = fal.listfiles(action_path)
        return Response({"actions_list": actions_list})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_file(request):
    try:
        check_get_parameters(request.GET, ["project_name", "filename"])

        project_name = request.GET.get("project_name", None)
        filename = request.GET.get("filename", None)

        code_path = fal.code_path(project_name)

        file_path = fal.path_join(code_path, filename)
        content = fal.read(file_path)
        serializer = FileContentSerializer({"content": content})
        return Response(serializer.data)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def create_action(request):
    try:
        check_post_parameters(request.data, ["project_name", "filename", "template"])

        # Get the file info
        project_name = request.data.get("project_name")
        filename = request.data.get("filename")
        template = request.data.get("template")

        # Make folder path relative to Django app
        action_path = fal.actions_path(project_name)
        file_path = fal.path_join(action_path, filename + ".py")

        content = fal.get_action_template(filename, template)
        fal.create(file_path, content)
        return JsonResponse({"success": True}, status=status.HTTP_200_OK)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def create_file(request):
    try:
        check_post_parameters(
            request.data, ["project_name", ("location", -1), "file_name"]
        )
        # Get the file info
        project_name = request.data.get("project_name")
        location = request.data.get("location")
        filename = request.data.get("file_name")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        create_path = fal.path_join(code_path, location)
        file_path = fal.path_join(create_path, filename)

        fal.create(file_path, "")
        return Response({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def create_folder(request):
    try:
        check_post_parameters(
            request.data, ["project_name", ("location", -1), "folder_name"]
        )
        # Get the file info
        project_name = request.data.get("project_name")
        location = request.data.get("location")
        folder_name = request.data.get("folder_name")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        create_path = fal.path_join(code_path, location)
        folder_path = fal.path_join(create_path, folder_name)

        fal.mkdir(folder_path)
        return Response({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def rename_file(request):
    try:
        check_post_parameters(request.data, ["project_name", "path", "rename_to"])
        # Get the file info
        project_name = request.data.get("project_name")
        path = request.data.get("path")
        rename_path = request.data.get("rename_to")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        file_path = fal.path_join(code_path, path)
        new_path = fal.path_join(code_path, rename_path)

        fal.renamefile(file_path, new_path)
        return JsonResponse({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def rename_folder(request):
    try:
        check_post_parameters(request.data, ["project_name", "path", "rename_to"])
        # Get the folder info
        project_name = request.data.get("project_name")
        path = request.data.get("path")
        rename_path = request.data.get("rename_to")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        file_path = fal.path_join(code_path, path)
        new_path = fal.path_join(code_path, rename_path)

        fal.renamedir(file_path, new_path)
        return JsonResponse({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def delete_file(request):
    try:
        check_post_parameters(request.data, ["project_name", "path"])
        # Get the file info
        project_name = request.data.get("project_name")
        path = request.data.get("path")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        file_path = fal.path_join(code_path, path)

        fal.removefile(file_path)
        return JsonResponse({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def delete_folder(request):
    try:
        check_post_parameters(request.data, ["project_name", "path"])
        # Get the folder info
        project_name = request.data.get("project_name")
        path = request.data.get("path")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        file_path = fal.path_join(code_path, path)

        fal.removedir(file_path)
        return JsonResponse({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def save_file(request):
    try:
        check_post_parameters(
            request.data, ["project_name", "filename", ("content", -1)]
        )

        project_name = request.data.get("project_name")
        filename = request.data.get("filename")
        content = request.data.get("content")

        code_path = fal.code_path(project_name)
        file_path = fal.path_join(code_path, filename)

        fal.write(file_path, content)
        return Response({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def generate_local_app(request):
    try:
        check_post_parameters(request.data, ["app_name", "bt_order"])

        # Get the request parameters
        project_name = request.data.get("app_name")
        bt_order = request.data.get("bt_order")

        final_tree, actions = app_generator.generate_app(fal, project_name, bt_order)
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
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def generate_dockerized_app(request):
    try:
        check_post_parameters(request.data, ["app_name", "bt_order"])

        # Get the request parameters
        project_name = request.data.get("app_name")
        bt_order = request.data.get("bt_order")

        final_tree, _ = app_generator.generate_app(fal, project_name, bt_order)
        return JsonResponse({"success": True, "tree": final_tree})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_universe_file_list(request):
    try:
        check_get_parameters(request.GET, ["project_name", "universe_name"])

        project_name = request.GET.get("project_name")
        universe_name = request.GET.get("universe_name")

        universes_path = fal.universes_path(project_name)
        universe_path = fal.path_join(universes_path, universe_name)

        file_list = fal.list_formatted(universe_path)

        # Return the list of files
        return Response({"file_list": EntryEncoder().encode(file_list)})

    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["GET"])
def get_universe_file(request):
    try:
        check_get_parameters(request.GET, ["project_name", "universe_name", "filename"])

        project_name = request.GET.get("project_name", None)
        universe_name = request.GET.get("universe_name")
        filename = request.GET.get("filename", None)

        # Make folder path relative to Django app
        universes_path = fal.universes_path(project_name)
        universe_path = fal.path_join(universes_path, universe_name)

        file_path = fal.path_join(universe_path, filename)
        content = fal.read(file_path)
        serializer = FileContentSerializer({"content": content})
        return Response(serializer.data)
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


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
        fal.mkdir(universe_path)

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
        try:
            fal.removefile(temp_zip_path)
        except CUSTOM_EXCEPTIONS as e:
            return Response({"error": f"{str(e)}"}, status=e.error_code)
        except Exception as e:
            return Response({"success": False, "message": "Server error"}, status=500)

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
    try:
        check_post_parameters(request.data, ["app_name", "universe_name", "id"])

        # Get the name and the id file from the request
        universe_name = request.data.get("universe_name")
        project_name = request.data.get("app_name")
        id = request.data.get("id")

        # Make folder path relative to Django app
        universes_path = fal.universes_path(project_name)
        universe_path = fal.path_join(universes_path, universe_name)

        # Create the folder if it doesn't exist
        if not fal.exists(universe_path):
            fal.mkdir(universe_path)

        # Fill the config dictionary of the universe
        universe_config = {"name": universe_name, "id": id, "type": "robotics_backend"}

        # Generate the json config
        config_path = fal.path_join(universe_path, "config.json")

        universe_data = json.dumps(universe_config, ensure_ascii=False, indent=4)
        fal.create(config_path, universe_data)
        return Response(
            {"success": True, "message": "Universe uploaded successfully"},
            status=status.HTTP_200_OK,
        )
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)


@api_view(["POST"])
def upload_code(request):
    try:
        check_post_parameters(
            request.data, ["project_name", "file_name", "location", "content"]
        )

        # Get the name and the zip file from the request
        project_name = request.data.get("project_name")
        file_name = request.data.get("file_name")
        location = request.data.get("location")
        content = request.data.get("content")

        # Make folder path relative to Django app
        code_path = fal.code_path(project_name)
        create_path = fal.path_join(code_path, location)
        file_path = fal.path_join(create_path, file_name)

        # If file exist simply return
        if fal.exists(file_path):
            return JsonResponse(
                {"success": False, "message": "File already exists"}, status=409
            )

        fal.create_binary(file_path, base64.b64decode(content))
        return Response({"success": True})
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
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
    try:
        check_get_parameters(request.GET, ["name"])

        name = request.GET.get("name")

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
    except CUSTOM_EXCEPTIONS as e:
        return Response({"error": f"{str(e)}"}, status=e.error_code)
    except Exception as e:
        return Response({"error": f"An error occurred: {str(e)}"}, status=500)
