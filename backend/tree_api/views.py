import shutil
import json
import base64
from rest_framework.response import Response
from rest_framework import status
from django.http import JsonResponse
from django.conf import settings
from .serializers import FileContentSerializer
from .project_view import EntryEncoder
from .models import Universe
from .file_access import FAL
from .error_handler import error_wrapper
from . import json_translator
from . import app_generator

# PROJECT MANAGEMENT


fal = FAL(settings.BASE_DIR)


@error_wrapper("POST", ["project_name"])
def create_project(request):
    project_name = request.data.get("project_name")

    project_path = fal.project_path(project_name)
    action_path = fal.actions_path(project_name)
    universes_path = fal.universes_path(project_name)
    tree_path = fal.trees_path(project_name)
    subtree_path = fal.subtrees_path(project_name)

    config_path = fal.path_join(project_path, "config.json")
    base_tree_path = fal.path_join(tree_path, "main.json")

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
    graph_data = fal.get_base_tree_template()
    fal.create(base_tree_path, graph_data)
    return Response(
        {"success": True, "message": "Project created successfully"},
        status=status.HTTP_201_CREATED,
    )


@error_wrapper("POST", ["project_name"])
def delete_project(request):
    project_name = request.data.get("project_name")
    project_path = fal.project_path(project_name)
    fal.removedir(project_path)
    return Response({"success": True}, status=200)


@error_wrapper("GET")
def get_project_list(request):
    folder_path = fal.base_path()
    project_list = fal.listdirs(folder_path)
    return Response({"project_list": project_list})


@error_wrapper("POST", ["project_name", "graph_json"])
def save_base_tree(request):
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


@error_wrapper("GET", ["project_name"])
def get_base_tree(request):
    project_name = request.GET.get("project_name")

    # Generate the paths
    trees_path = fal.trees_path(project_name)
    graph_path = fal.path_join(trees_path, "main.json")

    graph_data = json.loads(fal.read(graph_path))
    return JsonResponse({"success": True, "graph_json": graph_data})


@error_wrapper("GET", ["project_name"])
def get_project_configuration(request):
    project_name = request.GET.get("project_name")

    project_path = fal.project_path(project_name)
    config_path = fal.path_join(project_path, "config.json")
    content = fal.read(config_path)
    return Response(content)


@error_wrapper("GET", ["project_name", "bt_order"])
def get_tree_structure(request):
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


@error_wrapper("GET", ["project_name", "subtree_name", "bt_order"])
def get_subtree_structure(request):
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


@error_wrapper("POST", ["project_name", "settings"])
def save_project_configuration(request):
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


# SUBTREE MANAGEMENT


@error_wrapper("POST", ["project_name", "subtree_name"])
def create_subtree(request):
    project_name = request.data.get("project_name")
    subtree_name = request.data.get("subtree_name")

    project_actions_path = fal.actions_path(project_name)
    project_subtree_path = fal.subtrees_path(project_name)

    library_base_path = fal.path_join(settings.BASE_DIR, "library")
    library_path = fal.path_join(library_base_path, subtree_name)
    library_actions_path = fal.path_join(library_path, "actions")
    template_path = fal.path_join(settings.BASE_DIR, "templates")
    project_json_path = fal.path_join(project_subtree_path, f"{subtree_name}.json")
    src_path = template_path

    # Check if the subtree is already implemented on the library
    if fal.exists(library_path):
        src_path = library_path
        if fal.exists(library_actions_path):
            shutil.copytree(
                library_actions_path, project_actions_path, dirs_exist_ok=True
            )

    # Create subtree directory if it does not exist
    if not fal.exists(project_subtree_path):
        fal.mkdir(project_subtree_path)

    # Setup init and copy paths
    if src_path == template_path:
        subtree = json.loads(fal.get_base_subtree_template())
    else:
        init_json_path = fal.path_join(src_path, "graph.json")
        subtree = json.loads(fal.read(init_json_path))

    subtree_formated = json.dumps(subtree, indent=4)
    fal.create(project_json_path, subtree_formated)
    return JsonResponse({"success": True}, status=status.HTTP_201_CREATED)


@error_wrapper("POST", ["project_name", "subtree_name", "subtree_json"])
def save_subtree(request):
    # Get the project name, subtree name, and subtree JSON
    project_name = request.data.get("project_name")
    subtree_name = request.data.get("subtree_name")
    subtree_json = request.data.get("subtree_json")

    # Generate the paths
    subtrees_path = fal.subtrees_path(project_name)
    subtree_path = fal.path_join(subtrees_path, f"{subtree_name}.json")

    fal.write(subtree_path, subtree_json)
    return JsonResponse({"success": True}, status=status.HTTP_200_OK)


@error_wrapper("GET", ["project_name", "subtree_name"])
def get_subtree(request):
    project_name = request.GET.get("project_name")
    subtree_name = request.GET.get("subtree_name")

    subtrees_path = fal.subtrees_path(project_name)
    subtree_path = fal.path_join(subtrees_path, f"{subtree_name}.json")

    subtree = json.loads(fal.read(subtree_path))
    return Response({"subtree": subtree}, status=status.HTTP_200_OK)


@error_wrapper("GET", ["project_name"])
def get_subtree_list(request):
    project_name = request.GET.get("project_name")

    subtrees_path = fal.subtrees_path(project_name)

    # List all files in the directory removing the .json extension
    subtree_list = fal.listfiles(subtrees_path)
    subtree_list = [f.split(".")[0] for f in subtree_list]
    return Response({"subtree_list": subtree_list})


# UNIVERSE MANAGEMENT


@error_wrapper("POST", ["project_name", "universe"])
def create_universe(request):
    project_name = request.data.get("project_name")
    universe_name = request.data.get("universe")

    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)

    fal.mkdir(universe_path)
    return Response({"success": True}, status=200)


@error_wrapper("POST", ["project_name", "universe_name"])
def delete_universe(request):
    project_name = request.data.get("project_name")
    universe_name = request.data.get("universe_name")

    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)

    fal.removedir(universe_path)
    return Response({"success": True}, status=200)


@error_wrapper("GET", ["project_name"])
def get_universes_list(request):
    project_name = request.GET.get("project_name")

    universes_path = fal.universes_path(project_name)

    universes_list = fal.listdirs(universes_path)
    return Response({"universes_list": universes_list})


@error_wrapper("POST", ["project_name", "universe_name"])
def create_universe_configuration(request):
    project_name = request.data.get("project_name")
    universe_name = request.data.get("universe_name")

    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)
    config_path = fal.path_join(universe_path, "config.json")

    universe_config = {
        "name": universe_name,
        "type": "custom",
        "ram_config": {
            "ros_version": "ROS2",
            "world": "gazebo",
            "launch_file_path": "",
            "visualization_config_path": "",
        },
    }

    universe_data = json.dumps(universe_config, ensure_ascii=False, indent=4)
    fal.create(config_path, universe_data)

    return Response(
        {"success": True, "message": "Universe config created successfully"}, status=200
    )


@error_wrapper("GET", ["project_name", "universe_name"])
def get_universe_configuration(request):
    project_name = request.GET.get("project_name")
    universe_name = request.GET.get("universe_name")

    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)
    config_path = fal.path_join(universe_path, "config.json")

    content = json.loads(fal.read(config_path))
    return Response({"success": True, "config": content}, status=200)  # Return as JSON


# FILE MANAGEMENT


@error_wrapper("GET", ["project_name"])
def get_file_list(request):
    project_name = request.GET.get("project_name")
    universe = request.GET.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    file_list = fal.list_formatted(path)

    # Return the list of files
    return Response({"file_list": EntryEncoder().encode(file_list)})


@error_wrapper("GET", ["project_name"])
def get_actions_list(request):
    project_name = request.GET.get("project_name")

    action_path = fal.actions_path(project_name)

    actions_list = fal.listfiles(action_path)
    return Response({"actions_list": actions_list})


@error_wrapper("GET", ["project_name", "filename"])
def get_file(request):
    project_name = request.GET.get("project_name", None)
    filename = request.GET.get("filename", None)
    universe = request.GET.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    file_path = fal.path_join(path, filename)
    content = fal.read(file_path)
    serializer = FileContentSerializer({"content": content})
    return Response(serializer.data)


@error_wrapper("POST", ["project_name", "filename", "template"])
def create_action(request):
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


@error_wrapper("POST", ["project_name", ("location", -1), "file_name"])
def create_file(request):
    project_name = request.data.get("project_name")
    location = request.data.get("location")
    filename = request.data.get("file_name")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    create_path = fal.path_join(path, location)
    file_path = fal.path_join(create_path, filename)

    fal.create(file_path, "")
    return Response({"success": True})


@error_wrapper("POST", ["project_name", ("location", -1), "folder_name"])
def create_folder(request):
    project_name = request.data.get("project_name")
    location = request.data.get("location")
    folder_name = request.data.get("folder_name")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    create_path = fal.path_join(path, location)
    folder_path = fal.path_join(create_path, folder_name)

    fal.mkdir(folder_path)
    return Response({"success": True})


@error_wrapper("POST", ["project_name", "path", "rename_to"])
def rename_file(request):
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        base_path = fal.path_join(universes_path, universe)
    else:
        base_path = fal.code_path(project_name)

    file_path = fal.path_join(base_path, path)
    new_path = fal.path_join(base_path, rename_path)

    fal.renamefile(file_path, new_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_name", "path", "rename_to"])
def rename_folder(request):
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    rename_path = request.data.get("rename_to")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        base_path = fal.path_join(universes_path, universe)
    else:
        base_path = fal.code_path(project_name)

    file_path = fal.path_join(base_path, path)
    new_path = fal.path_join(base_path, rename_path)

    fal.renamedir(file_path, new_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_name", "path"])
def delete_file(request):
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        base_path = fal.path_join(universes_path, universe)
    else:
        base_path = fal.code_path(project_name)

    file_path = fal.path_join(base_path, path)

    fal.removefile(file_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_name", "path"])
def delete_folder(request):
    project_name = request.data.get("project_name")
    path = request.data.get("path")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        base_path = fal.path_join(universes_path, universe)
    else:
        base_path = fal.code_path(project_name)

    file_path = fal.path_join(base_path, path)

    fal.removedir(file_path)
    return JsonResponse({"success": True})


@error_wrapper("POST", ["project_name", "filename", ("content", -1)])
def save_file(request):
    project_name = request.data.get("project_name")
    filename = request.data.get("filename")
    content = request.data.get("content")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    file_path = fal.path_join(path, filename)

    fal.write(file_path, content)
    return Response({"success": True})


@error_wrapper("POST", ["app_name", "bt_order"])
def generate_local_app(request):
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


@error_wrapper("POST", ["app_name", "bt_order"])
def generate_dockerized_app(request):
    # Get the request parameters
    project_name = request.data.get("app_name")
    bt_order = request.data.get("bt_order")

    final_tree, _ = app_generator.generate_app(fal, project_name, bt_order)
    return JsonResponse({"success": True, "tree": final_tree})


@error_wrapper("POST", ["project_name", "universe_name"])
def create_custom_universe(request):
    # Get the name and the zip file from the request
    project_name = request.data.get("project_name")
    universe_name = request.data.get("universe_name")

    # Make folder path relative to Django app
    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)
    universe_launch_path = fal.path_join(universe_path, "launch")
    universe_world_path = fal.path_join(universe_path, "worlds")
    universe_models_path = fal.path_join(universe_path, "models")

    fal.mkdir(universe_path)
    fal.mkdir(universe_launch_path)
    fal.mkdir(universe_world_path)
    fal.mkdir(universe_models_path)

    # Fill the config dictionary of the universe
    ram_launch_path = (
        "/workspace/worlds/src/" + universe_name + "/launch/universe.launch.py"
    )
    ram_visualization_config_path = (
        "/workspace/worlds/src/" + universe_name + "/gz.config"
    )

    universe_config = {
        "name": universe_name,
        "type": "custom",
        "ram_config": {
            "ros_version": "ROS2",
            "world": "gazebo",
            "launch_file_path": ram_launch_path,
            "visualization_config_path": ram_visualization_config_path,
        },
    }

    # Generate the json config
    config_path = fal.path_join(universe_path, "config.json")

    universe_data = json.dumps(universe_config, ensure_ascii=False, indent=4)
    fal.create(config_path, universe_data)

    templates = fal.get_universe_template(universe_name)

    for t in templates:
        fal.create(fal.path_join(universe_path, t["path"]), t["content"])

    return Response(
        {"success": True, "message": "Universe uploaded successfully"},
        status=status.HTTP_200_OK,
    )


@error_wrapper("POST", ["app_name", "universe_name", "id"])
def add_docker_universe(request):
    # Get the name and the id file from the request
    universe_name = request.data.get("universe_name")
    project_name = request.data.get("app_name")
    id = request.data.get("id")

    # Make folder path relative to Django app
    universes_path = fal.universes_path(project_name)
    universe_path = fal.path_join(universes_path, universe_name)

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


@error_wrapper("POST", ["project_name", "file_name", ("location", -1), "content"])
def upload_code(request):
    # Get the name and the zip file from the request
    project_name = request.data.get("project_name")
    file_name = request.data.get("file_name")
    location = request.data.get("location")
    content = request.data.get("content")
    universe = request.data.get("universe")

    if universe is not None:
        universes_path = fal.universes_path(project_name)
        path = fal.path_join(universes_path, universe)
    else:
        path = fal.code_path(project_name)

    create_path = fal.path_join(path, location)
    file_path = fal.path_join(create_path, file_name)

    fal.create_binary(file_path, base64.b64decode(content))
    return Response({"success": True})


@error_wrapper("GET")
def list_docker_universes(request):
    universes = Universe.objects.all()
    universes_docker_list = [x.name for x in universes]
    # Return the list of projects
    return Response({"universes": universes_docker_list})


@error_wrapper("GET", ["name"])
def get_docker_universe_data(request):
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
