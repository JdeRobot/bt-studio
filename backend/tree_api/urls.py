from django.urls import path
from . import views

urlpatterns = [
    # Project Management
    path("create_project/", views.create_project, name="create_project"),
    path("delete_project/", views.delete_project, name="delete_project"),
    path("get_project_list/", views.get_project_list, name="get_project_list"),
    # Universe Management
    path("upload_universe/", views.upload_universe, name="upload_universe"),
    path("delete_universe/", views.delete_universe, name="delete_universe"),
    path("add_docker_universe/", views.add_docker_universe, name="add_docker_universe"),
    path("get_universes_list/", views.get_universes_list, name="get_universes_list"),
    path("get_universe_zip/", views.get_universe_zip, name="get_universe_zip"),
    path(
        "get_universe_configuration/",
        views.get_universe_configuration,
        name="get_universe_configuration",
    ),
    path(
        "list_docker_universes/",
        views.list_docker_universes,
        name="list_docker_universes",
    ),
    path(
        "get_docker_universe_path/",
        views.get_docker_universe_path,
        name="get_docker_universe_path",
    ),
    # Tree Management
    path("save_base_tree/", views.save_base_tree, name="save_base_tree"),
    path("get_project_graph/", views.get_project_graph, name="get_project_graph"),
    path("get_tree_structure/", views.get_tree_structure, name="get_tree_structure"),
    path(
        "get_subtree_structure/",
        views.get_subtree_structure,
        name="get_subtree_structure",
    ),
    path(
        "save_base_tree_configuration/",
        views.save_base_tree_configuration,
        name="save_base_tree_configuration",
    ),
    path("get_subtree_list/", views.get_subtree_list, name="get_subtree_list"),
    path("create_subtree/", views.create_subtree, name="create_subtree"),
    path("get_subtree/", views.get_subtree, name="get_subtree"),
    path("save_subtree/", views.save_subtree, name="save_subtree"),
    # File and Folder Management
    path("get_file_list/", views.get_file_list, name="get_file_list"),
    path("get_file/", views.get_file, name="get_file"),
    path("create_file/", views.create_file, name="create_file"),
    path("create_folder/", views.create_folder, name="create_folder"),
    path("rename_file/", views.rename_file, name="rename_file"),
    path("delete_file/", views.delete_file, name="delete_file"),
    path("delete_folder/", views.delete_folder, name="delete_folder"),
    path("save_file/", views.save_file, name="save_file"),
    path("upload_code/", views.upload_code, name="upload_code"),
    # Actions Management
    path("get_actions_list/", views.get_actions_list, name="get_actions_list"),
    path("create_action/", views.create_action, name="create_action"),
    # Other
    path("generate_local_app/", views.generate_local_app, name="generate_local_app"),
    path(
        "generate_dockerized_app/",
        views.generate_dockerized_app,
        name="generate_dockerized_app",
    ),
    path(
        "get_project_configuration/",
        views.get_project_configuration,
        name="get_project_configuration",
    ),
]
