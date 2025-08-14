from django.urls import path
from . import views

urlpatterns = [
    # Project Management
    path("create_project/", views.create_project, name="create_project"),
    path("delete_project/", views.delete_project, name="delete_project"),
    path("get_project_list/", views.get_project_list, name="get_project_list"),
    path(
        "save_project_configuration/",
        views.save_project_configuration,
        name="save_project_configuration",
    ),
    path(
        "get_project_configuration/",
        views.get_project_configuration,
        name="get_project_configuration",
    ),
    # Universe Management
    path("create_universe/", views.create_universe, name="create_universe"),
    path("delete_universe/", views.delete_universe, name="delete_universe"),
    path(
        "create_universe_configuration/",
        views.create_universe_configuration,
        name="create_universe_configuration",
    ),
    path(
        "get_universe_configuration/",
        views.get_universe_configuration,
        name="get_universe_configuration",
    ),
    path("get_universes_list/", views.get_universes_list, name="get_universes_list"),
    ## Robotics Backend Universes
    path("add_docker_universe/", views.add_docker_universe, name="add_docker_universe"),
    path(
        "get_docker_universe_data/",
        views.get_docker_universe_data,
        name="get_docker_universe_data",
    ),
    path(
        "list_docker_universes/",
        views.list_docker_universes,
        name="list_docker_universes",
    ),
    ## Custom Universes
    path(
        "create_custom_universe/",
        views.create_custom_universe,
        name="create_custom_universe",
    ),
    # Tree Management
    path("get_base_tree/", views.get_base_tree, name="get_base_tree"),
    path("get_tree_structure/", views.get_tree_structure, name="get_tree_structure"),
    # Subtree Management
    path("create_subtree/", views.create_subtree, name="create_subtree"),
    path("get_subtree/", views.get_subtree, name="get_subtree"),
    path("get_subtree_path/", views.get_subtree_path, name="get_subtree_path"),
    path("get_subtree_list/", views.get_subtree_list, name="get_subtree_list"),
    path(
        "get_subtree_structure/",
        views.get_subtree_structure,
        name="get_subtree_structure",
    ),
    # File Management
    path("create_file/", views.create_file, name="create_file"),
    path("delete_file/", views.delete_file, name="delete_file"),
    path("rename_file/", views.rename_file, name="rename_file"),
    path("save_file/", views.save_file, name="save_file"),
    path("get_file/", views.get_file, name="get_file"),
    path("get_file_list/", views.get_file_list, name="get_file_list"),
    # Folder Management
    path("create_folder/", views.create_folder, name="create_folder"),
    path("delete_folder/", views.delete_folder, name="delete_folder"),
    path("rename_folder/", views.rename_folder, name="rename_folder"),
    # Actions Management
    path("create_action/", views.create_action, name="create_action"),
    path("get_actions_list/", views.get_actions_list, name="get_actions_list"),
    # Generate App
    path("generate_local_app/", views.generate_local_app, name="generate_local_app"),
    path(
        "generate_dockerized_app/",
        views.generate_dockerized_app,
        name="generate_dockerized_app",
    ),
    # Other
    path("upload_code/", views.upload_code, name="upload_code"),
    # Subtree Library
    path(
        "get_subtree_library_list/",
        views.get_subtree_library_list,
        name="get_subtree_library_list",
    ),
    path(
        "get_user_subtree_library_list/",
        views.get_user_subtree_library_list,
        name="get_user_subtree_library_list",
    ),
    path("get_library_tree/", views.get_library_tree, name="get_library_tree"),
    path(
        "get_user_library_tree/",
        views.get_user_library_tree,
        name="get_user_library_tree",
    ),
    path("import_library_tree/", views.import_library_tree, name="import_library_tree"),
    path(
        "import_user_library_tree/",
        views.import_user_library_tree,
        name="import_user_library_tree",
    ),
]
