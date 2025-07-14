import json
from django.test import Client
from unittest import TestCase
from django.contrib.staticfiles.testing import StaticLiveServerTestCase
from selenium.webdriver.common.by import By
from selenium.webdriver.firefox.webdriver import WebDriver
from selenium.webdriver import FirefoxOptions
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.webdriver.common.action_chains import ActionChains
from . import tests_data

# Create your tests here.
# * Login: Entrar con un usuario de test
# * Crear Proyecto: Crear un nuevo proyecto llamado "test"
# * Entrar en proyecto: Entrar en el proyecto llamado "test"
# * Crear un acción: Crear una acción llamada Action usando una plantilla
# * Crear un BT: Crear un BT así: Root -> Sequence -> Action
# * Guardar el BT: Asegurar que se guarda el BT
# * Crear un Universo: Abrir el modal y crear un universo llamado "test" usando el universo del Robotics Backend: Follow Person
# * Seleccionar un Universo: Seleccionar el universo "test"
# - Ejecutar la aplicación: Ejecutar la aplicación
# - Monitorización de la ejecución: Comprobar que el monitor de ejecución cambia los valores
# * Borrar el universo: con la ejecución parada borrar el universo
# * Borrar el proyecto: hace falta salir de BT y volver a entrar para poder borrarlo
# Coverage: python -m coverage run --source="./backend/tree_api/" manage.py test && python -m coverage html


def create_proyect(self):
    # * Crear Proyecto: Crear un nuevo proyecto llamado "test"
    response = self.c.get("/bt_studio/get_project_list/")
    try:
        self.assertEqual("test" in response.json()["project_list"], False)
    except:
        pass
    response = self.c.post("/bt_studio/create_project/", {"project_name": "test"})
    self.assertEqual(response.status_code, 201)
    response = self.c.get("/bt_studio/get_project_list/")
    self.assertEqual("test" in response.json()["project_list"], True)


def check_proyect_content(self):
    # * Entrar en proyecto: Entrar en el proyecto llamado "test"
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    self.assertEqual(response.json(), self.empty_project_content)


def delete_proyect(self):
    # * Borrar el proyecto: hace falta salir de BT y volver a entrar para poder borrarlo
    response = self.c.get("/bt_studio/get_project_list/")
    self.assertEqual("test" in response.json()["project_list"], True)
    response = self.c.post("/bt_studio/delete_project/", {"project_name": "test"})
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_project_list/")
    self.assertEqual("test" in response.json()["project_list"], False)


def create_RB_universe(self, id):
    # * Crear un Universo: Abrir el modal y crear un universo llamado "test" usando el universo del Robotics Backend: Follow Person
    response = self.c.get("/bt_studio/get_universes_list/", {"project_name": "test"})
    self.assertEqual("test" in response.json()["universes_list"], False)
    response = self.c.post(
        "/bt_studio/add_docker_universe/",
        {"app_name": "test", "universe_name": "test", "id": id},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_universes_list/", {"project_name": "test"})
    self.assertEqual("test" in response.json()["universes_list"], True)


def select_RB_universe(self):
    # - Seleccionar un Universo: Seleccionar el universo "test"
    response = self.c.get(
        "/bt_studio/get_universe_configuration/",
        {"project_name": "test", "universe_name": "test"},
    )
    self.assertEqual(response.json()["config"], self.test_rb_universe_config)


def check_RB_universe(self):
    # - Seleccionar un Universo: Seleccionar el universo "test"
    response = self.c.get(
        "/bt_studio/get_file_list/",
        {"project_name": "test", "universe": "test"},
    )
    self.assertEqual(response.json(), self.base_rb_universe_content)


def delete_RB_universe(self):
    # - Borrar el universo: con la ejecución parada borrar el universo
    response = self.c.get("/bt_studio/get_universes_list/", {"project_name": "test"})
    self.assertEqual("test" in response.json()["universes_list"], True)
    response = self.c.post(
        "/bt_studio/delete_universe/", {"project_name": "test", "universe_name": "test"}
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_universes_list/", {"project_name": "test"})
    self.assertEqual("test" in response.json()["universes_list"], False)


def create_action(self):
    # - Crear un acción: Crear una acción llamada Action usando una plantilla
    response = self.c.get("/bt_studio/get_actions_list/", {"project_name": "test"})
    self.assertEqual("Action.py" in response.json()["actions_list"], False)
    self.assertEqual(len(response.json()["actions_list"]) == 0, True)
    response = self.c.post(
        "/bt_studio/create_action/",
        {"project_name": "test", "template": "action", "filename": "Action"},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_actions_list/", {"project_name": "test"})
    self.assertEqual("Action.py" in response.json()["actions_list"], True)


def check_action_content(self):
    # - Comprobar el contenido de una acción con plantilla llamada Action
    response = self.c.get(
        "/bt_studio/get_file/",
        {"project_name": "test", "filename": "actions/Action.py"},
    )
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.json(), self.test_action_content)


def delete_action(self):
    # - Borrar un acción: Crear una acción llamada Action usando una plantilla
    response = self.c.get("/bt_studio/get_actions_list/", {"project_name": "test"})
    self.assertEqual("Action.py" in response.json()["actions_list"], True)
    response = self.c.post(
        "/bt_studio/delete_file/",
        {"project_name": "test", "path": "actions/Action.py"},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_actions_list/", {"project_name": "test"})
    self.assertEqual("Action.py" in response.json()["actions_list"], False)


def create_file(self, dir, file):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data[1]["files"]), 0)
    response = self.c.post(
        "/bt_studio/create_file/",
        {"project_name": "test", "location": dir, "file_name": file},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(data[1]["files"][0]["name"], file)


def rename_file(self, dir, file, new_name):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(data[1]["files"][0]["name"], file)
    response = self.c.post(
        "/bt_studio/rename_file/",
        {
            "project_name": "test",
            "path": dir + "/" + file,
            "rename_to": dir + "/" + new_name,
        },
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data[1]["files"]), 1)
    self.assertEqual(data[1]["files"][0]["name"], new_name)


def delete_file(self, dir, file):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(data[1]["files"][0]["name"], file)
    response = self.c.post(
        "/bt_studio/delete_file/",
        {"project_name": "test", "path": dir + "/" + file},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data[1]["files"]), 0)


def upload_file(self, dir, file, content):
    response = self.c.post(
        "/bt_studio/upload_code/",
        {
            "project_name": "test",
            "location": dir,
            "file_name": file,
            "content": content,
        },
    )
    self.assertEqual(response.status_code, 200)


def write_to_file(self, dir, file, content):
    response = self.c.post(
        "/bt_studio/save_file/",
        {"project_name": "test", "filename": dir + "/" + file, "content": content},
    )
    self.assertEqual(response.status_code, 200)


def check_file(self, dir, file, content):
    response = self.c.get(
        "/bt_studio/get_file/", {"project_name": "test", "filename": dir + "/" + file}
    )
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.json()["content"], content)


def create_folder(self, dir):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data), 2)

    response = self.c.post(
        "/bt_studio/create_folder/",
        {"project_name": "test", "location": "", "folder_name": dir},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data), 3)
    self.assertEqual(data[1]["name"], dir)


def rename_folder(self, dir, new_name):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(data[1]["name"], dir)

    response = self.c.post(
        "/bt_studio/rename_folder/",
        {"project_name": "test", "path": dir, "rename_to": new_name},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data), 3)
    self.assertEqual(data[1]["name"], new_name)


def delete_folder(self, dir):
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(data[1]["name"], dir)
    response = self.c.post(
        "/bt_studio/delete_folder/",
        {"project_name": "test", "path": dir},
    )
    self.assertEqual(response.status_code, 200)
    response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
    data = json.loads(response.json()["file_list"])
    self.assertEqual(len(data), 2)


def get_tree(self, exepected):
    response = self.c.get(
        "/bt_studio/get_base_tree/",
        {"project_name": "test"},
    )
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.json()["graph_json"], exepected)


def write_tree(self, content):
    response = self.c.post(
        "/bt_studio/save_file/",
        {
            "project_name": "test",
            "filename": "trees/main.json",
            "content": content,
        },
    )
    self.assertEqual(response.status_code, 200)


def create_subtree(self, subtree):
    response = self.c.post(
        "/bt_studio/create_subtree/",
        {"project_name": "test", "subtree_name": subtree},
    )
    self.assertEqual(response.status_code, 201)


def get_subtree(self, subtree, exepected):
    response = self.c.get(
        "/bt_studio/get_subtree/",
        {"project_name": "test", "subtree_name": subtree},
    )
    self.assertEqual(response.status_code, 200)
    self.assertEqual(response.json()["subtree"], exepected)


def write_subtree(self, subtree, content):
    response = self.c.post(
        "/bt_studio/save_file/",
        {
            "project_name": "test",
            "filename": "trees/subtrees/" + subtree + ".json",
            "content": content,
        },
    )
    self.assertEqual(response.status_code, 200)


class LocalTestCase(TestCase):
    empty_project_content = {
        "file_list": '[{"is_dir": true, "name": "actions", "path": "actions", "group": "Code", "access": true, "files": []}, {"is_dir": true, "name": "trees", "path": "trees", "group": "Trees", "access": true, "files": [{"is_dir": true, "name": "subtrees", "path": "trees/subtrees", "group": "Trees", "access": true, "files": []}, {"is_dir": false, "name": "main.json", "path": "trees/main.json", "group": "Trees", "access": true, "files": []}]}]'
    }

    base_project_config = {
        "name": "test",
        "config": {
            "editorShowAccentColors": True,
            "theme": "dark",
            "btOrder": "bottom-to-top",
        },
    }

    test_rb_universe_config = {
        "name": "test",
        "id": "Follow Person",
        "type": "robotics_backend",
    }

    base_rb_universe_content = {
        "file_list": '[{"is_dir": true, "name": "test", "path": "test", "group": "Universes", "access": true, "files": [{"is_dir": false, "name": "config.json", "path": "test/config.json", "group": "Universes", "access": true, "files": []}]}]'
    }

    test_action_content = {
        "content": 'import py_trees\nimport geometry_msgs\n\nclass Action(py_trees.behaviour.Behaviour):\n\n    def __init__(self, name, ports = None):\n\n        """ Constructor, executed when the class is instantiated """\n\n        # Configure the name of the behavioure\n        super().__init__(name)\n        self.logger.debug("%s.__init__()" % (self.__class__.__name__))\n\n        # Get the ports\n        self.ports = ports\n\n    def setup(self, **kwargs: int) -> None:\n\n        """ Executed when the setup function is called upon the tree """\n\n        # Get the node passed from the tree (needed for interaction with ROS)\n        try:\n            self.node = kwargs[\'node\']\n        except KeyError as e:\n            error_message = "Couldn\'t find the tree node"\n            raise KeyError(error_message) from e\n\n    def initialise(self) -> None:\n\n        """ Executed when coming from an idle state """\n\n        # Debugging\n        self.logger.debug("%s.initialise()" % (self.__class__.__name__))\n\n    def update(self) -> py_trees.common.Status:\n\n        """ Executed when the action is ticked. Do not block! """\n\n        return py_trees.common.Status.RUNNING \n\n    def terminate(self, new_status: py_trees.common.Status) -> None:\n\n        """ Called whenever the behaviour switches to a non-running state """\n\n        # Debugging\n        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))'
    }

    base_tree_content = {
        "id": "fa2362dc-cffa-4764-9b70-90d475be0c02",
        "offsetX": -5.628119613229821,
        "offsetY": 4.788388817065306,
        "zoom": 66,
        "gridSize": 0,
        "layers": [
            {
                "id": "1d9af8b5-858e-4253-9057-fbf3c9f2442d",
                "type": "diagram-links",
                "isSvg": True,
                "transformed": True,
                "models": {},
            },
            {
                "id": "fed3b351-5f1a-4cad-9f61-031f8a6af57e",
                "type": "diagram-nodes",
                "isSvg": False,
                "transformed": True,
                "models": {
                    "94891f48-9deb-4724-a39d-3551b000dcf6": {
                        "id": "94891f48-9deb-4724-a39d-3551b000dcf6",
                        "type": "basic",
                        "selected": False,
                        "x": 200,
                        "y": 200,
                        "ports": [
                            {
                                "id": "d988236c-73d4-423a-8acc-05911daede71",
                                "type": "children",
                                "x": 291.64069082023457,
                                "y": 215.99999978001466,
                                "name": "children",
                                "alignment": "right",
                                "parentNode": "94891f48-9deb-4724-a39d-3551b000dcf6",
                                "links": [],
                                "in": False,
                                "label": "children",
                            }
                        ],
                        "name": "Tree Root",
                        "color": "rgb(0,204,0)",
                        "is_selected": False,
                    }
                },
            },
        ],
    }

    @classmethod
    def setUpClass(self):
        self.c = Client()

    def tearDown(self):
        # Delete all remaining projects
        response = self.c.get("/bt_studio/get_project_list/")
        if "test" in response.json()["project_list"]:
            response = self.c.post(
                "/bt_studio/delete_project/", {"project_name": "test"}
            )
            self.assertEqual(response.status_code, 200)
        response = self.c.get("/bt_studio/get_project_list/")
        self.assertEqual("test" in response.json()["project_list"], False)

    def test_proyect(self):
        """Test if proyect is created with the proper content and deleted"""
        create_proyect(self)
        check_proyect_content(self)
        delete_proyect(self)

    def test_project_config(self):
        """Test if proyect is created with the proper content and deleted"""
        create_proyect(self)
        check_proyect_content(self)
        response = self.c.get(
            "/bt_studio/get_project_configuration/", {"project_name": "test"}
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(json.loads(response.json()), self.base_project_config)
        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": response.json()},
        )
        self.assertEqual(response.status_code, 200)
        delete_proyect(self)

    def test_universe_db(self):
        response = self.c.get("/bt_studio/list_docker_universes/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.json()["universes"]) > 0, True)
        self.assertEqual("Follow Person" in response.json()["universes"], True)

        response = self.c.get(
            "/bt_studio/get_docker_universe_data/", {"name": "Follow Person"}
        )
        self.assertEqual(response.status_code, 200)

    def test_universe(self):
        """Test universe creation and deletion"""
        create_proyect(self)
        check_proyect_content(self)
        create_RB_universe(self, "Follow Person")
        select_RB_universe(self)
        check_RB_universe(self)
        response = self.c.get(
            "/bt_studio/get_file/",
            {
                "project_name": "test",
                "universe": "",
                "filename": "test/config.json",
            },
        )
        self.assertEqual(
            json.loads(response.json()["content"]), self.test_rb_universe_config
        )
        delete_RB_universe(self)
        delete_proyect(self)

    def test_actions(self):
        """Test action creation with templates"""
        create_proyect(self)
        check_proyect_content(self)
        create_action(self)
        check_action_content(self)
        delete_action(self)
        delete_proyect(self)

    def test_filesystem(self):
        """Test file and dir creation, rename and deletion"""
        dir = "dir"
        dir_rename = "dir2"
        file = "file.txt"
        file_rename = "file2.txt"
        content = "Test\nAgain"
        create_proyect(self)
        check_proyect_content(self)
        create_folder(self, dir)
        create_file(self, dir, file)
        write_to_file(self, dir, file, content)
        rename_file(self, dir, file, file_rename)
        check_file(self, dir, file_rename, content)
        rename_folder(self, dir, dir_rename)
        check_file(self, dir_rename, file_rename, content)
        delete_file(self, dir_rename, file_rename)
        delete_folder(self, dir_rename)
        delete_proyect(self)

    def test_upload_code(self):
        """Test file and dir creation, rename and deletion"""
        dir = "dir"
        file = "file.txt"
        content = "Test Again"
        content_b64 = "VGVzdCBBZ2Fpbg=="
        create_proyect(self)
        check_proyect_content(self)
        create_folder(self, dir)
        upload_file(self, dir, file, content_b64)
        check_file(self, dir, file, content)

    def test_tree_creation(self):
        """Test action creation with templates"""
        create_proyect(self)
        check_proyect_content(self)
        get_tree(self, self.base_tree_content)
        delete_proyect(self)

    def test_subtree_creation(self):
        """Test action creation with templates"""
        create_proyect(self)
        response = self.c.post(
            "/bt_studio/delete_folder/",
            {"project_name": "test", "path": "trees/subtrees"},
        )
        self.assertEqual(response.status_code, 200)
        create_subtree(self, "subtree")
        get_subtree(self, "subtree", self.base_tree_content)
        write_subtree(self, "subtree", "{}")
        get_subtree(self, "subtree", {})
        response = self.c.get(
            "/bt_studio/get_subtree_list/",
            {"project_name": "test"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["subtree_list"], ["subtree"])

    def test_tree_structure(self):
        """Test action creation with templates"""
        create_proyect(self)
        write_tree(self, tests_data.composition_tree_content)
        get_tree(self, json.loads(tests_data.composition_tree_content))
        response = self.c.get(
            "/bt_studio/get_tree_structure/",
            {"project_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            response.json()["tree_structure"], tests_data.composition_tree_structure
        )

    def test_subtree_structure(self):
        """Test action creation with templates"""
        create_proyect(self)
        write_tree(self, tests_data.composition_tree_content)
        get_tree(self, json.loads(tests_data.composition_tree_content))
        create_subtree(self, "AvoidObstacle")
        write_subtree(
            self, "AvoidObstacle", tests_data.composition_subtree_avoid_content
        )
        get_subtree(
            self,
            "AvoidObstacle",
            json.loads(tests_data.composition_subtree_avoid_content),
        )
        response = self.c.get(
            "/bt_studio/get_subtree_structure/",
            {
                "project_name": "test",
                "subtree_name": "AvoidObstacle",
                "bt_order": "top-to-bottom",
            },
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(
            response.json()["tree_structure"],
            tests_data.composition_subtree_avoid_structure,
        )

    def test_composition_generate_local_app(self):
        """Test action creation with templates"""
        create_proyect(self)

        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": tests_data.composition_proj_config},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "CheckObstacle"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )
        check_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Forward"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )
        check_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Turn"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Turn.py", tests_data.composition_action_turn_code
        )
        check_file(self, "actions", "Turn.py", tests_data.composition_action_turn_code)

        write_tree(self, tests_data.composition_tree_content)
        get_tree(self, json.loads(tests_data.composition_tree_content))
        create_subtree(self, "AvoidObstacle")
        write_subtree(
            self, "AvoidObstacle", tests_data.composition_subtree_avoid_content
        )
        get_subtree(
            self,
            "AvoidObstacle",
            json.loads(tests_data.composition_subtree_avoid_content),
        )
        create_subtree(self, "ObstacleDetection")
        write_subtree(
            self, "ObstacleDetection", tests_data.composition_subtree_obstacle_content
        )
        get_subtree(
            self,
            "ObstacleDetection",
            json.loads(tests_data.composition_subtree_obstacle_content),
        )

        response = self.c.post(
            "/bt_studio/generate_local_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["tree"], tests_data.composition_xml)
        self.assertEqual(
            response.json()["dependencies"], tests_data.composition_deps_list
        )

    def test_composition_generate_docker_app(self):
        """Test action creation with templates"""
        create_proyect(self)

        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": tests_data.composition_proj_config},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "CheckObstacle"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )
        check_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Forward"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )
        check_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Turn"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Turn.py", tests_data.composition_action_turn_code
        )
        check_file(self, "actions", "Turn.py", tests_data.composition_action_turn_code)

        write_tree(self, tests_data.composition_tree_content)
        get_tree(self, json.loads(tests_data.composition_tree_content))
        create_subtree(self, "AvoidObstacle")
        write_subtree(
            self, "AvoidObstacle", tests_data.composition_subtree_avoid_content
        )
        get_subtree(
            self,
            "AvoidObstacle",
            json.loads(tests_data.composition_subtree_avoid_content),
        )
        create_subtree(self, "ObstacleDetection")
        write_subtree(
            self, "ObstacleDetection", tests_data.composition_subtree_obstacle_content
        )
        get_subtree(
            self,
            "ObstacleDetection",
            json.loads(tests_data.composition_subtree_obstacle_content),
        )

        response = self.c.post(
            "/bt_studio/generate_dockerized_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["tree"], tests_data.composition_xml)

    def test_basic_generate_local_app(self):
        """Test action creation with templates"""
        create_proyect(self)

        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": tests_data.composition_proj_config},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/delete_folder/",
            {"project_name": "test", "path": "trees/subtrees"},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "CheckObstacle"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )
        check_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Forward"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )
        check_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Turn"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Turn.py", tests_data.composition_action_turn_code
        )
        check_file(self, "actions", "Turn.py", tests_data.composition_action_turn_code)

        write_tree(self, tests_data.basic_tree_content)
        get_tree(self, json.loads(tests_data.basic_tree_content))

        response = self.c.post(
            "/bt_studio/generate_local_app/",
            {"app_name": "test", "bt_order": "bottom-to-top"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["tree"], tests_data.basic_xml)
        self.assertEqual(
            response.json()["dependencies"], tests_data.composition_deps_list
        )

    def test_basic_generate_docker_app(self):
        """Test action creation with templates"""
        create_proyect(self)

        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": tests_data.composition_proj_config},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/delete_folder/",
            {"project_name": "test", "path": "trees/subtrees"},
        )
        self.assertEqual(response.status_code, 200)

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "CheckObstacle"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )
        check_file(
            self,
            "actions",
            "CheckObstacle.py",
            tests_data.composition_action_obstacle_code,
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Forward"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )
        check_file(
            self, "actions", "Forward.py", tests_data.composition_action_forward_code
        )

        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Turn"},
        )
        self.assertEqual(response.status_code, 200)
        write_to_file(
            self, "actions", "Turn.py", tests_data.composition_action_turn_code
        )
        check_file(self, "actions", "Turn.py", tests_data.composition_action_turn_code)

        write_tree(self, tests_data.basic_tree_content)
        get_tree(self, json.loads(tests_data.basic_tree_content))

        response = self.c.post(
            "/bt_studio/generate_dockerized_app/",
            {"app_name": "test", "bt_order": "bottom-to-top"},
        )
        self.assertEqual(response.status_code, 200)
        self.assertEqual(response.json()["tree"], tests_data.basic_xml)


class LocalTestFailedCase(TestCase):

    @classmethod
    def setUpClass(self):
        self.c = Client()
        self.no_param = 400
        self.no_files = 404
        self.dup_file = 409
        self.bad_data = 422
        self.bad_path = 403

    def tearDown(self):
        # Delete all remaining projects
        response = self.c.get("/bt_studio/get_project_list/")
        if "test" in response.json()["project_list"]:
            response = self.c.post(
                "/bt_studio/delete_project/", {"project_name": "test"}
            )
            self.assertEqual(response.status_code, 200)
        response = self.c.get("/bt_studio/get_project_list/")
        self.assertEqual("test" in response.json()["project_list"], False)

    def test_incorrect_create_proyect(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/create_project/")
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_create_proyect(self):
        """Test if error appears when paramters are wrong"""
        create_proyect(self)
        response = self.c.get("/bt_studio/get_project_list/")
        self.assertEqual("test" in response.json()["project_list"], True)
        response = self.c.post("/bt_studio/create_project/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.dup_file)
        delete_proyect(self)

    def test_incorrect_delete_proyect(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/delete_project/")
        self.assertEqual(response.status_code, self.no_param)

    def test_bad_name_delete_proyect(self):
        """Test if error appears when paramters are wrong"""
        response = self.c.get("/bt_studio/get_project_list/")
        self.assertEqual("test" in response.json()["project_list"], False)
        response = self.c.post("/bt_studio/delete_project/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_main_tree(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_base_tree/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_main_tree(self):
        """Test if error appears when paramters are wrong"""
        response = self.c.get("/bt_studio/get_base_tree/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.no_files)

    def test_bad_json_get_main_tree(self):
        """Test if error appears when paramters are wrong"""
        create_proyect(self)
        write_to_file(self, "trees", "main.json", "{[]}")
        check_file(self, "trees", "main.json", "{[]}")
        response = self.c.get("/bt_studio/get_base_tree/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.bad_data)
        delete_proyect(self)

    def test_incorrect_get_proy_config(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_project_configuration/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_proy_config(self):
        """Test if error appears when paramters are wrong"""
        response = self.c.get(
            "/bt_studio/get_project_configuration/", {"project_name": "test"}
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_tree_structure(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_tree_structure/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_tree_structure(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_tree_structure/",
            {"project_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_write_error_get_tree_structure(self):
        create_proyect(self)
        create_subtree(self, "subtree")
        response = self.c.get(
            "/bt_studio/get_tree_structure/",
            {
                "project_name": "test",
                "bt_order": "top-to-bottom",
            },
        )
        self.assertEqual(response.status_code, 500)
        delete_proyect(self)

    def test_incorrect_get_subtree_structure(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_subtree_structure/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_subtree_structure(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_subtree_structure/",
            {
                "project_name": "test",
                "subtree_name": "subtree",
                "bt_order": "top-to-bottom",
            },
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_write_error_get_subtree_structure(self):
        create_proyect(self)
        create_subtree(self, "subtree")
        response = self.c.get(
            "/bt_studio/get_subtree_structure/",
            {
                "project_name": "test",
                "subtree_name": "subtree",
                "bt_order": "top-to-bottom",
            },
        )
        self.assertEqual(response.status_code, 500)
        delete_proyect(self)

    def test_incorrect_save_proj_configuration(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/save_project_configuration/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_content_save_proj_configuration(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": ""},
        )
        self.assertEqual(response.status_code, 400)

    def test_bad_json_save_proj_configuration(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        response = self.c.post(
            "/bt_studio/save_project_configuration/",
            {"project_name": "test", "settings": "{"},
        )
        self.assertEqual(response.status_code, self.bad_data)
        delete_proyect(self)

    def test_incorrect_create_subtree(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/create_subtree/")
        self.assertEqual(response.status_code, self.no_param)
        response = self.c.post(
            "/bt_studio/create_subtree/",
            {"project_name": "", "subtree_name": "subtree"},
        )
        self.assertEqual(response.status_code, self.no_param)
        response = self.c.post(
            "/bt_studio/create_subtree/", {"project_name": "test", "subtree_name": ""}
        )
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_create_subtree(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_subtree(self, "subtree")
        response = self.c.post(
            "/bt_studio/create_subtree/",
            {"project_name": "test", "subtree_name": "subtree"},
        )
        self.assertEqual(response.status_code, self.dup_file)
        delete_proyect(self)

    def test_incorrect_get_subtree(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_subtree/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_subtree(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_subtree/",
            {"project_name": "test", "subtree_name": "subtree"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_subtree_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_subtree_list/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_subtree_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_subtree_list/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_delete_universe(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/delete_universe/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_delete_universe(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/delete_universe/",
            {"project_name": "test", "universe_name": "test"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_universes_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_universes_list/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_universes_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_universes_list/", {"project_name": "test"}
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_universe_configuration(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_universe_configuration/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_universe_configuration(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_universe_configuration/",
            {"project_name": "test", "universe_name": "test"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_bad_json_get_universe_configuration(self):
        """Test if error appears when paramters are wrong"""
        create_proyect(self)
        create_RB_universe(self, "Follow Person")
        write_to_file(self, "../universes/test/", "config.json", "{[]}")
        response = self.c.get(
            "/bt_studio/get_universe_configuration/",
            {"project_name": "test", "universe_name": "test"},
        )
        self.assertEqual(response.status_code, self.bad_data)
        delete_proyect(self)

    def test_incorrect_get_file_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_file_list/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_file_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_file_list/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_actions_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_actions_list/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_actions_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_actions_list/", {"project_name": "test"})
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_file/", {"project_name": "test", "filename": "a"}
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_create_action(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/create_action/")
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_create_action(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_action(self)
        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "action", "filename": "Action"},
        )
        self.assertEqual(response.status_code, self.dup_file)
        delete_action(self)
        delete_proyect(self)

    def test_no_template_create_action(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/create_action/",
            {"project_name": "test", "template": "error", "filename": "Action"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_create_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/create_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_create_file(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        create_file(self, "dir", "file.txt")
        response = self.c.post(
            "/bt_studio/create_file/",
            {"project_name": "test", "location": "dir", "file_name": "file.txt"},
        )
        self.assertEqual(response.status_code, self.dup_file)
        delete_proyect(self)

    def test_incorrect_create_folder(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/create_folder/")
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_create_folder(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        response = self.c.post(
            "/bt_studio/create_folder/",
            {"project_name": "test", "location": "", "folder_name": "dir"},
        )
        self.assertEqual(response.status_code, self.dup_file)
        delete_proyect(self)

    def test_invalid_name_create_folder(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        response = self.c.post(
            "/bt_studio/create_folder/",
            {"project_name": "test", "location": "", "folder_name": "a/./.."},
        )
        self.assertEqual(response.status_code, self.bad_path)
        delete_proyect(self)

    def test_incorrect_rename_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/rename_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_rename_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/rename_file/",
            {"project_name": "test", "path": "dir", "rename_to": "file.txt"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_invalid_name_rename_file(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        create_file(self, "dir", "file.txt")
        response = self.c.post(
            "/bt_studio/rename_file/",
            {
                "project_name": "test",
                "path": "dir/file.txt",
                "rename_to": "../file.txt",
            },
        )
        self.assertEqual(response.status_code, self.bad_path)
        delete_proyect(self)

    def test_incorrect_rename_folder(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/rename_folder/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_create_folder(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/rename_folder/",
            {"project_name": "test", "path": "dir/", "rename_to": "dir/"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_invalid_name_rename_folder(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        response = self.c.post(
            "/bt_studio/rename_folder/",
            {"project_name": "test", "path": "dir", "rename_to": ".."},
        )
        self.assertEqual(response.status_code, self.bad_path)
        delete_proyect(self)

    def test_incorrect_delete_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/delete_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_delete_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/delete_file/", {"project_name": "test", "path": "a"}
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_delete_folder(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/delete_folder/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_delete_folder(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/delete_folder/", {"project_name": "test", "path": "a"}
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_save_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/save_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_save_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/save_file/",
            {"project_name": "test", "filename": "a", "content": "a"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_generate_local_app(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/generate_local_app/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_generate_local_app(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/generate_local_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_tree_error_generate_local_app(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        response = self.c.post(
            "/bt_studio/generate_local_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, 500)
        delete_proyect(self)

    def test_incorrect_generate_dockerized_app(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/generate_dockerized_app/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_generate_dockerized_app(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post(
            "/bt_studio/generate_dockerized_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_tree_error_generate_dockerized_app(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        response = self.c.post(
            "/bt_studio/generate_dockerized_app/",
            {"app_name": "test", "bt_order": "top-to-bottom"},
        )
        self.assertEqual(response.status_code, 500)
        delete_proyect(self)

    def test_incorrect_get_universe_file_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_file_list/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_universe_file_list(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_file_list/",
            {"project_name": "test", "universe": "test"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_get_universe_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_file/")
        self.assertEqual(response.status_code, self.no_param)

    def test_no_find_get_universe_file(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get(
            "/bt_studio/get_file/",
            {"project_name": "test", "universe": "test", "filename": "a"},
        )
        self.assertEqual(response.status_code, self.no_files)

    def test_incorrect_add_docker_universe(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/add_docker_universe/")
        self.assertEqual(response.status_code, self.no_param)

    def test_duplicate_upload_code(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        create_file(self, "dir", "file.txt")
        response = self.c.post(
            "/bt_studio/upload_code/",
            {
                "project_name": "test",
                "location": "dir",
                "file_name": "file.txt",
                "content": "asff",
            },
        )
        self.assertEqual(response.status_code, self.dup_file)
        delete_proyect(self)

    def test_bad_zip_upload_code(self):
        """Test if error appears when no paramters are passed"""
        create_proyect(self)
        create_folder(self, "dir")
        response = self.c.post(
            "/bt_studio/upload_code/",
            {
                "project_name": "test",
                "location": "dir",
                "file_name": "file.txt",
                "content": "a",
            },
        )
        self.assertEqual(response.status_code, self.bad_data)
        delete_proyect(self)

    def test_incorrect_upload_code(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.post("/bt_studio/upload_code/")
        self.assertEqual(response.status_code, self.no_param)

    def test_incorrect_get_docker_universe_path(self):
        """Test if error appears when no paramters are passed"""
        response = self.c.get("/bt_studio/get_docker_universe_data/")
        self.assertEqual(response.status_code, self.no_param)


class SeleniumTests(StaticLiveServerTestCase):

    def setUp(self):
        self.c = Client()

    def tearDown(self):
        response = self.c.get("/bt_studio/get_project_list/")
        if "test" in response.json()["project_list"]:
            self.c.post("/bt_studio/delete_project/", {"project_name": "test"})

    @classmethod
    def setUpClass(cls):
        super().setUpClass()
        opts = FirefoxOptions()
        opts.add_argument("--headless")
        cls.selenium = WebDriver(options=opts)
        cls.selenium.implicitly_wait(2)

    @classmethod
    def tearDownClass(cls):
        cls.selenium.quit()
        super().tearDownClass()

    def test_bt_create(self):
        print(self.live_server_url)
        self.selenium.get(f"{self.live_server_url}/frontend/")
        # Create Project
        self.selenium.find_element(By.ID, "create-project-open").click()
        self.selenium.find_element(By.ID, "projectName").send_keys("test")
        self.selenium.find_element(By.ID, "create-new-project").click()
        # Create Action
        explorer = self.selenium.find_element(By.ID, "Code")
        explorer.find_element(By.ID, "new-file-button").click()
        self.selenium.find_element(By.ID, "fileName").send_keys("Action")
        self.selenium.find_element(By.ID, "button-actions").click()
        self.selenium.find_element(By.ID, "button-action").click()
        self.selenium.find_element(By.ID, "create-new-action").click()
        # Open BT
        explorer.find_element(By.XPATH, "//*[contains(text(), 'main.json')]").click()
        print()
        WebDriverWait(self.selenium, 0.5).until(
            EC.presence_of_element_located((By.ID, "Sequences"))
        )
        # Create BT
        self.selenium.find_element(By.ID, "Sequences").click()
        menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
        menu.find_element(By.ID, "Sequence").click()
        WebDriverWait(self.selenium, 0.2).until(
            EC.invisibility_of_element_located(menu)
        )

        nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
        action = ActionChains(self.selenium)

        root_out_tag_node = None
        sequence_node = None
        sequence_in_tag_node = None

        for node in nodes:
            name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
            if name == "Sequence":
                sequence_node = node
                sequence_in_tag_node = node.find_element(
                    By.CLASS_NAME, "bt-basic-parent-port"
                )
            elif name == "Tree Root":
                root_out_tag_node = node.find_element(
                    By.CLASS_NAME, "bt-basic-children-port"
                )

        if sequence_node and root_out_tag_node and sequence_in_tag_node:
            action.drag_and_drop(sequence_node, root_out_tag_node)
            action.drag_and_drop_by_offset(sequence_node, 100, 0)
            action.drag_and_drop(root_out_tag_node, sequence_in_tag_node)
            action.perform()

        self.selenium.find_element(By.ID, "Actions").click()
        menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
        menu.find_element(By.ID, "Action").click()
        WebDriverWait(self.selenium, 0.2).until(
            EC.invisibility_of_element_located(menu)
        )

        nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
        action = ActionChains(self.selenium)

        sequence_out_tag_node = None
        action_node = None
        action_in_tag_node = None

        for node in nodes:
            name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
            if name == "Sequence":
                sequence_out_tag_node = node.find_element(
                    By.CLASS_NAME, "bt-basic-children-port"
                )
            elif name == "Action":
                action_node = node
                action_in_tag_node = node.find_element(
                    By.CLASS_NAME, "bt-basic-parent-port"
                )

        if action_node and sequence_out_tag_node and action_in_tag_node:
            action.drag_and_drop(sequence_out_tag_node, action_in_tag_node)
            action.drag_and_drop(action_node, sequence_out_tag_node)
            action.click(sequence_node)
            action.drag_and_drop_by_offset(action_node, 100, 0)
            action.perform()

        self.selenium.find_element(By.ID, "save-button").click()
        self.selenium.get_full_page_screenshot_as_file(
            "/BtStudio/.test_logs/bt_create.png"
        )


#     # def test_bt_execution(self):
#     #     print(self.live_server_url)
#     #     self.selenium.get(f"{self.live_server_url}/frontend/")
#     #     # Create Project
#     #     self.selenium.find_element(By.ID, "create-project-open").click()
#     #     self.selenium.find_element(By.ID, "projectName").send_keys("test")
#     #     self.selenium.find_element(By.ID, "create-new-project").click()
#     #     # Connect to RB
#     #     rb_button = self.selenium.find_element(By.ID, "reset-connection")
#     #     rb_button.click()
#     #     # WebDriverWait(self.selenium, 3).until(EC.invisibility_of_element_located(rb_button))
#     #     # Create Action
#     #     self.selenium.find_element(By.ID, "create-file-open").click()
#     #     self.selenium.find_element(By.ID, "fileName").send_keys("Action")
#     #     self.selenium.find_element(By.ID, "button-actionsType").click()
#     #     self.selenium.find_element(By.ID, "button-actionTemplate").click()
#     #     self.selenium.find_element(By.ID, "create-new-action").click()
#     #     # # Create BT
#     #     self.selenium.find_element(By.ID, "Sequences").click()
#     #     menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
#     #     menu.find_element(By.ID, "Sequence").click()
#     #     WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

#     #     nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
#     #     action = ActionChains(self.selenium)

#     #     root_out_tag_node = None
#     #     sequence_node = None
#     #     sequence_in_tag_node = None

#     #     for node in nodes:
#     #         name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
#     #         if name == "Sequence":
#     #             sequence_node = node
#     #             sequence_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")
#     #         elif name == "Tree Root":
#     #             root_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")

#     #     if sequence_node and root_out_tag_node and sequence_in_tag_node:
#     #         action.drag_and_drop(sequence_node, root_out_tag_node )
#     #         action.drag_and_drop_by_offset(sequence_node, 100, 0)
#     #         action.drag_and_drop(root_out_tag_node, sequence_in_tag_node)
#     #         action.perform()

#     #     self.selenium.find_element(By.ID, "Actions").click()
#     #     menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
#     #     menu.find_element(By.ID, "Action").click()
#     #     WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

#     #     nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
#     #     action = ActionChains(self.selenium)

#     #     sequence_out_tag_node = None
#     #     action_node = None
#     #     action_in_tag_node = None

#     #     for node in nodes:
#     #         name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
#     #         if name == "Sequence":
#     #             sequence_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")
#     #         elif name == "Action":
#     #             action_node = node
#     #             action_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")

#     #     if action_node and sequence_out_tag_node and action_in_tag_node:
#     #         action.drag_and_drop(sequence_out_tag_node, action_in_tag_node)
#     #         action.drag_and_drop(action_node, sequence_out_tag_node )
#     #         action.click(sequence_node)
#     #         action.drag_and_drop_by_offset(action_node, 100, 0)
#     #         action.perform()
#     #     # # Move and connect # drag_and_drop_by_offset
#     #     # self.selenium.find_element(By.ID, "save-bt-changes").click()
#     #     # # Create Universe
#     #     # self.selenium.find_element(By.ID, "open-universe-manager").click()
#     #     # self.selenium.find_element(By.ID, "open-import-rb-universe").click()
#     #     # self.selenium.find_element(By.ID, "universeName").send_keys("test")
#     #     # self.selenium.find_element(By.ID, "dockerUniverseName").send_keys(
#     #     #     "Follow Person"
#     #     # )
#     #     # self.selenium.find_element(By.ID, "create-new-universe").click()
#     #     # self.selenium.find_element(By.ID, "project-test").click()
#     #     # # Switch to BT monitor
#     #     # self.selenium.find_element(By.ID, "bt-node-change-view-button").click()
#     #     # # Run App
#     #     # self.selenium.find_element(By.ID, "run-app").click()
#     #     # self.selenium.pause(10)
#     #     # self.selenium.find_element(By.ID, "run-app").click()
#     #     self.selenium.save_screenshot('screenshot.png')
#     #     # self.selenium.find_element(By.ID, "reset-app").click()
#     #     # # Delete Universe
#     #     # self.selenium.find_element(By.ID, "open-universe-manager").click()
#     #     # self.selenium.find_element(By.ID, "delete-project-test").click()
#     #     # self.selenium.find_element(By.ID, "close-modal").click()

#     # frontend/
#     # create-project-open
#     # projectName
#     # create-new-project
#     # create-file-open
#     # fileName
#     # actionsType
#     # actionTemplate
#     # create-new-action
#     # Sequences
#     # Sequence
#     # Move and connect # drag_and_drop_by_offset
#     # Actions
#     # Action
#     # Move and connect # drag_and_drop_by_offset
#     # save-bt-changes # Optional
#     # open-universe-manager
#     # open-import-rb-universe
#     # universeName
#     # dockerUniverseName
#     # create-new-universe
#     # project-test
#     # bt-node-change-view-button
#     # run-app
#     # wait 10s
#     # run-app
#     # reset-app
#     # open-universe-manager
#     # delete-project-test
#     # close-modal
