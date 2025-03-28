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

# Create your tests here.
# * Login: Entrar con un usuario de test
# * Crear Proyecto: Crear un nuevo proyecto llamado "test"
# * Entrar en proyecto: Entrar en el proyecto llamado "test"
# * Crear un acción: Crear una acción llamada Action usando una plantilla
# - Crear un BT: Crear un BT así: Root -> Sequence -> Action
# - Guardar el BT: Asegurar que se guarda el BT
# * Crear un Universo: Abrir el modal y crear un universo llamado "test" usando el universo del Robotics Backend: Follow Person
# * Seleccionar un Universo: Seleccionar el universo "test"
# - Ejecutar la aplicación: Ejecutar la aplicación
# - Monitorización de la ejecución: Comprobar que el monitor de ejecución cambia los valores
# * Borrar el universo: con la ejecución parada borrar el universo
# * Borrar el proyecto: hace falta salir de BT y volver a entrar para poder borrarlo


def create_proyect(self):
    # * Crear Proyecto: Crear un nuevo proyecto llamado "test"
    response = self.c.get("/bt_studio/get_project_list/")
    self.assertEqual("test" in response.json()["project_list"], False)
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
    self.assertEqual(response.json(), self.test_rb_universe_config)


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
        {"project_name": "test", "location": "dir", "file_name": file},
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


class LocalTestCase(TestCase):
    empty_project_content = {
        "file_list": '[{"is_dir": true, "name": "actions", "path": "actions", "files": []}, {"is_dir": true, "name": "trees", "path": "trees", "files": [{"is_dir": true, "name": "subtrees", "path": "trees/subtrees", "files": []}, {"is_dir": false, "name": "main.json", "path": "trees/main.json", "files": []}]}]'
    }

    test_rb_universe_config = {
        "success": True,
        "config": {"name": "test", "id": "Follow Person", "type": "robotics_backend"},
    }

    test_action_content = {
        "content": 'import py_trees\nimport geometry_msgs\n\nclass Action(py_trees.behaviour.Behaviour):\n\n    def __init__(self, name, ports = None):\n\n        """ Constructor, executed when the class is instantiated """\n\n        # Configure the name of the behavioure\n        super().__init__(name)\n        self.logger.debug("%s.__init__()" % (self.__class__.__name__))\n\n        # Get the ports\n        self.ports = ports\n\n    def setup(self, **kwargs: int) -> None:\n\n        """ Executed when the setup function is called upon the tree """\n\n        # Get the node passed from the tree (needed for interaction with ROS)\n        try:\n            self.node = kwargs[\'node\']\n        except KeyError as e:\n            error_message = "Couldn\'t find the tree node"\n            raise KeyError(error_message) from e\n\n    def initialise(self) -> None:\n\n        """ Executed when coming from an idle state """\n\n        # Debugging\n        self.logger.debug("%s.initialise()" % (self.__class__.__name__))\n\n    def update(self) -> py_trees.common.Status:\n\n        """ Executed when the action is ticked. Do not block! """\n\n        return py_trees.common.Status.RUNNING \n\n    def terminate(self, new_status: py_trees.common.Status) -> None:\n\n        """ Called whenever the behaviour switches to a non-running state """\n\n        # Debugging\n        self.logger.debug("%s.terminate()[%s->%s]" % (self.__class__.__name__, self.status, new_status))'
    }

    def setUp(self):
        self.c = Client()

    def tearDown(self):
        response = self.c.get("/bt_studio/get_project_list/")
        if "test" in response.json()["project_list"]:
            self.c.post("/bt_studio/delete_project/", {"project_name": "test"})

    def test_proyect(self):
        """Test if proyect is created with the proper content and deleted"""
        create_proyect(self)
        check_proyect_content(self)
        delete_proyect(self)

    def test_universe_db(self):
        response = self.c.get("/bt_studio/list_docker_universes/")
        self.assertEqual(response.status_code, 200)
        self.assertEqual(len(response.json()["universes"]) > 0, True)
        self.assertEqual("Follow Person" in response.json()["universes"], True)

        response = self.c.get(
            "/bt_studio/get_docker_universe_path/", {"name": "Follow Person"}
        )
        self.assertEqual(response.status_code, 200)

    def test_universe(self):
        """Test universe creation and deletion"""
        create_proyect(self)
        check_proyect_content(self)
        create_RB_universe(self, "Follow Person")
        select_RB_universe(self)
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
        create_proyect(self)
        check_proyect_content(self)
        create_folder(self, dir)
        create_file(self, dir, file)
        rename_file(self, dir, file, file_rename)
        rename_folder(self, dir, dir_rename)
        delete_file(self, dir_rename, file_rename)
        delete_folder(self, dir_rename)
        delete_proyect(self)


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
        cls.selenium.implicitly_wait(10)

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
        self.selenium.find_element(By.ID, "create-file-open").click()
        self.selenium.find_element(By.ID, "fileName").send_keys("Action")
        self.selenium.find_element(By.ID, "button-actionsType").click()
        self.selenium.find_element(By.ID, "button-actionTemplate").click()
        self.selenium.find_element(By.ID, "create-new-action").click()
        # Create BT
        self.selenium.find_element(By.ID, "Sequences").click()
        menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
        menu.find_element(By.ID, "Sequence").click()
        WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

        nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
        action = ActionChains(self.selenium) 

        root_out_tag_node = None
        sequence_node = None
        sequence_in_tag_node = None

        for node in nodes:
            name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
            if name == "Sequence":
                sequence_node = node
                sequence_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")
            elif name == "Tree Root":
                root_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")

        if sequence_node and root_out_tag_node and sequence_in_tag_node:
            action.drag_and_drop(sequence_node, root_out_tag_node )
            action.drag_and_drop_by_offset(sequence_node, 100, 0)
            action.drag_and_drop(root_out_tag_node, sequence_in_tag_node)
            action.perform()

        self.selenium.find_element(By.ID, "Actions").click()
        menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
        menu.find_element(By.ID, "Action").click()
        WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

        nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
        action = ActionChains(self.selenium) 

        sequence_out_tag_node = None
        action_node = None
        action_in_tag_node = None

        for node in nodes:
            name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
            if name == "Sequence":
                sequence_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")
            elif name == "Action":
                action_node = node
                action_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")

        if action_node and sequence_out_tag_node and action_in_tag_node:
            action.drag_and_drop(sequence_out_tag_node, action_in_tag_node)
            action.drag_and_drop(action_node, sequence_out_tag_node )
            action.click(sequence_node)
            action.drag_and_drop_by_offset(action_node, 100, 0)
            action.perform() 

        self.selenium.find_element(By.ID, "save-bt-changes").click()
        self.selenium.get_full_page_screenshot_as_file('/BtStudio/.test_logs/bt_create.png')

    # def test_bt_execution(self):
    #     print(self.live_server_url)
    #     self.selenium.get(f"{self.live_server_url}/frontend/")
    #     # Create Project
    #     self.selenium.find_element(By.ID, "create-project-open").click()
    #     self.selenium.find_element(By.ID, "projectName").send_keys("test")
    #     self.selenium.find_element(By.ID, "create-new-project").click()
    #     # Connect to RB
    #     rb_button = self.selenium.find_element(By.ID, "reset-connection")
    #     rb_button.click()
    #     # WebDriverWait(self.selenium, 3).until(EC.invisibility_of_element_located(rb_button))
    #     # Create Action
    #     self.selenium.find_element(By.ID, "create-file-open").click()
    #     self.selenium.find_element(By.ID, "fileName").send_keys("Action")
    #     self.selenium.find_element(By.ID, "button-actionsType").click()
    #     self.selenium.find_element(By.ID, "button-actionTemplate").click()
    #     self.selenium.find_element(By.ID, "create-new-action").click()
    #     # # Create BT
    #     self.selenium.find_element(By.ID, "Sequences").click()
    #     menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
    #     menu.find_element(By.ID, "Sequence").click()
    #     WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

    #     nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
    #     action = ActionChains(self.selenium) 

    #     root_out_tag_node = None
    #     sequence_node = None
    #     sequence_in_tag_node = None

    #     for node in nodes:
    #         name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
    #         if name == "Sequence":
    #             sequence_node = node
    #             sequence_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")
    #         elif name == "Tree Root":
    #             root_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")

    #     if sequence_node and root_out_tag_node and sequence_in_tag_node:
    #         action.drag_and_drop(sequence_node, root_out_tag_node )
    #         action.drag_and_drop_by_offset(sequence_node, 100, 0)
    #         action.drag_and_drop(root_out_tag_node, sequence_in_tag_node)
    #         action.perform()

    #     self.selenium.find_element(By.ID, "Actions").click()
    #     menu = self.selenium.find_element(By.CLASS_NAME, "MuiPopover-root")
    #     menu.find_element(By.ID, "Action").click()
    #     WebDriverWait(self.selenium, 0.2).until(EC.invisibility_of_element_located(menu))

    #     nodes = self.selenium.find_elements(By.CLASS_NAME, "bt-basic-node")
    #     action = ActionChains(self.selenium) 

    #     sequence_out_tag_node = None
    #     action_node = None
    #     action_in_tag_node = None

    #     for node in nodes:
    #         name = node.find_element(By.CLASS_NAME, "bt-basic-title").text
    #         if name == "Sequence":
    #             sequence_out_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-children-port")
    #         elif name == "Action":
    #             action_node = node
    #             action_in_tag_node = node.find_element(By.CLASS_NAME, "bt-basic-parent-port")

    #     if action_node and sequence_out_tag_node and action_in_tag_node:
    #         action.drag_and_drop(sequence_out_tag_node, action_in_tag_node)
    #         action.drag_and_drop(action_node, sequence_out_tag_node )
    #         action.click(sequence_node)
    #         action.drag_and_drop_by_offset(action_node, 100, 0)
    #         action.perform() 
    #     # # Move and connect # drag_and_drop_by_offset
    #     # self.selenium.find_element(By.ID, "save-bt-changes").click()
    #     # # Create Universe
    #     # self.selenium.find_element(By.ID, "open-universe-manager").click()
    #     # self.selenium.find_element(By.ID, "open-import-rb-universe").click()
    #     # self.selenium.find_element(By.ID, "universeName").send_keys("test")
    #     # self.selenium.find_element(By.ID, "dockerUniverseName").send_keys(
    #     #     "Follow Person"
    #     # )
    #     # self.selenium.find_element(By.ID, "create-new-universe").click()
    #     # self.selenium.find_element(By.ID, "project-test").click()
    #     # # Switch to BT monitor
    #     # self.selenium.find_element(By.ID, "bt-node-change-view-button").click()
    #     # # Run App
    #     # self.selenium.find_element(By.ID, "run-app").click()
    #     # self.selenium.pause(10)
    #     # self.selenium.find_element(By.ID, "run-app").click()
    #     self.selenium.save_screenshot('screenshot.png')
    #     # self.selenium.find_element(By.ID, "reset-app").click()
    #     # # Delete Universe
    #     # self.selenium.find_element(By.ID, "open-universe-manager").click()
    #     # self.selenium.find_element(By.ID, "delete-project-test").click()
    #     # self.selenium.find_element(By.ID, "close-modal").click()

        # frontend/
        # create-project-open
        # projectName
        # create-new-project
        # create-file-open
        # fileName
        # actionsType
        # actionTemplate
        # create-new-action
        # Sequences
        # Sequence
        # Move and connect # drag_and_drop_by_offset
        # Actions
        # Action
        # Move and connect # drag_and_drop_by_offset
        # save-bt-changes # Optional
        # open-universe-manager
        # open-import-rb-universe
        # universeName
        # dockerUniverseName
        # create-new-universe
        # project-test
        # bt-node-change-view-button
        # run-app
        # wait 10s
        # run-app
        # reset-app
        # open-universe-manager
        # delete-project-test
        # close-modal
