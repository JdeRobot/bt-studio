import axios, { AxiosResponse } from "axios";
import { SettingsData, SettingData } from "../components/options/Options";
import { publish } from "../components/helper/TreeEditorHelper";

// Helpers

const isSuccessful = (response: AxiosResponse) => {
  return response.status >= 200 && response.status < 300;
};
const getCookie=(name: string)=>{

    const value = `; ${document.cookie}`;
    const parts = value.split(`; ${name}=`);  //to get CSRF token among all the cookies in 'value'
    if (parts.length === 2) return parts.pop()?.split(';').shift();
    return undefined;
};

const csrfToken = getCookie('csrftoken');
const axiosExtra = {
  headers: {
    //@ts-ignore Needed for compatibility with Unibotics
    "X-CSRFToken": csrfToken,
  },
};

// File management

const getFileList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_file_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get file list."); // Response error
    }

    return response.data.file_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getFile = async (projectName: string, fileName: string) => {
  if (!projectName) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  const apiUrl = `/bt_studio/get_file?project_name=${encodeURIComponent(projectName)}&filename=${encodeURIComponent(fileName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get file list."); // Response error
    }

    return response.data.content;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getActionsList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_actions_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get actions list."); // Response error
    }

    if (! Array.isArray(response.data.actions_list)) {
      throw new Error("API response is not an array")
    }

    return response.data.actions_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const saveFile = async (
  projectName: string,
  fileName: string,
  content: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("Current File name is not set");
  if (!content) throw new Error("Content does not exist");

  if (fileName.split("/")[0] === "trees") {
    console.log(fileName + " is Read Only."); //TODO: test in Unibotics
    return;
  }

  const apiUrl = "/bt_studio/save_file/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        filename: fileName,
        content: content,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        //throw new Error("You're using too much AWS space!" ||  response.data.message)
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error) {
    console.log(error)
    throw error; // Rethrow
  }
};

// Project management

const createProject = async (projectName: string) => {
  if (!projectName.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/create_project/`;

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const deleteProject = async (projectName: string) => {
  if (!projectName.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/delete_project/`;

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to delete project."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const saveBaseTree = async (modelJson: string, currentProjectname: string) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = "/bt_studio/save_base_tree/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: currentProjectname,
        graph_json: JSON.stringify(modelJson),
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const loadProjectConfig = async (
  currentProjectname: string,
  settings: SettingsData
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_configuration?project_name=${currentProjectname}`;
  try {
    const response = await axios.get(apiUrl);

    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project config"
      );
    }

    let raw_config = JSON.parse(response.data);
    let project_settings = raw_config.config;

    Object.entries(settings).forEach(([key, value]) => {
      value.setter(
        project_settings[key] ? project_settings[key] : value.default_value
      );
    });
  } catch (error) {
    console.log("Loading default settings");
    Object.entries(settings).forEach(([key, value]) => {
      value.setter(value.default_value);
    });

    throw error;
  }
};

const saveProjectConfig = async (
  currentProjectname: string,
  settings: string
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!settings) throw new Error("Settings content is null");

  const apiUrl = "/bt_studio/save_project_configuration/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: currentProjectname,
        settings: settings,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const getProjectGraph = async (currentProjectname: string) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_graph?project_name=${currentProjectname}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project graph"
      ); // Response error
    }

    return response.data.graph_json;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// Universe management

const getUniverseConfig = async (
  universeName: string,
  currentProjectname: string
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_universe_configuration?project_name=${encodeURIComponent(
    currentProjectname
  )}&universe_name=${encodeURIComponent(universeName)}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve universe config"
      ); // Response error
    }

    return JSON.stringify(response.data.config);
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getRoboticsBackendUniversePath = async (universeName: string) => {
  if (!universeName) throw new Error("The universe name is not set");

  const apiUrl = `/bt_studio/get_docker_universe_path?name=${encodeURIComponent(universeName)}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve universe config"
      ); // Response error
    }

    return {
      world: response.data.universe.world,
      robot: response.data.universe.robot,
    };
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const createRoboticsBackendUniverse = async (
  projectName: string,
  universeName: string,
  universeId: string
) => {
  if (!projectName) throw new Error("The project name is not set");
  if (!universeName) throw new Error("The universe name is not set");
  if (!universeId) throw new Error("The universe id is not set");

  const apiUrl = "/bt_studio/add_docker_universe/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        app_name: projectName,
        universe_name: universeName,
        id: universeId,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to save subtree."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const deleteUniverse = async (projectName: string, universeName: string) => {
  if (!projectName) throw new Error("The project name is not set");
  if (!universeName) throw new Error("The universe name is not set");

  const apiUrl = "/bt_studio/delete_universe/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        universe_name: universeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to save subtree."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getCustomUniverseZip = async (
  universeName: string,
  currentProjectname: string
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = "/bt_studio/get_universe_zip/";
  try {
    // Make the request
    const response = await axios.post(
      apiUrl,
      {
        app_name: currentProjectname,
        universe_name: universeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve custom universe"
      ); // Response error
    }
    return response.data;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// App management

const generateLocalApp = async (
  currentProjectname: string,
  btOrder: string
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/generate_local_app/`;
  try {
    const response = await axios.post(
      apiUrl,
      {
        app_name: currentProjectname,
        bt_order: btOrder,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create app."); // Response error
    }

    return response.data;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const generateDockerizedApp = async (
  currentProjectname: string,
  btOrder: string
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/generate_dockerized_app/`;
  try {
    const response = await axios.post(
      apiUrl,
      {
        app_name: currentProjectname,
        bt_order: btOrder,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create app."); // Response error
    }

    return response.data;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// Subtree management

const createSubtree = async (
  subtreeName: string,
  currentProjectname: string
) => {
  if (!subtreeName.trim()) {
    throw new Error("Subtree name cannot be empty.");
  }
  if (!currentProjectname) {
    throw new Error("Current Project name is not set");
  }

  const apiUrl = `/bt_studio/create_subtree/`;

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: currentProjectname,
        subtree_name: subtreeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const getSubtreeList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);
    console.log(response);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree list."); // Response error
    }

    if (! Array.isArray(response.data.subtree_list)) {
      throw new Error("API response is not an array")
    }

    return response.data.subtree_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getSubtree = async (subtreeName: string, projectName: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree?project_name=${encodeURIComponent(projectName)}&subtree_name=${encodeURIComponent(subtreeName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.subtree;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const saveSubtree = async (
  modelJson: string,
  currentProjectname: string,
  subtreeName: string
) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!subtreeName) throw new Error("Subtree name is not set");

  const apiUrl = "/bt_studio/save_subtree/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: currentProjectname,
        subtree_name: subtreeName,
        subtree_json: JSON.stringify(modelJson),
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const uploadFile = async (
  projectName: string,
  fileName: string,
  location: string,
  content: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (!location) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");

  const apiUrl = "/bt_studio/upload_code/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        file_name: fileName,
        location: location,
        content: content,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const createAction = async (
  projectName: string,
  fileName: string,
  template: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (!template) throw new Error("Template is not set");

  const apiUrl = "/bt_studio/create_action/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        filename: fileName,
        template: template,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const createFile = async (
  projectName: string,
  fileName: string,
  location: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (!location) throw new Error("Location is not set");

  const apiUrl = "/bt_studio/create_file/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        location: location,
        file_name: fileName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        console.log("Entering right thorugh max user size limit")
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const createFolder = async (
  projectName: string,
  folderName: string,
  location: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (!location) throw new Error("Location is not set");

  const apiUrl = "/bt_studio/create_folder/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        location: location,
        folder_name: folderName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      if (response.status == 507){
        throw new Error("You're using too much AWS space!")
      } else {
      throw new Error(response.data.message || "Failed to create project."); // Response error
      }
    }
  } catch (error: unknown) {
    console.log(error)
    throw error; // Rethrow
  }
};

const renameFile = async (
  projectName: string,
  path: string,
  new_path: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_file/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        path: path,
        rename_to: new_path,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }

    publish("updateActionList");
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const renameFolder = async (
  projectName: string,
  path: string,
  new_path: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_folder/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        path: path,
        rename_to: new_path,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const deleteFile = async (projectName: string, path: string) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_file/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        path: path,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }
    
    publish("updateActionList");
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const deleteFolder = async (projectName: string, path: string) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_folder/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_name: projectName,
        path: path,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const uploadUniverse = async (
  projectName: string,
  universeName: string,
  uploadedUniverse: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!universeName) throw new Error("Universe name is not set");
  if (!uploadedUniverse) throw new Error("Content is not set");

  const apiUrl = "/bt_studio/upload_universe/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        universe_name: universeName,
        zip_file: uploadedUniverse,
        app_name: projectName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to upload file."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const listDockerUniverses = async () => {
  const apiUrl = `/bt_studio/list_docker_universes`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.universes;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const listProjects = async () => {
  const apiUrl = `/bt_studio/get_project_list`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.project_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const listUniverses = async (projectName: string) => {
  if (!projectName) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_universes_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.universes_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getTreeStructure = async (projectName: string, btOrder: string) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_tree_structure?project_name=${encodeURIComponent(projectName)}&bt_order=${encodeURIComponent(btOrder)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.tree_structure;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getSubtreeStructure = async (
  projectName: string,
  subtreeName: string,
  btOrder: string
) => {
  if (!projectName) throw new Error("Current Project name is not set");
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_subtree_structure?project_name=${encodeURIComponent(projectName)}&subtree_name=${encodeURIComponent(subtreeName)}&bt_order=${encodeURIComponent(btOrder)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree."); // Response error
    }

    return response.data.tree_structure;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// Named export
export {
  createAction,
  createFile,
  createFolder,
  createProject,
  createRoboticsBackendUniverse,
  createSubtree,
  deleteFile,
  deleteFolder,
  deleteProject,
  deleteUniverse,
  generateDockerizedApp,
  generateLocalApp,
  getActionsList,
  getCustomUniverseZip,
  getFile,
  getFileList,
  getProjectGraph,
  getRoboticsBackendUniversePath,
  getSubtree,
  getSubtreeList,
  getSubtreeStructure,
  getTreeStructure,
  getUniverseConfig,
  listDockerUniverses,
  listProjects,
  listUniverses,
  loadProjectConfig,
  renameFile,
  renameFolder,
  saveBaseTree,
  saveFile,
  saveProjectConfig,
  saveSubtree,
  uploadFile,
  uploadUniverse,
};
