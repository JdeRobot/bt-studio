import axios, { AxiosError, AxiosResponse } from "axios";
import { SettingsData } from "BtComponents/options/Options";
import { publish } from "BtComponents/helper/TreeEditorHelper";

// Helpers

const isSuccessful = (response: AxiosResponse) => {
  return response.status >= 200 && response.status < 300;
};
const getCookie = (name: string) => {
  const value = `; ${document.cookie}`;
  const parts = value.split(`; ${name}=`); //to get CSRF token among all the cookies in 'value'
  if (parts.length === 2) return parts.pop()?.split(";").shift();
  return undefined;
};

const csrfToken = getCookie("csrftoken");
const axiosExtra = {
  headers: {
    "X-CSRFToken": csrfToken,
  },
};

////////////////////////////// User management /////////////////////////////////

const getUserInfo = async () => {
  const apiUrl = `/bt_studio/get_user_size`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data;
};

//////////////////////////// Project management ////////////////////////////////

const createProject = async (projectName: string) => {
  if (!projectName.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/create_project/`;

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
};

const deleteProject = async (projectId: string) => {
  if (!projectId.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/delete_project/`;

  const response = await axios.post(
    apiUrl,
    {
      project_id: projectId,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to delete project."); // Response error
  }
};

const getProjectInfo = async (projectId: string) => {
  if (!projectId.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/get_project_info/?project_id=${projectId}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data;
};

const listProjects = async () => {
  const apiUrl = `/bt_studio/get_project_list`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.project_list;
};

const getProjectConfig = async (
  currentProjectname: string,
  settings: SettingsData
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_configuration?project_id=${currentProjectname}`;

  try {
    const response = await axios.get(apiUrl);

    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project config"
      );
    }

    const raw_config = JSON.parse(response.data);
    const project_settings = raw_config.config;

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

const getProjectConfigRaw = async (projectId: string) => {
  if (!projectId) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_configuration?project_id=${projectId}`;

  try {
    const response = await axios.get(apiUrl);

    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project config"
      );
    }

    const raw_config = JSON.parse(response.data);

    return raw_config.config;
  } catch (error) {
    console.log("Loading default settings");
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
        project_id: currentProjectname,
        settings: settings,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

//////////////////////////// Universe management ///////////////////////////////

const createEmptyUniverse = async (projectId: string, universeName: string) => {
  if (!projectId) throw new Error("The universe name is not set");
  if (!universeName) throw new Error("The universe name is not set");
  if (!projectId.trim()) throw new Error("Project name cannot be empty.");
  if (!universeName.trim()) throw new Error("Project name cannot be empty.");

  const apiUrl = `/bt_studio/create_universe/`;

  const response = await axios.post(
    apiUrl,
    {
      project_id: projectId,
      universe: universeName,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create project."); // Response error
  }
};

const getUniverseConfig = async (
  universeName: string,
  currentProjectname: string
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_universe_configuration?project_id=${encodeURIComponent(
    currentProjectname
  )}&universe_name=${encodeURIComponent(universeName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to retrieve universe config"
    ); // Response error
  }

  return JSON.stringify(response.data.config);
};

const createUniverseConfig = async (
  currentProjectname: string,
  universeName: string
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = "/bt_studio/create_universe_configuration/";

  const response = await axios.post(
    apiUrl,
    {
      project_id: currentProjectname,
      universe_name: universeName,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to create universe config"
    ); // Response error
  }
};

const getRoboticsBackendUniverse = async (universeName: string) => {
  if (!universeName) throw new Error("The universe name is not set");

  const apiUrl = `/bt_studio/get_docker_universe_data?name=${encodeURIComponent(universeName)}`;

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
    tools: response.data.universe.tools,
    tools_config: response.data.universe.tools_config,
  };
};

const createRoboticsBackendUniverse = async (
  projectId: string,
  universeName: string,
  universeId: string
) => {
  if (!projectId) throw new Error("The project name is not set");
  if (!universeName) throw new Error("The universe name is not set");
  if (!universeId) throw new Error("The universe id is not set");

  const apiUrl = "/bt_studio/add_docker_universe/";

  const response = await axios.post(
    apiUrl,
    {
      project_id: projectId,
      universe_name: universeName,
      id: universeId,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to save subtree."); // Response error
  }
};

const deleteUniverse = async (projectId: string, universeName: string) => {
  if (!projectId) throw new Error("The project name is not set");
  if (!universeName) throw new Error("The universe name is not set");

  const apiUrl = "/bt_studio/delete_universe/";

  const response = await axios.post(
    apiUrl,
    {
      project_id: projectId,
      universe_name: universeName,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to save subtree."); // Response error
  }
};

const listUniverses = async (projectId: string) => {
  if (!projectId) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_universes_list?project_id=${encodeURIComponent(projectId)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.universes_list;
};

const createCustomUniverse = async (
  projectId: string,
  universeName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!universeName) throw new Error("Universe name is not set");

  const apiUrl = "/bt_studio/create_custom_universe/";

  const response = await axios.post(
    apiUrl,
    {
      project_id: projectId,
      universe_name: universeName,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }
};

const listDockerUniverses = async () => {
  const apiUrl = `/bt_studio/list_docker_universes`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.universes;
};

////////////////////////////// App management //////////////////////////////////

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
        project_id: currentProjectname,
        bt_order: btOrder,
      },
      axiosExtra
    );
    return response.data;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(error.response?.data.error || "Failed to create app.");
  }
};

const generateDockerizedApp = async (
  currentProjectname: string,
  btOrder: string
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/generate_dockerized_app/`;

  const response = await axios.post(
    apiUrl,
    {
      project_id: currentProjectname,
      bt_order: btOrder,
    },
    axiosExtra
  );

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to create app."); // Response error
  }

  return response.data;
};

////////////////////////////// Tree management /////////////////////////////////

const getBaseTree = async (currentProjectname: string) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_base_tree?project_id=${currentProjectname}`;
  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to retrieve project graph"
    ); // Response error
  }

  return response.data.graph_json;
};

const getTreeStructure = async (projectId: string, btOrder: string) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_tree_structure?project_id=${encodeURIComponent(projectId)}&bt_order=${encodeURIComponent(btOrder)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.tree_structure;
  } catch (e: unknown) {
    const error = e as AxiosError<any, Record<string, unknown>>;
    throw Error(
      error.response?.data.error || "Failed to get subtree structure."
    );
  }
};

///////////////////////////// Subtree management ///////////////////////////////

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
        project_id: currentProjectname,
        subtree_name: subtreeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

const getSubtree = async (subtreeName: string, projectId: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!projectId) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree?project_id=${encodeURIComponent(projectId)}&subtree_name=${encodeURIComponent(subtreeName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.subtree;
};

const getSubtreePath = async (projectId: string, subtreeName: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!projectId) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree_path?project_id=${encodeURIComponent(projectId)}&subtree_name=${encodeURIComponent(subtreeName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.subtree;
};

const getSubtreeList = async (projectId: string) => {
  if (!projectId) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree_list?project_id=${encodeURIComponent(projectId)}`;

  const response = await axios.get(apiUrl);
  console.log(response);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree list."); // Response error
  }

  if (!Array.isArray(response.data.subtree_list)) {
    throw new Error("API response is not an array");
  }

  return response.data.subtree_list;
};

const getSubtreeStructure = async (
  projectId: string,
  subtreeName: string,
  btOrder: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_subtree_structure?project_id=${encodeURIComponent(projectId)}&subtree_name=${encodeURIComponent(subtreeName)}&bt_order=${encodeURIComponent(btOrder)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree."); // Response error
  }

  return response.data.tree_structure;
};

////////////////////////////// File management /////////////////////////////////

const createFile = async (
  projectId: string,
  fileName: string,
  location: string,
  universeName: string | undefined = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");

  const apiUrl = "/bt_studio/create_file/";

  if (location.startsWith("trees")) throw new Error("Cannot create a tree this way");

  let params = {
    project_id: projectId,
    location: location,
    file_name: fileName,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  try {
    const response = await axios.post(apiUrl, params, axiosExtra);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

const createAction = async (
  projectId: string,
  fileName: string,
  template: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (!template) throw new Error("Template is not set");

  const apiUrl = "/bt_studio/create_action/";

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_id: projectId,
        filename: fileName,
        template: template,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

const getFile = async (
  projectId: string,
  fileName: string,
  universeName: string | undefined = undefined
) => {
  if (!projectId) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/bt_studio/get_file?project_id=${encodeURIComponent(projectId)}&filename=${encodeURIComponent(fileName)}`;

  if (universeName !== undefined)
    apiUrl += `&universe=${encodeURIComponent(universeName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.content;
};

const getUniverseFile = async (
  projectId: string,
  universeName: string,
  fileName: string
) => {
  if (!projectId) throw new Error("Project name is not set");
  if (!universeName) throw new Error("Universe name is not set");
  if (!fileName) throw new Error("File name is not set");

  return await getFile(projectId, fileName, universeName);
};

const saveFile = async (
  projectId: string,
  fileName: string,
  content: string,
  universeName: string | undefined = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("Current File name is not set");

  const apiUrl = "/bt_studio/save_file/";

  let params = {
    project_id: projectId,
    content: content,
    filename: fileName,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  try {
    const response = await axios.post(apiUrl, params, axiosExtra);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error) {
    console.log(error);
    throw error; // Rethrow
  }
};

const renameFile = async (
  projectId: string,
  path: string,
  new_path: string,
  universeName: string | undefined = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_file/";

  let params = {
    project_id: projectId,
    path: path,
    rename_to: new_path,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }

  publish("updateActionList");
};

const deleteFile = async (
  projectId: string,
  path: string,
  universeName: string | undefined = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_file/";

  let params = {
    project_id: projectId,
    path: path,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }

  publish("updateActionList");
};

const uploadFile = async (
  projectId: string,
  fileName: string,
  location: string,
  content: string,
  universeName: undefined | string = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");

  const apiUrl = "/bt_studio/upload_code/";
  let params = {
    project_id: projectId,
    file_name: fileName,
    location: location,
    content: content,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }
};

const uploadFileUniverse = async (
  projectId: string,
  fileName: string,
  location: string,
  content: string,
  universeName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location !== undefined) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");
  if (!universeName) throw new Error("Universe name is not set");

  return await uploadFile(projectId, fileName, location, content, universeName);
};

const getFileList = async (
  projectId: string,
  universeName: string | undefined = undefined
): Promise<string> => {
  if (!projectId) throw new Error("Project name is not set");

  let apiUrl = `/bt_studio/get_file_list?project_id=${encodeURIComponent(projectId)}`;

  if (universeName !== undefined)
    apiUrl += `&universe=${encodeURIComponent(universeName)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get file list."); // Response error
  }

  return response.data.file_list;
};

const getActionsList = async (projectId: string) => {
  if (!projectId) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_actions_list?project_id=${encodeURIComponent(projectId)}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get actions list."); // Response error
  }

  if (!Array.isArray(response.data.actions_list)) {
    throw new Error("API response is not an array");
  }

  return response.data.actions_list;
};

///////////////////////////// Folder management ////////////////////////////////

const createFolder = async (
  projectId: string,
  location: string,
  folderName: string,
  universeName: undefined | string = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (location === undefined) throw new Error("Location is not set");

  if (location.startsWith("trees")) throw new Error("Cannot create a subtree this way");

  const apiUrl = "/bt_studio/create_folder/";

  let params = {
    project_id: projectId,
    location: location,
    folder_name: folderName,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  try {
    const response = await axios.post(apiUrl, params, axiosExtra);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

const createUniverseFolder = async (
  projectId: string,
  folderName: string,
  location: string,
  universeName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (location === undefined) throw new Error("Location is not set");
  if (!universeName) throw new Error("Universe name is not set");

  return await createFolder(projectId, folderName, location, universeName);
};

const renameFolder = async (
  projectId: string,
  path: string,
  new_path: string,
  universeName: undefined | string = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_folder/";

  let params = {
    project_id: projectId,
    path: path,
    rename_to: new_path,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }
};

const deleteFolder = async (
  projectId: string,
  path: string,
  universeName: undefined | string = undefined
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_folder/";

  let params = {
    project_id: projectId,
    path: path,
  };

  if (universeName !== undefined)
    params = Object.assign({}, params, { universe: universeName });

  const response = await axios.post(apiUrl, params, axiosExtra);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to upload file."); // Response error
  }
};

const getLibraryTree = async (entry: string) => {
  if (!entry) throw new Error("Current Library Tree name is not set");

  const apiUrl = `/bt_studio/get_library_tree?entry=${entry}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to retrieve project graph"
    ); // Response error
  }

  return {
    graph_json: response.data.graph_json,
    btOrder: response.data.bt_order,
    actions: response.data.actions,
    subtrees: response.data.subtrees,
  };
};

const getSubtreeLibrary = async () => {
  const apiUrl = `/bt_studio/get_subtree_library_list`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree list."); // Response error
  }

  if (!Array.isArray(response.data.subtree_list)) {
    throw new Error("API response is not an array");
  }

  return response.data.subtree_list;
};

const getUserSubtreeLibrary = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_user_subtree_library_list?project=${project}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(response.data.message || "Failed to get subtree list."); // Response error
  }

  if (!Array.isArray(response.data.library)) {
    throw new Error("API response is not an array");
  }

  return response.data.library;
};

const getUserLibraryTree = async (project: string, entry: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");

  const apiUrl = `/bt_studio/get_user_library_tree?project=${project}&entry=${entry}`;

  const response = await axios.get(apiUrl);

  // Handle unsuccessful response status (e.g., non-2xx status)
  if (!isSuccessful(response)) {
    throw new Error(
      response.data.message || "Failed to retrieve project graph"
    ); // Response error
  }

  return {
    graph_json: response.data.graph_json,
    btOrder: response.data.bt_order,
    actions: response.data.actions,
    subtrees: response.data.subtrees,
  };
};

const importLibrarySubtree = async (
  projectId: string,
  entry: string,
  subtreeName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");
  if (!subtreeName) throw new Error("Subtree name is not set");

  const apiUrl = `/bt_studio/import_library_tree/`;

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_id: projectId,
        entry: entry,
        name: subtreeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to import subtree."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

const importUserLibrarySubtree = async (
  projectId: string,
  entry: string,
  entryProject: string,
  subtreeName: string
) => {
  if (!projectId) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");
  if (!entryProject) throw new Error("Entry Project name is not set");
  if (!subtreeName) throw new Error("Subtree name is not set");

  const apiUrl = `/bt_studio/import_user_library_tree/`;

  try {
    const response = await axios.post(
      apiUrl,
      {
        project_id: projectId,
        entry: entry,
        entry_project: entryProject,
        name: subtreeName,
      },
      axiosExtra
    );

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to import subtree."); // Response error
    }
  } catch (error: unknown) {
    console.log(error);
    throw error; // Rethrow
  }
};

////////////////////////////////// Exports /////////////////////////////////////
export {
  createAction,
  createCustomUniverse,
  createEmptyUniverse,
  createFile,
  createFolder,
  createProject,
  createRoboticsBackendUniverse,
  createSubtree,
  createUniverseConfig,
  createUniverseFolder,
  deleteFile,
  deleteFolder,
  deleteProject,
  deleteUniverse,
  generateDockerizedApp,
  generateLocalApp,
  getActionsList,
  getBaseTree,
  getFile,
  getFileList,
  getProjectConfig,
  getProjectInfo,
  getRoboticsBackendUniverse,
  getSubtree,
  getSubtreeLibrary,
  getSubtreeList,
  getSubtreeStructure,
  getTreeStructure,
  getUniverseConfig,
  getUniverseFile,
  listDockerUniverses,
  listProjects,
  listUniverses,
  renameFile,
  renameFolder,
  saveFile,
  saveProjectConfig,
  uploadFile,
  uploadFileUniverse,
  getSubtreePath,
  getLibraryTree,
  getUserSubtreeLibrary,
  getUserLibraryTree,
  importLibrarySubtree,
  importUserLibrarySubtree,
  getProjectConfigRaw,
  getUserInfo
};
