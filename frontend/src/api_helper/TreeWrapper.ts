import axios, { AxiosError } from "axios";
import { SettingsData } from "BtContexts/ProjectSettingsContext";
import { publish } from "BtHelpers/utils";
import { BTWorldData } from "BtTypes/index";

// Helpers
const getCookie = (name: string) => {
  const value = `; ${document.cookie}`;
  const parts = value.split(`; ${name}=`); //to get CSRF token among all the cookies in 'value'
  if (parts.length === 2) return parts.pop()?.split(";").shift();
  return undefined;
};

const axiosExtra = () => ({
  headers: {
    "X-CSRFToken": getCookie("csrftoken"),
  },
});

type ApiError = AxiosError<Record<string, string>, Record<string, unknown>>;

////////////////////////////// User management /////////////////////////////////

const getUserInfo = async () => {
  const apiUrl = `/bt_studio/get_user_size/`;

  try {
    const response = await axios.get(apiUrl);
    return response.data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

//////////////////////////// Project management ////////////////////////////////

const createProject = async (projectName: string) => {
  if (!projectName.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = "/bt_studio/create_project/";

  const params: { [key: string]: string } = {
    project_name: projectName,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const deleteProject = async (projectId: string) => {
  if (!projectId.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = "/bt_studio/delete_project/";

  const params: { [key: string]: string } = {
    project_id: projectId,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getProjectInfo = async (projectId: string) => {
  if (!projectId.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/bt_studio/get_project_info/?project_id=${encodeURIComponent(projectId)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const listProjects = async () => {
  const apiUrl = `/bt_studio/get_project_list/`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.project_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getProjectConfig = async (project: string, settings: SettingsData) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_configuration?project_id=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);

    const raw_config = JSON.parse(response.data);
    const project_settings = raw_config.config;

    Object.entries(settings).forEach(([key, value]) => {
      value.setter(
        project_settings[key] ? project_settings[key] : value.default_value,
      );
    });
  } catch (e: unknown) {
    const error = e as ApiError;
    Object.entries(settings).forEach(([, value]) => {
      value.setter(value.default_value);
    });
    throw Error(error.response?.data.message);
  }
};

const getProjectConfigRaw = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_project_configuration?project_id=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return JSON.parse(response.data);
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const saveProjectConfig = async (project: string, settings: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!settings) throw new Error("Settings content is null");

  const apiUrl = "/bt_studio/save_project_configuration/";

  const params: { [key: string]: string } = {
    project_id: project,
    settings: settings,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

//////////////////////////// World management ///////////////////////////////

const createEmptyWorld = async (project: string, worldName: string) => {
  if (!project) throw new Error("The project name is not set");
  if (!worldName) throw new Error("The world name is not set");
  if (!project.trim()) throw new Error("Project name cannot be empty.");
  if (!worldName.trim()) throw new Error("Project name cannot be empty.");

  const apiUrl = "/bt_studio/create_world/";

  const params: { [key: string]: string } = {
    project_id: project,
    world: worldName,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getWorldConfig = async (
  project: string,
  world: string,
): Promise<BTWorldData> => {
  if (!project) throw new Error("Current Project name is not set");
  if (!world) throw new Error("The world name is not set");

  const apiUrl = `/bt_studio/get_world_configuration?project=${encodeURIComponent(
    project,
  )}&world=${encodeURIComponent(world)}`;

  try {
    const response = await axios.get(apiUrl);
    return {
      name: response.data.world.name,
      isCustom: response.data.world.custom,
      config: {
        scene: response.data.world.scene,
        robot: response.data.world.robot,
        tools: response.data.world.tools,
        tools_config: response.data.world.tools_config,
      },
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createWorldConfig = async (project: string, world: string) => {
  if (!world) throw new Error("The world name is not set");
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = "/bt_studio/create_world_configuration/";

  const params: { [key: string]: string } = {
    project: project,
    world: world,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createRoboticsBackendWorld = async (
  project: string,
  world: string,
  worldId: string,
) => {
  if (!project) throw new Error("The project name is not set");
  if (!world) throw new Error("The world name is not set");
  if (!worldId) throw new Error("The world id is not set");

  const apiUrl = "/bt_studio/add_docker_world/";

  const params: { [key: string]: string } = {
    project: project,
    world: world,
    id: worldId,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const deleteWorld = async (project: string, world: string) => {
  if (!project) throw new Error("The project name is not set");
  if (!world) throw new Error("The world name is not set");

  const apiUrl = "/bt_studio/delete_world/";

  const params: { [key: string]: string } = {
    project_id: project,
    world: world,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const listWorlds = async (projectId: string) => {
  if (!projectId) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_world_list?project_id=${encodeURIComponent(projectId)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.worlds_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createCustomWorld = async (project: string, world: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!world) throw new Error("World name is not set");

  const apiUrl = "/bt_studio/create_custom_world/";

  const params: { [key: string]: string } = {
    project: project,
    world: world,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const listDockerWorlds = async () => {
  const apiUrl = `/bt_studio/list_docker_worlds`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.worlds;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const listDockerRobots = async () => {
  const apiUrl = "/bt_studio/list_docker_robots/";
  try {
    const response = await axios.get(apiUrl);
    return response.data.robots;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createCombinedUniverse = async (
  projectId: string,
  universeName: string,
  worldId: string,
  robotId: string,
  startPose: number[] | null = null
) => {
  if (!projectId) throw new Error("The project name is not set");
  if (!universeName) throw new Error("The universe name is not set");
  if (!worldId) throw new Error("The world is not set");

  const apiUrl = "/bt_studio/create_combined_universe/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project_id: projectId,
        universe_name: universeName,
        world_id: worldId,
        robot_id: robotId,
        start_pose: startPose,
      },
      axiosExtra()
    );
    return response.data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getCombinedUniverseData = async (worldId: string, robotId: string) => {
  const apiUrl = `/bt_studio/get_combined_universe_data/?world_id=${encodeURIComponent(
    worldId
  )}&robot_id=${encodeURIComponent(robotId || "None")}`;
  try {
    const response = await axios.get(apiUrl);
    return {
      world: response.data.universe.world,
      robot: response.data.universe.robot,
      tools: response.data.universe.tools,
      tools_config: response.data.universe.tools_config,
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const captureRobotPose = async () => {
  const apiUrl = "/bt_studio/capture_robot_pose/";
  try {
    const response = await axios.get(apiUrl);
    return response.data.pose;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const saveCurrentPose = async (projectId: string, universeName: string) => {
  if (!projectId) throw new Error("Project name is not set");
  if (!universeName) throw new Error("Universe name is not set");

  const apiUrl = "/bt_studio/save_current_pose/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        project_id: projectId,
        universe_name: universeName,
      },
      axiosExtra()
    );
    return response.data.pose;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};


////////////////////////////// App management //////////////////////////////////

const generateLocalApp = async (project: string, btOrder: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = "/bt_studio/generate_local_app/";

  const params: { [key: string]: string } = {
    project_id: project,
    bt_order: btOrder,
  };

  try {
    const response = await axios.post(apiUrl, params, axiosExtra());
    return response.data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const generateDockerizedApp = async (project: string, btOrder: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = "/bt_studio/generate_dockerized_app/";

  const params: { [key: string]: string } = {
    project_id: project,
    bt_order: btOrder,
  };

  try {
    const response = await axios.post(apiUrl, params, axiosExtra());
    return response.data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

////////////////////////////// Tree management /////////////////////////////////

const getTreeData = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_tree_data?project_id=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.actions_data;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getBaseTree = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_base_tree?project_id=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.graph_json;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getTreeStructure = async (project: string, btOrder: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_tree_structure?project_id=${encodeURIComponent(project)}&bt_order=${encodeURIComponent(btOrder)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.tree_structure;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

///////////////////////////// Subtree management ///////////////////////////////

const createSubtree = async (subtreeName: string, project: string) => {
  if (!subtreeName.trim()) throw new Error("Subtree name cannot be empty.");
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = "/bt_studio/save_file/";

  const params: { [key: string]: string } = {
    project_id: project,
    subtree_name: subtreeName,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getSubtree = async (subtreeName: string, project: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!project) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree?project_id=${encodeURIComponent(project)}&subtree_name=${encodeURIComponent(subtreeName)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.subtree;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getSubtreePath = async (project: string, subtreeName: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!project) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree_path?project_id=${encodeURIComponent(project)}&subtree_name=${encodeURIComponent(subtreeName)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.subtree;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getSubtreeList = async (project: string) => {
  if (!project) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_subtree_path?project_id=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.subtree_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getSubtreeStructure = async (
  project: string,
  subtree: string,
  btOrder: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!subtree) throw new Error("Subtree name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = `/bt_studio/get_subtree_structure?project_id=${encodeURIComponent(
    project,
  )}&subtree_name=${encodeURIComponent(
    subtree,
  )}&bt_order=${encodeURIComponent(btOrder)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.tree_structure;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

////////////////////////////// File management /////////////////////////////////

const createFile = async (
  project: string,
  fileName: string,
  location: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");

  const apiUrl = "/bt_studio/create_file/";

  if (location.startsWith("trees"))
    throw new Error("Cannot create a tree this way");

  const params: { [key: string]: string } = {
    project: project,
    location: location,
    filename: fileName,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createAction = async (
  project: string,
  fileName: string,
  template: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (!template) throw new Error("Template is not set");

  const apiUrl = "/bt_studio/create_action/";

  const params: { [key: string]: string } = {
    project_id: project,
    filename: fileName,
    template: template,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getFile = async (
  project: string,
  fileName: string,
  world?: string,
  binary?: boolean,
) => {
  if (!project) throw new Error("Project name is not set");
  if (!fileName) throw new Error("File name is not set");

  let apiUrl = `/bt_studio/get_file?project=${encodeURIComponent(
    project,
  )}&filename=${encodeURIComponent(fileName)}`;

  if (world !== undefined) apiUrl += `&world=${encodeURIComponent(world)}`;
  if (binary) apiUrl += `&binary=true`;

  try {
    const response = await axios.get(apiUrl);
    if (binary) {
      return atob(response.data.content);
    }
    return response.data.content;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getWorldFile = async (
  project: string,
  world: string,
  fileName: string,
) => {
  if (!project) throw new Error("Project name is not set");
  if (!world) throw new Error("World name is not set");
  if (!fileName) throw new Error("File name is not set");

  return await getFile(project, fileName, world);
};

const saveFile = async (
  project: string,
  fileName: string,
  content: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("Current File name is not set");

  const apiUrl = "/bt_studio/save_file/";

  const params: { [key: string]: string } = {
    project: project,
    content: content,
    filename: fileName,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const renameFile = async (
  project: string,
  path: string,
  new_path: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_file/";

  const params: { [key: string]: string } = {
    project: project,
    path: path,
    rename_to: new_path,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
    publish("updateActionList");
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const deleteFile = async (project: string, path: string, world?: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_file/";

  const params: { [key: string]: string } = {
    project: project,
    path: path,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
    publish("updateActionList");
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const uploadFile = async (
  project: string,
  fileName: string,
  location: string,
  content: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location === undefined) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");

  const apiUrl = "/bt_studio/upload/";
  const params: { [key: string]: string } = {
    project: project,
    filename: fileName,
    location: location,
    content: content,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const uploadFileWorld = async (
  project: string,
  fileName: string,
  location: string,
  content: string,
  world: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!fileName) throw new Error("File name is not set");
  if (location !== undefined) throw new Error("Location is not set");
  if (!content) throw new Error("Content is not defined");
  if (!world) throw new Error("World name is not set");

  return await uploadFile(project, fileName, location, content, world);
};

const getFileList = async (
  project: string,
  world?: string,
): Promise<string> => {
  if (!project) throw new Error("Project name is not set");

  let apiUrl = `/bt_studio/get_file_list?project=${encodeURIComponent(project)}`;

  if (world !== undefined) apiUrl += `&world=${encodeURIComponent(world)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.file_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.file_list);
  }
};

const getActionsList = async (project: string) => {
  if (!project) throw new Error("Project name is not set");

  const apiUrl = `/bt_studio/get_actions_list?project=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.actions_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.file_list);
  }
};

///////////////////////////// Folder management ////////////////////////////////

const createFolder = async (
  project: string,
  location: string,
  folderName: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (location === undefined) throw new Error("Location is not set");

  if (location.startsWith("trees"))
    throw new Error("Cannot create a subtree this way");

  const apiUrl = "/bt_studio/create_folder/";

  const params: { [key: string]: string } = {
    project: project,
    location: location,
    folder_name: folderName,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const createWorldFolder = async (
  project: string,
  folderName: string,
  location: string,
  world: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!folderName) throw new Error("Folder name is not set");
  if (location === undefined) throw new Error("Location is not set");
  if (!world) throw new Error("World name is not set");

  return await createFolder(project, folderName, location, world);
};

const renameFolder = async (
  project: string,
  path: string,
  new_path: string,
  world?: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");
  if (!new_path) throw new Error("New path is not set");

  const apiUrl = "/bt_studio/rename_folder/";

  const params: { [key: string]: string } = {
    project: project,
    path: path,
    rename_to: new_path,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const deleteFolder = async (project: string, path: string, world?: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!path) throw new Error("Path is not set");

  const apiUrl = "/bt_studio/delete_folder/";

  const params: { [key: string]: string } = {
    project: project,
    path: path,
  };

  if (world !== undefined) params["world"] = world;

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getLibraryTree = async (entry: string) => {
  if (!entry) throw new Error("Current Library Tree name is not set");

  const apiUrl = `/bt_studio/get_library_tree?entry=${encodeURIComponent(entry)}`;

  try {
    const response = await axios.get(apiUrl);
    return {
      graph_json: response.data.graph_json,
      btOrder: response.data.bt_order,
      actions: response.data.actions,
      subtrees: response.data.subtrees,
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getSubtreeLibrary = async () => {
  const apiUrl = `/bt_studio/get_subtree_library_list`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.subtree_list;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getUserSubtreeLibrary = async (project: string) => {
  if (!project) throw new Error("Current Project name is not set");

  const apiUrl = `/bt_studio/get_user_subtree_library_list?project=${encodeURIComponent(project)}`;

  try {
    const response = await axios.get(apiUrl);
    return response.data.library;
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const getUserLibraryTree = async (project: string, entry: string) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");

  const apiUrl = `/bt_studio/get_user_library_tree?project=${encodeURIComponent(
    project,
  )}&entry=${encodeURIComponent(entry)}`;

  try {
    const response = await axios.get(apiUrl);
    return {
      graph_json: response.data.graph_json,
      btOrder: response.data.bt_order,
      actions: response.data.actions,
      subtrees: response.data.subtrees,
    };
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const importLibrarySubtree = async (
  project: string,
  entry: string,
  subtree: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");
  if (!subtree) throw new Error("Subtree name is not set");

  const apiUrl = `/bt_studio/import_library_tree/`;

  const params: { [key: string]: string } = {
    project_id: project,
    entry: entry,
    name: subtree,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

const importUserLibrarySubtree = async (
  project: string,
  entry: string,
  entryProject: string,
  subtree: string,
) => {
  if (!project) throw new Error("Current Project name is not set");
  if (!entry) throw new Error("Current Library Tree name is not set");
  if (!entryProject) throw new Error("Entry Project name is not set");
  if (!subtree) throw new Error("Subtree name is not set");

  const apiUrl = `/bt_studio/import_user_library_tree/`;

  const params: { [key: string]: string } = {
    project_id: project,
    entry: entry,
    entry_project: entryProject,
    name: subtree,
  };

  try {
    await axios.post(apiUrl, params, axiosExtra());
  } catch (e: unknown) {
    const error = e as ApiError;
    throw Error(error.response?.data.message);
  }
};

////////////////////////////////// Exports /////////////////////////////////////
export {
  createAction,
  createCustomWorld,
  createEmptyWorld,
  createFile,
  createFolder,
  createProject,
  createRoboticsBackendWorld,
  createSubtree,
  createWorldConfig,
  createWorldFolder,
  deleteFile,
  deleteFolder,
  deleteProject,
  deleteWorld,
  generateDockerizedApp,
  generateLocalApp,
  getActionsList,
  getBaseTree,
  getFile,
  getFileList,
  getProjectConfig,
  getProjectInfo,
  getSubtree,
  getSubtreeLibrary,
  getSubtreeList,
  getSubtreeStructure,
  getTreeStructure,
  getWorldConfig,
  getWorldFile,
  listDockerWorlds,
  listDockerRobots,
  createCombinedUniverse,
  getCombinedUniverseData,
  captureRobotPose,
  saveCurrentPose,
  listProjects,

  listWorlds,
  renameFile,
  renameFolder,
  saveFile,
  saveProjectConfig,
  uploadFile,
  uploadFileWorld,
  getSubtreePath,
  getLibraryTree,
  getUserSubtreeLibrary,
  getUserLibraryTree,
  importLibrarySubtree,
  importUserLibrarySubtree,
  getProjectConfigRaw,
  getUserInfo,
  getTreeData,
};
