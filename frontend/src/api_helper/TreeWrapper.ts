import axios, { AxiosResponse, ResponseType } from "axios";
import { stringify } from "uuid";

// Helpers

const isSuccessful = (response: AxiosResponse) => {
  return response.status >= 200 && response.status < 300;
};

// File management

const getFileList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/tree_api/get_file_list?project_name=${encodeURIComponent(projectName)}`;

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

const getActionsList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/tree_api/get_actions_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get actions list."); // Response error
    }

    return response.data.actions_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// Project management

const createProject = async (projectName: string) => {
  if (!projectName.trim()) {
    throw new Error("Project name cannot be empty.");
  }

  const apiUrl = `/tree_api/create_project?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const saveProject = async (modelJson: string, currentProjectname: string) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = "/tree_api/save_project/";
  try {
    const response = await axios.post(apiUrl, {
      project_name: currentProjectname,
      graph_json: JSON.stringify(modelJson),
    });

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create project."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const loadProjectConfig = async (
  currentProjectname: string,
  settings: Object
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/tree_api/get_project_configuration?project_name=${currentProjectname}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project config"
      ); // Response error
    }

    // Extract the project settings from the response
    let raw_config = JSON.parse(response.data);
    let project_settings = raw_config.config;

    // Load all the settings
    Object.entries(settings).map(([key, value]) => {
      value.setter(
        project_settings[key] ? project_settings[key] : value.default_value
      );
    });
  } catch (error) {
    console.log("Loading default settings");
    Object.entries(settings).map(([key, value]) => {
      value.setter(value.default_value);
    });

    throw error; // Rethrow
  }
};

const getProjectGraph = async (currentProjectname: string) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/tree_api/get_project_graph?project_name=${currentProjectname}`;
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

  const apiUrl = `/tree_api/get_universe_configuration?project_name=${encodeURIComponent(
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

const getCustomUniverseZip = async (
  universeName: string,
  currentProjectname: string
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = "/tree_api/get_universe_zip/";
  try {
    // Configure the request options
    const config = {
      method: "POST",
      url: apiUrl,
      headers: {
        "Content-Type": "application/json",
      },
      data: JSON.stringify({
        app_name: currentProjectname,
        universe_name: universeName,
      }),
    };

    // Make the request
    const response = await axios(config);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve custom universe"
      ); // Response error
    }
    return new Blob([response.data], { type: "application/octet-stream" });
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// App management

const generateApp = async (
  modelJson: Object,
  currentProjectname: string,
  btOrder: string
) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = "/tree_api/generate_app/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        app_name: currentProjectname,
        tree_graph: JSON.stringify(modelJson),
        bt_order: btOrder,
      },
      {
        responseType: "blob", // Ensure the response is treated as a Blob
      }
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
  modelJson: Object,
  currentProjectname: string,
  btOrder: string
) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");
  if (!btOrder) throw new Error("Behavior Tree order is not set");

  const apiUrl = "/tree_api/generate_dockerized_app/";
  try {
    const response = await axios.post(
      apiUrl,
      {
        app_name: currentProjectname,
        tree_graph: JSON.stringify(modelJson),
        bt_order: btOrder,
      },
      {
        responseType: "blob", // Ensure the response is treated as a Blob
      }
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

  const apiUrl = `/tree_api/create_subtree/`;

  try {
    const response = await axios.post(apiUrl, {
      project_name: currentProjectname,
      subtree_name: subtreeName,
    });

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to create subtree."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getSubtreeList = async (projectName: string) => {
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/tree_api/get_subtree_list?project_name=${encodeURIComponent(projectName)}`;

  try {
    const response = await axios.get(apiUrl);
    console.log(response);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to get subtree list."); // Response error
    }

    return response.data.subtree_list;
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getSubtree = async (subtreeName: string, projectName: string) => {
  if (!subtreeName) throw new Error("Subtree name is not set");
  if (!projectName) throw new Error("Project name is not set");

  const apiUrl = `/tree_api/get_subtree?project_name=${encodeURIComponent(projectName)}&subtree_name=${encodeURIComponent(subtreeName)}`;

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

  const apiUrl = "/tree_api/save_subtree/";
  try {
    const response = await axios.post(apiUrl, {
      project_name: currentProjectname,
      subtree_name: subtreeName,
      subtree_json: JSON.stringify(modelJson),
    });

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(response.data.message || "Failed to save subtree."); // Response error
    }
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// Named export
export {
  createProject,
  saveProject,
  loadProjectConfig,
  getProjectGraph,
  generateApp,
  generateDockerizedApp,
  getUniverseConfig,
  getCustomUniverseZip,
  createSubtree,
  getSubtreeList,
  getSubtree,
  getFileList,
  getActionsList,
  saveSubtree,
};
