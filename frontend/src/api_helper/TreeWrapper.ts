import axios, { AxiosResponse } from "axios";

const isSuccessful = (response: AxiosResponse) => {
  return response.status >= 200 && response.status < 300;
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
  settings: Object,
) => {
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/tree_api/get_project_configuration?project_name=${currentProjectname}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve project config",
      ); // Response error
    }

    // Extract the project settings from the response
    let raw_config = JSON.parse(response.data);
    let project_settings = raw_config.config;

    // Load all the settings
    Object.entries(settings).map(([key, value]) => {
      value.setter(
        project_settings[key] ? project_settings[key] : value.default_value,
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

// Universe management

const getUniverseConfig = async (
  universeName: string,
  currentProjectname: string,
) => {
  if (!universeName) throw new Error("The universe name is not set");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  const apiUrl = `/tree_api/get_universe_configuration?project_name=${encodeURIComponent(
    currentProjectname,
  )}&universe_name=${encodeURIComponent(universeName)}`;
  try {
    const response = await axios.get(apiUrl);

    // Handle unsuccessful response status (e.g., non-2xx status)
    if (!isSuccessful(response)) {
      throw new Error(
        response.data.message || "Failed to retrieve universe config",
      ); // Response error
    }

    return JSON.stringify(response.data.config);
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

const getCustomUniverseZip = async (
  universeName: string,
  currentProjectname: string,
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
        response.data.message || "Failed to retrieve custom universe",
      ); // Response error
    }
    return new Blob([response.data], { type: "application/octet-stream" });
  } catch (error: unknown) {
    throw error; // Rethrow
  }
};

// App management

// const generateApp = async (
//   modelJson: string,
//   currentProjectname: string,
//   btOrder: string,
// ) => {
//   if (!modelJson) throw new Error("Tree JSON is empty!");
//   if (!currentProjectname) throw new Error("Current Project name is not set");

//   const apiUrl = "/tree_api/generate_app/";
//   try {
//     // Configure the request options
//     const config = {
//       method: "POST",
//       url: apiUrl,
//       headers: {
//         "Content-Type": "application/json",
//       },
//       data: {
//         app_name: currentProjectname,
//         tree_graph: JSON.stringify(modelJson),
//         bt_order: btOrder,
//       },
//     };

//     // Make the request
//     const response = await axios(config);
//     console.log(response.status);

//     // Handle unsuccessful response status (e.g., non-2xx status)
//     if (!isSuccessful(response)) {
//       throw new Error(response.data.message || "Failed to create app."); // Response error
//     }
//     return new Blob([response.data], { type: "application/zip" });
//   } catch (error: unknown) {
//     throw error; // Rethrow
//   }
// };

// const generateDockerizedApp = async (
//   modelJson: string,
//   currentProjectname: string,
//   btOrder: string,
// ) => {
//   if (!modelJson) throw new Error("Tree JSON is empty!");
//   if (!currentProjectname) throw new Error("Current Project name is not set");

//   const apiUrl = "/tree_api/generate_dockerized_app/";
//   try {
//     // Configure the request options
//     const config = {
//       method: "POST",
//       url: apiUrl,
//       headers: {
//         "Content-Type": "application/json",
//       },
//       data: {
//         app_name: currentProjectname,
//         tree_graph: JSON.stringify(modelJson),
//         bt_order: btOrder,
//       },
//     };

//     // Make the request
//     const response = await axios(config);
//     console.log(response.status);

//     // Handle unsuccessful response status (e.g., non-2xx status)
//     if (!isSuccessful(response)) {
//       throw new Error(response.data.message || "Failed to create app."); // Response error
//     }
//     return new Blob([response.data], { type: "application/zip" });
//   } catch (error: unknown) {
//     throw error; // Rethrow
//   }
// };

const generateApp = async (
  modelJson: string,
  currentProjectname: string,
  btOrder: string,
  dockerized: boolean = false,
) => {
  if (!modelJson) throw new Error("Tree JSON is empty!");
  if (!currentProjectname) throw new Error("Current Project name is not set");

  var apiUrl = "/tree_api/generate_app/";

  if (dockerized) {
    apiUrl = "/tree_api/generate_dockerized_app/";
  }

  const api_response = await fetch(apiUrl, {
    method: "POST",
    headers: {
      "Content-Type": "application/json",
    },
    body: JSON.stringify({
      app_name: currentProjectname,
      tree_graph: JSON.stringify(modelJson),
      bt_order: btOrder,
    }),
  });

  if (!api_response.ok) {
    var json_response = await api_response.json();
    throw new Error(json_response.message || "An error occurred.");
  }

  return api_response.blob();
};

// Named export
export {
  createProject,
  saveProject,
  loadProjectConfig,
  generateApp,
  // generateDockerizedApp,
  getUniverseConfig,
  getCustomUniverseZip,
};
