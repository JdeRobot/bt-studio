import JSZip from "jszip";
import { getFileList, getUniverseFile } from "../../api_helper/TreeWrapper";
import { Entry } from "jderobot-ide-interface";

const zipFile = async (
  zip: JSZip,
  project: string,
  universe_name: string,
  file_path: string,
  file_name: string
) => {
  const content = await getUniverseFile(project, universe_name, file_path);
  zip.file(file_name, content);
};

const zipFolder = async (
  zip: JSZip,
  file: Entry,
  project: string,
  universe_name: string
) => {
  const folder = zip.folder(file.name);

  if (folder === null) {
    return;
  }

  for (let index = 0; index < file.files.length; index++) {
    const element = file.files[index];
    console.log(element);
    if (element.is_dir) {
      await zipFolder(folder, element, project, universe_name);
    } else {
      await zipFile(folder, project, universe_name, element.path, element.name);
    }
  }
};

const zipToData = (zip: JSZip) => {
  return new Promise((resolve) => {
    const reader = new FileReader();
    reader.onloadend = () => resolve(reader.result);
    zip.generateAsync({ type: "blob" }).then(function (content) {
      reader.readAsDataURL(content);
    });
  });
};

export const createCustomUniverseConfig = async (
  project: string,
  configJson: any
) => {
  const file_list = await getFileList(project, configJson.name);

  const files: Entry[] = JSON.parse(file_list);

  const universe: Entry = {
    name: configJson.name,
    is_dir: true,
    path: "",
    files: files,
    group: "",
    access: false,
  };

  const zip = new JSZip();

  for (let index = 0; index < universe.files.length; index++) {
    const element = universe.files[index];
    console.log(element);
    if (element.is_dir) {
      await zipFolder(zip, element, project, universe.name);
    } else {
      await zipFile(zip, project, universe.name, element.path, element.name);
    }
  }

  const base64data = await zipToData(zip);

  const universe_type = configJson.ram_config.type
    ? configJson.ram_config.type
    : "gz";
  const tools = configJson.ram_config.tools
    ? configJson.ram_config.tools
    : ["console", "simulator", "state_monitor"];
  let tools_config = configJson.ram_config.tools_config
    ? configJson.ram_config.tools_config
    : {};

  if (configJson.ram_config.visualization_config_path) {
    tools_config = { gzsim: configJson.ram_config.visualization_config_path };
  }

  const world_config = {
    name: configJson.name,
    launch_file_path: configJson.ram_config.launch_file_path,
    ros_version: configJson.ram_config.ros_version,
    type: universe_type,
    zip: base64data,
  };

  const robot_config = {
    name: null,
    launch_file_path: null,
    ros_version: null,
    type: null,
    start_pose: null,
  };

  const universe_config = {
    world: world_config,
    robot: robot_config,
    tools: tools,
    tools_config: tools_config,
  };

  return universe_config;
};
