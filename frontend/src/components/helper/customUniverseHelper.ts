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
  var content = await getUniverseFile(project, universe_name, file_path);
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

export const createCustomUniverseConfig = async (project: string, configJson: any) => {
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

  const world_config = {
    name: configJson.name,
    launch_file_path: configJson.ram_config.launch_file_path,
    ros_version: configJson.ram_config.ros_version,
    world: configJson.ram_config.world,
    tools_config: {},
    zip: base64data,
  };

  const robot_config = {
    name: null,
    launch_file_path: null,
    ros_version: null,
    world: null,
    start_pose: null,
  };

  const universe_config = {
    world: world_config,
    robot: robot_config,
    "tools": ["console", "simulator", "state_monitor"],
    "tools_config": {},
  };

  // config = {
  //     "name": universe.name,
  //     "world": {
  //         "name": universe.world.name,
  //         "launch_file_path": universe.world.launch_file_path,
  //         "ros_version": universe.world.ros_version,
  //         "type": universe.world.type,
  //         "tools_config": tools_configuration,
  //     },
  //     "tools": tools,
  //     "tools_config": tools_config,
  //     "robot": robot_config,
  //     "template": self.template,
  //     "exercise_id": self.exercise_id,
  // }

  return universe_config
};
