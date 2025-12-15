import { Entry, ExtraApi } from "jderobot-ide-interface";
import {
  getFile,
  saveFile,
  listUniverses,
  getUniverseConfig,
  getRoboticsBackendUniverse,
} from "BtApi/TreeWrapper";
import { createCustomUniverseConfig } from "../helper/customUniverseHelper";

export const editorApi: ExtraApi = {
  file: {
    get: (project: string, file: Entry) => {
      if (file.group === "Universes") {
        return getFile(project, file.path, "");
      }
      return getFile(project, file.path);
    },
    save: (project: string, file: Entry, content: string) => {
      if (file.group === "Universes") {
        return saveFile(project, file.path, content, "");
      }
      return saveFile(project, file.path, content);
    },
  },
  universes: {
    list: (project: string) => {
      return listUniverses(project);
    },
    get_config: async (project: string, universe: string) => {
      const config = await getUniverseConfig(universe, project);
      const configJson = JSON.parse(config);
      let universeConfig;

      if (configJson.type === "robotics_backend") {
        universeConfig = await getRoboticsBackendUniverse(configJson.id);
      } else {
        // Get custom universe config
        universeConfig = await createCustomUniverseConfig(project, configJson);
      }

      if (!universeConfig.tools.includes("state_monitor")) {
        universeConfig.tools.push("state_monitor");
      }
      
      return universeConfig;
    },
  },
};
