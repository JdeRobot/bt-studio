import { Entry, ExtraApi } from "jderobot-ide-interface";
import {
  getFile,
  saveFile,
  listUniverses,
  getUniverseConfig,
  getRoboticsBackendUniverse,
} from "../../api_helper/TreeWrapper";
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

      if (configJson.type === "robotics_backend") {
        return getRoboticsBackendUniverse(configJson.id);
      }
      // Get custom universe config
      return createCustomUniverseConfig(project, configJson);
    },
  },
};