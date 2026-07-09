import { Entry, ExtraApi } from "jderobot-ide-interface";
import {
  getFile,
  saveFile,
  listWorlds,
  getWorldConfig,
} from "BtApi/TreeWrapper";
import { zipCustomWorld } from "../helper/customUniverseHelper";

export const editorApi: ExtraApi = {
  file: {
    get: (project: string, file: Entry) => {
      if (file.group === "Worlds") {
        return getFile(project, file.path, "", file.binary);
      }
      return getFile(project, file.path, undefined, file.binary);
    },
    save: (project: string, file: Entry, content: string) => {
      if (file.group === "Worlds") {
        return saveFile(project, file.path, content, "");
      }
      return saveFile(project, file.path, content);
    },
  },
  worlds: {
    list: (project: string) => {
      return listWorlds(project);
    },
    get_config: async (project: string, world: string) => {
      const worldConfig = await getWorldConfig(project, world);

      if (worldConfig.isCustom) {
        worldConfig.config.scene["zip"] = await zipCustomWorld(
          project,
          worldConfig.name,
        );
      }

      if (!worldConfig.config.tools.includes("state_monitor")) {
        worldConfig.config.tools.push("state_monitor");
      }

      return worldConfig.config;
    },
  },
};
