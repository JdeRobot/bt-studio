import "./StatusBar.css";

import { ReactComponent as ResetIcon } from "./img/reset.svg";
import CommsManager from "../../../api_helper/CommsManager";
import { useEffect, useRef, useState } from "react";
import {
  getFileList,
  getRoboticsBackendUniverse,
  getUniverseConfig,
  getUniverseFile,
} from "../../../api_helper/TreeWrapper";
import JSZip from "jszip";
import UniverseModal from "./universe_modal/UniverseModal";
import { subscribe, unsubscribe } from "../../helper/TreeEditorHelper";

interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  files: Entry[];
}

const StatusBar = ({
  project,
  commsManager,
  resetManager,
}: {
  project: string;
  commsManager: CommsManager | null;
  resetManager: Function;
}) => {
  const [dockerData, setDockerData] = useState<any>(
    commsManager?.getHostData(),
  );
  const [state, setState] = useState<string | undefined>(
    commsManager?.getState(),
  );

  const connectWithRetry = async () => {
    const data = commsManager?.getHostData();
    if (data) {
      setDockerData(data);
      return;
    }
    setTimeout(connectWithRetry, 1000);
  };

  if (dockerData === undefined) {
    connectWithRetry();
  }

  const updateState = (e: any) => {
    setState(e.detail.state);
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

  return (
    <div className="bt-status-bar-container">
      {dockerData ? (
        <>
          <div className="bt-status-bar-div" title="ROS 2 version">
            <label className="bt-status-bar-label">{`ROS 2: ${dockerData.ros_version}`}</label>
          </div>
          <div className="bt-status-bar-div" title="GPU status">
            <label className="bt-status-bar-label">{`GPU: ${dockerData.gpu_avaliable}`}</label>
          </div>
          <div className="bt-status-bar-div" title="Robotics Backend version">
            <label className="bt-status-bar-label">{`Robotics Backend: ${dockerData.robotics_backend_version}`}</label>
          </div>
        </>
      ) : (
        <button
          className={`bt-status-bar-button`}
          id={`reset-connection`}
          onClick={() => {
            resetManager();
          }}
          title="Reconnect with Robotics Backend"
        >
          <ResetIcon className="bt-status-bar-icon" stroke={"var(--icon)"} />
          <label className="bt-status-bar-label">Reconnect</label>
        </button>
      )}
      <div className="bt-status-bar-div" title="Robotics Backend state">
        <label className="bt-status-bar-label">{state}</label>
      </div>
      <ModalUniverseSelector project={project} commsManager={commsManager} />
    </div>
  );
};

export default StatusBar;

const DefaultUniverseSelector = ({
  project,
  commsManager,
}: {
  project: string;
  commsManager: CommsManager | null;
}) => {
  const [universe, setUniverse] = useState<string | undefined>(
    commsManager?.getUniverse(),
  );

  const [isUniversesModalOpen, setUniversesModalOpen] = useState(false);

  useEffect(() => {
    if (commsManager) {
      console.log("Change Universe", commsManager.getUniverse());
      setUniverse(commsManager.getUniverse());
    }
  }, [commsManager?.getUniverse()]);

  const terminateUniverse = async () => {
    if (!commsManager) {
      // warning(
      //   "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      // );
      return;
    }
    // Down the RB ladder
    await commsManager.terminateApplication();
    await commsManager.terminateVisualization();
    await commsManager.terminateUniverse();
  };

  const zipFile = async (
    zip: JSZip,
    universe_name: string,
    file_path: string,
    file_name: string,
  ) => {
    var content = await getUniverseFile(project, universe_name, file_path);
    zip.file(file_name, content);
  };

  const zipFolder = async (zip: JSZip, file: Entry, universe_name: string) => {
    const folder = zip.folder(file.name);

    if (folder === null) {
      return;
    }

    for (let index = 0; index < file.files.length; index++) {
      const element = file.files[index];
      console.log(element);
      if (element.is_dir) {
        await zipFolder(folder, element, universe_name);
      } else {
        await zipFile(folder, universe_name, element.path, element.name);
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

  const launchUniverse = async (universeConfig: string) => {
    if (!commsManager) {
      // warning(
      //   "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      // );
      return;
    }

    if (project === "") {
      // error("Failed to find the current project name.");
      return;
    }

    console.log("UC: " + universeConfig);
    const configJson = JSON.parse(universeConfig);

    try {
      if (configJson.type === "robotics_backend") {
        const dockerUniverseInfo = await getRoboticsBackendUniverse(
          configJson.id,
        );

        let visualization = "bt_studio";

        if (dockerUniverseInfo.visualization === "gzsim_rae") {
          visualization = "bt_studio_gz";
        }

        const world_config = dockerUniverseInfo.world;

        const robot_config = dockerUniverseInfo.robot;

        const universe_config = {
          name: configJson.name,
          world: world_config,
          robot: robot_config,
        };

        await commsManager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await commsManager.prepareVisualization(
          visualization,
          dockerUniverseInfo.visualization_config,
        );
        console.log("Viz ready!");
      } else {
        const file_list = await getFileList(project, configJson.name);

        const files: Entry[] = JSON.parse(file_list);

        const universe: Entry = {
          name: configJson.name,
          is_dir: true,
          path: "",
          files: files,
        };

        const zip = new JSZip();

        for (let index = 0; index < universe.files.length; index++) {
          const element = universe.files[index];
          console.log(element);
          if (element.is_dir) {
            await zipFolder(zip, element, universe.name);
          } else {
            await zipFile(zip, universe.name, element.path, element.name);
          }
        }

        const base64data = await zipToData(zip);

        const world_config = {
          name: configJson.name,
          launch_file_path: configJson.ram_config.launch_file_path,
          ros_version: configJson.ram_config.ros_version,
          world: configJson.ram_config.world,
          zip: base64data,
        };

        const robot_config = {
          name: null,
          launch_file_path: null,
          ros_version: null,
          visualization: null,
          world: null,
          start_pose: null,
        };

        const universe_config = {
          name: configJson.name,
          world: world_config,
          robot: robot_config,
        };

        const visualization_config = configJson.ram_config
          .visualization_config_path
          ? configJson.ram_config.visualization_config_path
          : null;

        await commsManager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await commsManager.prepareVisualization(
          "bt_studio_gz",
          visualization_config,
        );
        console.log("Viz ready!");
      }
    } catch (e: unknown) {
      throw e; // rethrow
    }
  };

  const onCloseUniverseModal = async (universeName: string) => {
    // First, close the modal for reactivity
    setUniversesModalOpen(false);

    if (!universeName) return;

    // Get the config from the backend
    try {
      const universeConfig = await getUniverseConfig(universeName, project);
      try {
        // Launch if new universe selected
        if (universeName !== universe) {
          if (universe) await terminateUniverse();
          await launchUniverse(universeConfig);
          console.log("Launch universe successful");
        }
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Unable to launch selected universe: " + e.message);
          // error("Unable to launch selected universe: " + e.message);
        }
      }
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Unable to retrieve universe config: " + e.message);
        // error("Unable to retrieve universe config: " + e.message);
      }
    }
  };

  const onOpenUniverseModal = (e: any) => {
    setUniversesModalOpen(true);
  };

  return (
    <>
      <div
        className="bt-status-bar-div"
        title="Select Universe"
        onClick={onOpenUniverseModal}
      >
        <label className="bt-status-bar-label">
          {universe ? `Universe: ${universe}` : "Click to select universe"}
        </label>
      </div>
      <UniverseModal
        isOpen={isUniversesModalOpen}
        onSubmit={(data: unknown) => {}}
        onClose={onCloseUniverseModal}
        currentProject={project}
      />
    </>
  );
};

const ModalUniverseSelector = ({
  project,
  commsManager,
}: {
  project: string;
  commsManager: CommsManager | null;
}) => {
  const [universe, setUniverse] = useState<string | undefined>(
    commsManager?.getUniverse(),
  );

  const [isUniversesModalOpen, setUniversesModalOpen] = useState(false);

  useEffect(() => {
    if (commsManager) {
      console.log("Change Universe", commsManager.getUniverse());
      setUniverse(commsManager.getUniverse());
    }
  }, [commsManager?.getUniverse()]);

  const terminateUniverse = async () => {
    if (!commsManager) {
      // warning(
      //   "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      // );
      return;
    }
    // Down the RB ladder
    await commsManager.terminateApplication();
    await commsManager.terminateVisualization();
    await commsManager.terminateUniverse();
  };

  const zipFile = async (
    zip: JSZip,
    universe_name: string,
    file_path: string,
    file_name: string,
  ) => {
    var content = await getUniverseFile(project, universe_name, file_path);
    zip.file(file_name, content);
  };

  const zipFolder = async (zip: JSZip, file: Entry, universe_name: string) => {
    const folder = zip.folder(file.name);

    if (folder === null) {
      return;
    }

    for (let index = 0; index < file.files.length; index++) {
      const element = file.files[index];
      console.log(element);
      if (element.is_dir) {
        await zipFolder(folder, element, universe_name);
      } else {
        await zipFile(folder, universe_name, element.path, element.name);
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

  const launchUniverse = async (universeConfig: string) => {
    if (!commsManager) {
      // warning(
      //   "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      // );
      return;
    }

    if (project === "") {
      // error("Failed to find the current project name.");
      return;
    }

    console.log("UC: " + universeConfig);
    const configJson = JSON.parse(universeConfig);

    try {
      if (configJson.type === "robotics_backend") {
        const dockerUniverseInfo = await getRoboticsBackendUniverse(
          configJson.id,
        );

        let visualization = "bt_studio";

        if (dockerUniverseInfo.visualization === "gzsim_rae") {
          visualization = "bt_studio_gz";
        }

        const world_config = dockerUniverseInfo.world;

        const robot_config = dockerUniverseInfo.robot;

        const universe_config = {
          name: configJson.name,
          world: world_config,
          robot: robot_config,
        };

        await commsManager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await commsManager.prepareVisualization(
          visualization,
          dockerUniverseInfo.visualization_config,
        );
        console.log("Viz ready!");
      } else {
        const file_list = await getFileList(project, configJson.name);

        const files: Entry[] = JSON.parse(file_list);

        const universe: Entry = {
          name: configJson.name,
          is_dir: true,
          path: "",
          files: files,
        };

        const zip = new JSZip();

        for (let index = 0; index < universe.files.length; index++) {
          const element = universe.files[index];
          console.log(element);
          if (element.is_dir) {
            await zipFolder(zip, element, universe.name);
          } else {
            await zipFile(zip, universe.name, element.path, element.name);
          }
        }

        const base64data = await zipToData(zip);

        const world_config = {
          name: configJson.name,
          launch_file_path: configJson.ram_config.launch_file_path,
          ros_version: configJson.ram_config.ros_version,
          world: configJson.ram_config.world,
          zip: base64data,
        };

        const robot_config = {
          name: null,
          launch_file_path: null,
          ros_version: null,
          visualization: null,
          world: null,
          start_pose: null,
        };

        const universe_config = {
          name: configJson.name,
          world: world_config,
          robot: robot_config,
        };

        const visualization_config = configJson.ram_config
          .visualization_config_path
          ? configJson.ram_config.visualization_config_path
          : null;

        await commsManager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await commsManager.prepareVisualization(
          "bt_studio_gz",
          visualization_config,
        );
        console.log("Viz ready!");
      }
    } catch (e: unknown) {
      throw e; // rethrow
    }
  };

  const onCloseUniverseModal = async (universeName: string) => {
    // First, close the modal for reactivity
    setUniversesModalOpen(false);

    if (!universeName) return;

    // Get the config from the backend
    try {
      const universeConfig = await getUniverseConfig(universeName, project);
      try {
        // Launch if new universe selected
        if (universeName !== universe) {
          if (universe) await terminateUniverse();
          await launchUniverse(universeConfig);
          console.log("Launch universe successful");
        }
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Unable to launch selected universe: " + e.message);
          // error("Unable to launch selected universe: " + e.message);
        }
      }
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Unable to retrieve universe config: " + e.message);
        // error("Unable to retrieve universe config: " + e.message);
      }
    }
  };

  const onOpenUniverseModal = (e: any) => {
    setUniversesModalOpen(true);
  };

  return (
    <>
      <div
        className="bt-status-bar-div"
        title="Select Universe"
        onClick={onOpenUniverseModal}
      >
        <label className="bt-status-bar-label">
          {universe ? `Universe: ${universe}` : "Click to select universe"}
        </label>
      </div>
      <UniverseModal
        isOpen={isUniversesModalOpen}
        onSubmit={(data: unknown) => {}}
        onClose={onCloseUniverseModal}
        currentProject={project}
      />
    </>
  );
};
