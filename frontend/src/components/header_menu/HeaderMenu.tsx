import { useContext, useState } from "react";
import JSZip from "jszip";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import {
  createProject,
  generateLocalApp,
  generateDockerizedApp,
  getUniverseConfig,
  getRoboticsBackendUniversePath,
  getUniverseFile,
  getUniverseFileList,
} from "../../api_helper/TreeWrapper";
import CommsManager from "../../api_helper/CommsManager";

import { ReactComponent as LogoIcon } from "../file_editor/img/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "../file_editor/img/logo_unibotics_monocolor.svg";

import "./HeaderMenu.css";
import { ReactComponent as ProjectsIcon } from "./img/change_project.svg";
import { ReactComponent as SaveIcon } from "./img/save_project.svg";
import { ReactComponent as UniversesIcon } from "./img/universes.svg";
import { ReactComponent as SettingsIcon } from "./img/settings.svg";
import { ReactComponent as DownloadIcon } from "./img/download.svg";
import { ReactComponent as RunIcon } from "./img/run.svg";
import { ReactComponent as StopIcon } from "./img/stop.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";
import ProjectModal from "./modals/ProjectModal";
import UniversesModal from "./modals/UniverseModal";
import SettingsModal from "../settings_popup/SettingsModal";
import { OptionsContext } from "../options/Options";

import RosTemplates from "./../../templates/RosTemplates";
import TreeGardener from "./../../templates/TreeGardener";
import { useError } from "../error_popup/ErrorModal";
import { Entry } from "../file_browser/FileBrowser";

const HeaderMenu = ({
  currentProjectname,
  setCurrentProjectname,
  currentUniverseName,
  setCurrentUniverseName,
  setSaveCurrentDiagram,
  projectChanges,
  setProjectChanges,
  gazeboEnabled,
  setGazeboEnabled,
  manager,
  showVNCViewer,
  isUnibotics,
}: {
  currentProjectname: string;
  setCurrentProjectname: Function;
  currentUniverseName: string;
  setCurrentUniverseName: Function;
  setSaveCurrentDiagram: Function;
  projectChanges: boolean;
  setProjectChanges: Function;
  gazeboEnabled: boolean;
  setGazeboEnabled: Function;
  manager: CommsManager | null;
  showVNCViewer: Function;
  isUnibotics: boolean;
}) => {
  const { warning, error } = useError();

  // Settings
  const settings = useContext(OptionsContext);

  // Project state
  const [existingProjects, setExistingProjects] = useState<string[]>([]);

  // App state
  const [appRunning, setAppRunning] = useState(false);

  // Modal state
  const [isProjectModalOpen, setProjectModalOpen] = useState(true);
  const [isUniversesModalOpen, setUniversesModalOpen] = useState(false);
  const [isSettingsModalOpen, setSettingsModalOpen] = useState(false);

  // RB helpers

  const terminateUniverse = async () => {
    if (!manager) {
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }
    // Down the RB ladder
    await manager.terminateApplication();
    await manager.terminateVisualization();
    await manager.terminateUniverse();
  };

  const zipFile = async (
    zip: JSZip,
    universe_name: string,
    file_path: string,
    file_name: string,
  ) => {
    var content = await getUniverseFile(
      currentProjectname,
      universe_name,
      file_path,
    );
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
    if (!manager) {
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }

    if (currentProjectname === "") {
      error("Failed to find the current project name.");
      return;
    }

    console.log("UC: " + universeConfig);
    const configJson = JSON.parse(universeConfig);

    try {
      if (configJson.type === "robotics_backend") {
        const dockerUniverseInfo = await getRoboticsBackendUniversePath(
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

        await manager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await manager.prepareVisualization(
          visualization,
          dockerUniverseInfo.visualization_config,
        );
        console.log("Viz ready!");
      } else {
        const file_list = await getUniverseFileList(
          currentProjectname,
          configJson.name,
        );

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
          world: world_config,
          robot: robot_config,
        };

        const visualization_config = configJson.ram_config
          .visualization_config_path
          ? configJson.ram_config.visualization_config_path
          : null;

        await manager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await manager.prepareVisualization(
          "bt_studio_gz",
          visualization_config,
        );
        console.log("Viz ready!");
      }
    } catch (e: unknown) {
      throw e; // rethrow
    }
  };

  // Project handling

  const onCreateProject = async (projectName: string) => {
    try {
      await createProject(projectName);
      setCurrentProjectname(projectName);
      console.log("Project created successfully");
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Error creating project: " + e.message);
        error("Error creating project: " + e.message);
      }
    }
  };

  const onChangeProject = async (projectName: string) => {
    if (existingProjects.includes(projectName)) {
      // Project exists, proceed to change
      setCurrentProjectname(projectName);

      // Proper simulation loading
      setGazeboEnabled(false);

      // Terminate the universe
      if (currentUniverseName) {
        await terminateUniverse();
      }
      setCurrentUniverseName("");
      console.log(`Switched to project ${projectName}`);
      console.log("Universe terminated!");
    } else {
      console.error(`The project ${projectName} does not exist`);
      error(`The project ${projectName} does not exist`);
    }
  };

  const onSaveProject = async () => {
    if (!projectChanges) {
      return;
    }
    try {
      //TODO: check if possible concurrency problems
      setSaveCurrentDiagram(true);
      setProjectChanges(false);
      console.log("Project saved");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error saving project: " + e.message);
        error(e.message);
      }
    }
  };

  // App handling

  const onDownloadApp = async () => {
    try {
      await onSaveProject();

      // Get the blob from the API wrapper
      const appFiles = await generateLocalApp(
        currentProjectname,
        settings.btOrder.value,
      );

      // Create the zip with the files
      const zip = new JSZip();

      console.log(appFiles.dependencies);

      TreeGardener.addLocalFiles(zip);
      RosTemplates.addLocalFiles(
        zip,
        currentProjectname,
        appFiles.tree,
        appFiles.dependencies,
      );

      zip.generateAsync({ type: "blob" }).then(function (content) {
        // Create a download link and trigger download
        const url = window.URL.createObjectURL(content);
        const a = document.createElement("a");
        a.style.display = "none";
        a.href = url;
        a.download = `${currentProjectname}.zip`; // Set the downloaded file's name
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url); // Clean up after the download
      });

      console.log("App downloaded successfully");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error downloading app: " + e.message);
        error("Error downloading app: " + e.message);
      }
    }
  };

  const onAppStateChange = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }

    if (!gazeboEnabled) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

    await onSaveProject();

    if (!appRunning) {
      try {
        // Get the blob from the API wrapper
        const appFiles = await generateDockerizedApp(
          currentProjectname,
          settings.btOrder.value,
        );

        // Create the zip with the files
        const zip = new JSZip();

        zip.file("self_contained_tree.xml", appFiles.tree);
        TreeGardener.addDockerFiles(zip);
        RosTemplates.addDockerFiles(zip);

        // Convert the blob to base64 using FileReader
        const reader = new FileReader();
        reader.onloadend = async () => {
          const base64data = reader.result; // Get the zip in base64
          // Send the base64 encoded blob
          await manager.run({
            type: "bt-studio",
            code: base64data,
          });
          console.log("Dockerized app started successfully");
        };

        zip.generateAsync({ type: "blob" }).then(function (content) {
          reader.readAsDataURL(content);
        });

        setAppRunning(true);
        console.log("App started successfully");
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error running app: " + e.message);
          error("Error running app: " + e.message);
        }
      }
    } else {
      try {
        await manager.pause();
        setAppRunning(false);
        console.log("App paused correctly!");
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Error pausing app: " + e.message);
          error("Error pausing app: " + e.message);
        }
      }
    }
  };

  const onResetApp = async () => {
    if (!manager) {
      console.error("Manager is not running");
      warning(
        "Failed to connect with the Robotics Backend docker. Please make sure it is connected.",
      );
      return;
    }

    if (!gazeboEnabled) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

    await manager.terminateApplication();
    console.log("App reseted!");
    setAppRunning(false);
  };

  // Modal handling

  const onOpenProjectModal = (e: any) => {
    setProjectModalOpen(true);
    onSaveProject();
  };

  const onCloseProjectModal = async (projectName: string) => {
    if (projectName) {
      await onChangeProject(projectName);
    }
    setProjectModalOpen(false);
  };

  const onOpenUniverseModal = (e: any) => {
    setUniversesModalOpen(true);
    onSaveProject();
  };

  const onCloseUniverseModal = async (universeName: string) => {
    // First, close the modal for reactivity
    setUniversesModalOpen(false);

    if (!universeName) return;

    // Get the config from the backend
    try {
      const universeConfig = await getUniverseConfig(
        universeName,
        currentProjectname,
      );
      try {
        // Launch if new universe selected
        if (universeName !== currentUniverseName) {
          if (currentUniverseName) await terminateUniverse();
          await launchUniverse(universeConfig);
          console.log("Launch universe successful");
          setGazeboEnabled(true);
          setCurrentUniverseName(universeName);
          showVNCViewer();
        }
      } catch (e: unknown) {
        if (e instanceof Error) {
          console.error("Unable to launch selected universe: " + e.message);
          error("Unable to launch selected universe: " + e.message);
        }
      }
    } catch (e: unknown) {
      if (e instanceof Error) {
        console.error("Unable to retrieve universe config: " + e.message);
        error("Unable to retrieve universe config: " + e.message);
      }
    }
  };

  const onOpenSettingsModal = (e: any) => {
    setSettingsModalOpen(true);
    onSaveProject();
  };

  const onCloseSettingsModal = () => {
    setSettingsModalOpen(false);
  };

  const onSubmit = (data: unknown) => {};

  return (
    <AppBar position="static">
      <Toolbar style={{ backgroundColor: "var(--header)" }}>
        {isUnibotics ? (
          <a href="/apps">
            <LogoUniboticsIcon className="bt-jde-icon" fill="var(--icon)" />
          </a>
        ) : (
          <LogoIcon className="bt-jde-icon" fill="var(--icon)" />
        )}
        <h1 className="bt-Header-text">BT Studio IDE</h1>
        <ProjectModal
          isOpen={isProjectModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseProjectModal}
          currentProject={currentProjectname}
          existingProjects={existingProjects}
          setExistingProjects={setExistingProjects}
          createProject={onCreateProject}
        />
        <UniversesModal
          isOpen={isUniversesModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseUniverseModal}
          currentProject={currentProjectname}
        />

        <SettingsModal
          isOpen={isSettingsModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseSettingsModal}
          currentProjectname={currentProjectname}
        />

        <div className="bt-header-button-container">
          {currentProjectname && (
            <span className="bt-project-name-box">
              <div className="bt-project-name">
                {currentProjectname +
                  " ~ " +
                  (currentUniverseName
                    ? currentUniverseName
                    : "No Universe selected")}
              </div>
              {projectChanges && <div className="bt-small-text">Unsaved</div>}
            </span>
          )}
          <button
            className="bt-header-button"
            id="open-project-manager"
            onClick={onOpenProjectModal}
            title="Change project"
          >
            <ProjectsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            id="open-universe-manager"
            onClick={onOpenUniverseModal}
            title="Universe menu"
          >
            <UniversesIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            id="open-settings-manager"
            onClick={onOpenSettingsModal}
            title="Settings"
          >
            <SettingsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            id="save-bt-changes"
            onClick={onSaveProject}
            title="Save project"
          >
            <SaveIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            id="download-app"
            onClick={onDownloadApp}
            title="Download app"
          >
            <DownloadIcon className="bt-header-icon" stroke={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            id="run-app"
            onClick={onAppStateChange}
            title="Run app"
          >
            {appRunning ? (
              <StopIcon className="bt-header-icon" fill={"var(--icon)"} />
            ) : (
              <RunIcon className="bt-header-icon" fill={"var(--icon)"} />
            )}
          </button>
          <button
            className="bt-header-button"
            id="reset-app"
            onClick={onResetApp}
            title="Reset app"
          >
            <ResetIcon className="bt-header-icon" stroke={"var(--icon)"} />
          </button>
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
