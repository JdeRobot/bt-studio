import { MouseEventHandler, useContext, useEffect, useState } from "react";
import JSZip from "jszip";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import {
  createProject,
  saveBaseTree,
  generateApp,
  generateDockerizedApp,
  getUniverseConfig,
  getCustomUniverseZip,
  getRoboticsBackendUniversePath,
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

const HeaderMenu = ({
  currentProjectname,
  setCurrentProjectname,
  currentUniverseName,
  setCurrentUniverseName,
  modelJson,
  projectChanges,
  setProjectChanges,
  gazeboEnabled,
  setGazeboEnabled,
  // onSetShowExecStatus,
  manager,
  showVNCViewer,
  isUnibotics,
}: {
  currentProjectname: string;
  setCurrentProjectname: Function;
  currentUniverseName: string;
  setCurrentUniverseName: Function;
  modelJson: string;
  projectChanges: boolean;
  setProjectChanges: Function;
  gazeboEnabled: boolean;
  setGazeboEnabled: Function;
  manager: CommsManager | null;
  showVNCViewer: Function;
  isUnibotics: boolean;
}) => {
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
      return;
    }
    // Down the RB ladder
    await manager.terminateApplication();
    await manager.terminateVisualization();
    await manager.terminateUniverse();
  };

  const launchUniverse = async (universeConfig: string) => {
    if (!manager) {
      return;
    }

    console.log("UC: " + universeConfig);
    const configJson = JSON.parse(universeConfig);

    try {
      if (configJson.type === "robotics_backend") {
        const launch_file_path = await getRoboticsBackendUniversePath(
          configJson.id,
        );

        const universe_config = {
          name: configJson.name,
          launch_file_path: launch_file_path,
          ros_version: "ROS2",
          visualization: "bt_studio",
          world: "gazebo",
        };

        await manager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await manager.prepareVisualization(universe_config.visualization);
        console.log("Viz ready!");
      } else {
        console.log("Custom universe rework underway");
        console.warn("Custom universe rework underway");
        const zipBlob: Blob = await getCustomUniverseZip(
          configJson.name,
          currentProjectname,
        );
        var reader = new FileReader();
        reader.readAsDataURL(zipBlob);
        reader.onloadend = async function () {
          // Get the zip in base64
          var base64data = reader.result;
          const universe_config = {
            name: configJson.name,
            launch_file_path: configJson.ram_config.launch_file_path,
            ros_version: configJson.ram_config.ros_version,
            visualization: "bt_studio",
            world: configJson.ram_config.world,
            zip: base64data,
          };
          await manager.launchWorld(universe_config);
          console.log("RB universe launched!");
          await manager.prepareVisualization(universe_config.visualization);
          console.log("Viz ready!");
        };
      }
    } catch (error: unknown) {
      throw error; // rethrow
    }
  };

  // Project handling

  const onCreateProject = async (projectName: string) => {
    try {
      await createProject(projectName);
      setCurrentProjectname(projectName);
      console.log("Project created successfully");
    } catch (error: unknown) {
      if (error instanceof Error) {
        console.error("Error creating project: " + error.message);
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
    }
  };

  const onSaveProject = async () => {
    if (!projectChanges) {
      return;
    }
    try {
      await saveBaseTree(modelJson, currentProjectname);
      setProjectChanges(false);
      console.log("Project saved");
    } catch (error) {
      if (error instanceof Error) {
        console.error("Error saving project: " + error.message);
      }
    }
  };

  // App handling

  const onDownloadApp = async () => {
    try {
      // Get the blob from the API wrapper
      const appBlob = await generateApp(
        modelJson,
        currentProjectname,
        settings.btOrder.value,
      );

      // Create a download link and trigger download
      const url = window.URL.createObjectURL(appBlob);
      const a = document.createElement("a");
      a.style.display = "none";
      a.href = url;
      a.download = `${currentProjectname}.zip`; // Set the downloaded file's name
      document.body.appendChild(a);
      a.click();
      window.URL.revokeObjectURL(url); // Clean up after the download
      console.log("App downloaded successfully");
    } catch (error) {
      if (error instanceof Error) {
        console.error("Error downloading app: " + error.message);
      }
    }
  };

  const onAppStateChange = async () => {
    if (!manager) {
      console.error("Manager is not running");
      return;
    }

    if (!gazeboEnabled) {
      console.error("Simulation is not ready!");
      return;
    }

    if (!appRunning) {
      try {
        // Get the blob from the API wrapper
        const appFiles = await generateDockerizedApp(
          modelJson,
          currentProjectname,
          settings.btOrder.value,
        );

        // Create the zip with the files

        const zip = new JSZip();

        zip.file("self_contained_tree.xml", appFiles.tree);
        zip.file("tree_factory.py", appFiles.factory);
        zip.file("tree_tools.py", appFiles.tools);
        zip.file("execute_docker.py", appFiles.entrypoint);

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

        zip.generateAsync({type:"blob"}).then(function(content) {
          reader.readAsDataURL(content);
        });

        setAppRunning(true);
        console.log("App started successfully");
      } catch (error: unknown) {
        if (error instanceof Error) {
          console.error("Error running app: " + error.message);
        }
      }
    } else {
      try {
        await manager.pause();
        setAppRunning(false);
        console.log("App paused correctly!");
      } catch (error: unknown) {
        if (error instanceof Error) {
          console.error("Error pausing app: " + error.message);
        }
      }
    }
  };

  const onResetApp = async () => {
    if (!manager) {
      console.error("Manager is not running");
      return;
    }

    if (!gazeboEnabled) {
      console.error("Simulation is not ready!");
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
      } catch (error: unknown) {
        if (error instanceof Error) {
          console.error("Unable launch selected universe: " + error.message);
        }
      }
    } catch (error: unknown) {
      if (error instanceof Error) {
        console.error("Unable to retrieve universe config: " + error.message);
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

  const openError = (err: unknown) => {
    console.log("Modal error!");
  };

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
          openError={openError}
        />
        <UniversesModal
          isOpen={isUniversesModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseUniverseModal}
          currentProject={currentProjectname}
          openError={openError}
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
            onClick={onOpenProjectModal}
            title="Change project"
          >
            <ProjectsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            onClick={onOpenUniverseModal}
            title="Universe menu"
          >
            <UniversesIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            onClick={onOpenSettingsModal}
            title="Settings"
          >
            <SettingsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            onClick={onSaveProject}
            title="Save project"
          >
            <SaveIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
            onClick={onDownloadApp}
            title="Download app"
          >
            <DownloadIcon className="bt-header-icon" stroke={"var(--icon)"} />
          </button>
          <button
            className="bt-header-button"
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
