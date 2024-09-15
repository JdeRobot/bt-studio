import { MouseEventHandler, useEffect, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import {
  createProject,
  saveProject,
  generateApp,
  getUniverseConfig,
  getCustomUniverseZip,
} from "../../api_helper/TreeWrapper";
import CommsManager from "../../api_helper/CommsManager";

import { ReactComponent as LogoIcon } from "../file_editor/img/logo_jderobot_monocolor.svg";

import "./HeaderMenu.css";
import { ReactComponent as ProjectsIcon } from "./img/change_project.svg";
import { ReactComponent as SaveIcon } from "./img/save_project.svg";
import { ReactComponent as UniversesIcon } from "./img/universes.svg";
import { ReactComponent as SettingsIcon } from "./img/settings.svg";
import { ReactComponent as DownloadIcon } from "./img/download.svg";
import { ReactComponent as RunIcon } from "./img/run.svg";
import { ReactComponent as StopIcon } from "./img/stop.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";
import { ReactComponent as EyeOpenIcon } from "./img/eye_open.svg";
import { ReactComponent as EyeClosedIcon } from "./img/eye_closed.svg";
import ProjectModal from "./modals/ProjectModal";
import UniversesModal from "./modals/UniverseModal";
import SettingsModal from "../settings_popup/SettingsModal";

const HeaderMenu = ({
  currentProjectname,
  setCurrentProjectname,
  currentUniverseName,
  setCurrentUniverseName,
  modelJson,
  projectChanges,
  setProjectChanges,
  settingsProps,
  gazeboEnabled,
  setGazeboEnabled,
  // onSetShowExecStatus,
  manager,
}: {
  currentProjectname: string;
  setCurrentProjectname: Function;
  currentUniverseName: string;
  setCurrentUniverseName: Function;
  modelJson: string;
  projectChanges: boolean;
  setProjectChanges: Function;
  settingsProps: Object;
  gazeboEnabled: boolean;
  setGazeboEnabled: Function;
  manager: CommsManager;
}) => {
  // Project state
  const [existingProjects, setExistingProjects] = useState("");

  // App state
  const [appRunning, setAppRunning] = useState(false);

  // Modal state
  const [isProjectModalOpen, setProjectModalOpen] = useState(true);
  const [isUniversesModalOpen, setUniversesModalOpen] = useState(false);
  const [isSettingsModalOpen, setSettingsModalOpen] = useState(false);

  // RB helpers

  const terminateUniverse = async () => {
    // Down the RB ladder
    await manager.terminateApplication();
    await manager.terminateVisualization();
    await manager.terminateUniverse();
  };

  const launchUniverse = async (universeConfig: string) => {
    console.log("UC: " + universeConfig);
    const configJson = JSON.parse(universeConfig);

    try {
      if (configJson.type === "robotics_backend") {
        const universe_config = {
          name: configJson.name,
          launch_file_path: configJson.config.launch_file_path,
          ros_version: "ROS2",
          visualization: "bt_studio",
          world: "gazebo",
          exercise_id: configJson.config.id,
        };

        await manager.launchWorld(universe_config);
        console.log("RB universe launched!");
        await manager.prepareVisualization(universe_config.visualization);
        console.log("Viz ready!");
      } else {
        console.warn("Custom universe rework underway");
      }
    } catch (error: unknown) {
      throw error; // rethrow
    }
  };

  // const launchUniverse = async (universe_name) => {
  //   const apiUrl = `/tree_api/get_universe_configuration?project_name=${encodeURIComponent(
  //     currentProjectname,
  //   )}&universe_name=${encodeURIComponent(universe_name)}`;

  //   try {
  //     const response = await axios.get(apiUrl);
  //     console.log("Stored universe config: " + response.data);
  //     const stored_cfg = JSON.parse(response.data);
  //     if (stored_cfg.type === "robotics_backend") {
  //       await launchBackendUniverse(stored_cfg);
  //     } else {
  //       launchCustomUniverse(stored_cfg);
  //     }
  //   } catch (error) {
  //     console.error("Error launching universe:", error);
  //   }
  // };

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
    try {
      await saveProject(modelJson, currentProjectname);
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
      // Get the base64 string from the API wrapper
      const response = await generateApp(
        modelJson,
        currentProjectname,
        "bottom-to-top"
      );

      const app_base64 = response?.file; // Assuming the base64 string is returned in "file" key

      if (!app_base64) {
        throw new Error("Downloaded base64 string is empty");
      }

      // Log to check if base64 string is populated
      console.log(app_base64);

      // Convert base64 string to binary string
      const binaryString = window.atob(app_base64); // Decodes base64 string to binary string

      // Convert binary string to a Uint8Array (required for Blob creation)
      const len = binaryString.length;
      const bytes = new Uint8Array(len);

      for (let i = 0; i < len; i++) {
        bytes[i] = binaryString.charCodeAt(i);
      }

      // Create a blob from the Uint8Array
      const blob = new Blob([bytes], { type: "application/zip" });

      // Create a download link and trigger download
      const url = window.URL.createObjectURL(blob);
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
    if (!gazeboEnabled) {
      console.error("Simulation is not ready!");
    }

    if (!appRunning) {
      try {
        // Get the blob from the API wrapper
        const app_blob = await generateApp(
          modelJson,
          currentProjectname,
          "bottom-to-top"
        );
        const base64data = await app_blob.text();

        // Send the zip
        await manager.run({
          type: "bt-studio",
          code: base64data,
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
        currentProjectname
      );
      try {
        // Launch if new universe selected
        if (universeName !== currentUniverseName) {
          if (currentUniverseName) await terminateUniverse();
          await launchUniverse(universeConfig);
          console.log("Launch universe successful");
          setGazeboEnabled(true);
          setCurrentUniverseName(universeName);
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
      <Toolbar>
        <LogoIcon className="jde-icon" fill="var(--icon)" />
        <h1 className="Header-text">BT Studio IDE</h1>
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
          settings={settingsProps}
        />

        <div className="header-button-container">
          {currentProjectname && (
            <span className="project-name-box">
              <div className="project-name">
                {currentProjectname +
                  " ~ " +
                  (currentUniverseName
                    ? currentUniverseName
                    : "No Universe selected")}
              </div>
              {projectChanges && <div className="small-text">Unsaved</div>}
            </span>
          )}
          <button
            className="header-button"
            onClick={onOpenProjectModal}
            title="Change project"
          >
            <ProjectsIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={onOpenUniverseModal}
            title="Universe menu"
          >
            <UniversesIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={onOpenSettingsModal}
            title="Settings"
          >
            <SettingsIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={onSaveProject}
            title="Save project"
          >
            <SaveIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={onDownloadApp}
            title="Download app"
          >
            <DownloadIcon className="header-icon" stroke={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={onAppStateChange}
            title="Run app"
          >
            {appRunning ? (
              <StopIcon className="header-icon" fill={"var(--icon)"} />
            ) : (
              <RunIcon className="header-icon" fill={"var(--icon)"} />
            )}
          </button>
          <button
            className="header-button"
            onClick={onResetApp}
            title="Reset app"
          >
            <ResetIcon className="header-icon" stroke={"var(--icon)"} />
          </button>
          {/* <button
            className="header-button"
            onClick={() => onSetShowExecStatus()}
            title="Toggle status monitoring"
          >
            <EyeOpenIcon className="header-icon" stroke={"var(--icon)"} />
            {appRunning ? (
            ) : (
              <EyeClosedIcon className="header-icon" stroke={"var(--icon)"} />
            )}
          </button> */}
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
