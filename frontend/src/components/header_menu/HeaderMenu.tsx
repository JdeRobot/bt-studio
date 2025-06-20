import { useContext, useRef, useState } from "react";
import JSZip from "jszip";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import {
  createProject,
  generateLocalApp,
  generateDockerizedApp,
  getRoboticsBackendUniverse,
  getUniverseFile,
  getFileList,
} from "../../api_helper/TreeWrapper";
import { CommsManager } from "jderobot-commsmanager";

import { ReactComponent as LogoIcon } from "../icons/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "../icons/logo_unibotics_monocolor.svg";

import "./HeaderMenu.css";
import { ReactComponent as ProjectsIcon } from "./img/change_project.svg";
import { ReactComponent as SettingsIcon } from "./img/settings.svg";
import { ReactComponent as DownloadIcon } from "./img/download.svg";
import { ReactComponent as RunIcon } from "./img/run.svg";
import { ReactComponent as StopIcon } from "./img/stop.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";
import ProjectModal from "./modals/ProjectModal";
import SettingsModal from "../settings_popup/SettingsModal";
import { OptionsContext } from "../options/Options";

import RosTemplates from "./../../templates/RosTemplates";
import TreeGardener from "./../../templates/TreeGardener";
import { useError } from "../error_popup/ErrorModal";

interface Entry {
  name: string;
  is_dir: boolean;
  path: string;
  files: Entry[];
}

const HeaderMenu = ({
  currentProjectname,
  setCurrentProjectname,
  manager,
  isUnibotics,
  setLayout,
}: {
  currentProjectname: string;
  setCurrentProjectname: Function;
  manager: CommsManager | null;
  isUnibotics: boolean;
  setLayout: Function;
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

      // Terminate the universe
      if (manager?.getUniverse()) {
        await terminateUniverse();
      }
      console.log(`Switched to project ${projectName}`);
      console.log("Universe terminated!");
    } else {
      console.error(`The project ${projectName} does not exist`);
      error(`The project ${projectName} does not exist`);
    }
  };

  // App handling

  const onDownloadApp = async () => {
    try {
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

    if (
      manager.getState() !== "visualization_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
      console.error("Simulation is not ready!");
      warning(
        "Failed to found a running simulation. Please make sure an universe is selected.",
      );
      return;
    }

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

    if (
      manager.getState() !== "visualization_ready" &&
      manager.getState() !== "application_running" &&
      manager.getState() !== "paused"
    ) {
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
  };

  const onCloseProjectModal = async (projectName: string) => {
    if (projectName) {
      await onChangeProject(projectName);
    }
    setProjectModalOpen(false);
  };

  const onOpenSettingsModal = (e: any) => {
    setSettingsModalOpen(true);
  };

  const onCloseSettingsModal = () => {
    setSettingsModalOpen(false);
  };

  const onSubmit = (data: unknown) => {};

  return (
    <AppBar position="static">
      <Toolbar
        style={{
          backgroundColor: "var(--header)",
          height: "40px",
          minHeight: "40px",
        }}
      >
        {isUnibotics ? (
          <a href="/apps">
            <LogoUniboticsIcon className="bt-jde-icon" fill="var(--icon)" />
          </a>
        ) : (
          <LogoIcon className="bt-jde-icon" fill="var(--icon)" />
        )}
        <h1 className="bt-Header-text">
          {isUnibotics ? "Projects" : "BT Studio IDE"}
        </h1>
        <ProjectModal
          isOpen={isProjectModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseProjectModal}
          currentProject={currentProjectname}
          existingProjects={existingProjects}
          setExistingProjects={setExistingProjects}
          createProject={onCreateProject}
        />

        <SettingsModal
          isOpen={isSettingsModalOpen}
          onSubmit={onSubmit}
          onClose={onCloseSettingsModal}
          currentProjectname={currentProjectname}
        />

        <span className="bt-project-name-box">
          <div className="bt-project-name">{currentProjectname}</div>
        </span>
        <div className="bt-header-button-container">
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
            id="open-settings-manager"
            onClick={onOpenSettingsModal}
            title="Settings"
          >
            <SettingsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </button>
          <Dropdown
            className="bt-header-button"
            id="open-settings-manager"
            title="Layout"
            width={120}
            down
            setter={setLayout}
            possibleValues={["only-editor", "only-viewers", "both"]}
          >
            <SettingsIcon className="bt-header-icon" fill={"var(--icon)"} />
          </Dropdown>
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

const Dropdown = ({
  className,
  id,
  title,
  width,
  down,
  setter,
  possibleValues,
  children,
}: {
  className: string;
  id: string;
  title: string;
  width: number;
  down: boolean;
  setter: Function;
  possibleValues: any[];
  children: any;
}) => {
  const [open, setOpen] = useState<boolean>(false);
  const [right, setRight] = useState<any>(width / 2 + 13);
  const dropdown = useRef<HTMLDivElement>(null);

  const changeValue = (e: any, value: any) => {
    e.preventDefault();
    setter(value);
    setOpen(false);
  };

  const closeOpenMenus = (e: any) => {
    if (open && !dropdown.current?.contains(e.target)) {
      setOpen(false);
    }
  };

  const checkPosition = (x: number) => {
    if (x + width / 2 > window.innerWidth) {
      // To the left
      setRight(x);
    } else if (x < width / 2) {
      // To the right
      setRight(x - width);
    } else {
      // In the middle
      setRight(x - width / 2 + 13);
    }
  };

  document.addEventListener("mousedown", closeOpenMenus);

  return (
    <div ref={dropdown}>
      <button
        className={className}
        id={id}
        title={title}
        onClick={(e) => {
          checkPosition(e.clientX);
          e.preventDefault();
          setOpen(!open);
        }}
      >
        {children}
      </button>
      {open && (
        <div
          className="bt-dropdown-list"
          style={{ width: `${width}px`, left: `${right}px` }}
        >
          {possibleValues.map((name, index) => (
            <button onClick={(e: any) => changeValue(e, name)}>{name}</button>
          ))}
        </div>
      )}
    </div>
  );
};
