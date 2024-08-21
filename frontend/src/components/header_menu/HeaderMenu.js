import React, { useEffect, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import Typography from "@mui/material/Typography";
import axios from "axios";

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
import ProjectModal from "./modals/ProjectModal";
import UniversesModal from "./modals/UniverseModal";
import SettingsModal from "../settings_popup/SettingsModal";

const HeaderMenu = ({
  setCurrentProjectname,
  currentProjectname,
  setCurrentUniverseName,
  currentUniverseName,
  launchUniverse,
  terminateUniverse,
  changeUniverse,
  modelJson,
  projectChanges,
  setProjectChanges,
  openError,
  settingsProps,
  onDownloadApp,
  onRunApp,
  isAppRunning,
  onResetApp,
}) => {
  const [isProjectModalOpen, setProjectModalOpen] = useState(true);
  const [isUniversesModalOpen, setUniversesModalOpen] = useState(false);
  const [isSettingsModalOpen, setSettingsModalOpen] = useState(false);
  const [existingProjects, setExistingProjects] = useState("");

  const createProject = async (projectName) => {
    if (projectName === null || projectName === "") {
      // User pressed cancel or entered an empty string
      return;
    }

    try {
      const apiUrl = `/tree_api/create_project?project_name=${encodeURIComponent(projectName)}`;
      const response = await axios.get(apiUrl);
      if (response.data.success) {
        // Successfully created the project
        setCurrentProjectname(projectName);
        console.log("Project created successfully");
      }
    } catch (error) {
      if (error.response) {
        // The request was made and the server responded with a status code
        // that falls out of the range of 2xx
        if (error.response.status === 409) {
          openError(`The project ${projectName} already exists`);
        } else {
          // Handle other statuses or general API errors
          openError(
            "Unable to connect with the backend server. Please check the backend status."
          );
        }
      }
    }
  };

  const changeProject = function (projectName) {
    if (projectName === null || projectName === "") {
      // User pressed cancel or entered an empty string
      return;
    }

    if (existingProjects.includes(projectName)) {
      // Project exists, proceed to change
      setCurrentProjectname(projectName);
      // Close the universe
      if (currentUniverseName) {
        console.log(currentUniverseName);
        terminateUniverse();
      }
      setCurrentUniverseName(null);
      console.log("Universe terminated!");

      console.log(`Switched to project ${projectName}`);
    } else {
      // Project doesn't exist
      openError(`The project ${projectName} does not exist`);
    }
  };

  const openProjectView = (e) => {
    setProjectModalOpen(true);
    saveProject();
  };

  const openUniversesView = (e) => {
    setUniversesModalOpen(true);
    saveProject();
  };

  const openSettingsView = (e) => {
    setSettingsModalOpen(true);
    saveProject();
  };

  const saveProject = async () => {
    // Assuming modelJson and currentProjectname are correctly populated
    if (!modelJson || !currentProjectname) {
      console.error("Either modelJson or currentProjectname is not set.");
      openError("Please, select or create a project to store changes");
      return;
    }

    setProjectChanges(false);

    try {
      const response = await axios.post("/tree_api/save_project/", {
        project_name: currentProjectname,
        graph_json: modelJson,
      });
      if (response.data.success) {
        console.log("Project saved successfully.");
      } else {
        console.error(
          "Error saving project:",
          response.data.message || "Unknown error"
        );
      }
    } catch (error) {
      console.error("Axios Error:", error);
    }
  };

  const handleCloseProjectModal = (project) => {
    if (project) {
      changeProject(project);
    }
    setProjectModalOpen(false);
  };

  const handleCloseUniversesModal = (universe_name) => {
    setUniversesModalOpen(false);

    if (universe_name === undefined) {
      return;
    }

    if (currentUniverseName !== null) {
      if (universe_name !== currentUniverseName) {
        setCurrentUniverseName(universe_name);
        changeUniverse(universe_name);
      }
    } else {
      setCurrentUniverseName(universe_name);
      launchUniverse(universe_name);
    }
  };

  const handleCloseSettingsModal = () => {
    setSettingsModalOpen(false);
  };

  const handleFormSubmit = (data) => {};

  return (
    <AppBar position="static">
      <Toolbar>
        <LogoIcon className="jde-icon" fill="var(--icon)" />
        <h1 className="Header-text">BT Studio IDE</h1>
        <ProjectModal
          isOpen={isProjectModalOpen}
          onSubmit={handleFormSubmit}
          onClose={handleCloseProjectModal}
          currentProject={currentProjectname}
          existingProjects={existingProjects}
          setExistingProjects={setExistingProjects}
          createProject={createProject}
          openError={openError}
        />
        <UniversesModal
          isOpen={isUniversesModalOpen}
          onSubmit={handleFormSubmit}
          onClose={handleCloseUniversesModal}
          currentProject={currentProjectname}
          openError={openError}
        />

        <SettingsModal
          isOpen={isSettingsModalOpen}
          onSubmit={handleFormSubmit}
          onClose={handleCloseSettingsModal}
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
            onClick={openProjectView}
            title="Change project"
          >
            <ProjectsIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={openUniversesView}
            title="Universe menu"
          >
            <UniversesIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={openSettingsView}
            title="Settings"
          >
            <SettingsIcon className="header-icon" fill={"var(--icon)"} />
          </button>
          <button
            className="header-button"
            onClick={saveProject}
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
          <button className="header-button" onClick={onRunApp} title="Run app">
            {isAppRunning ? (
              <StopIcon className="header-icon" fill={"var(--icon)"} />
            ) : (
              <RunIcon className="header-icon" fill={"var(--icon)"} />
            )}
          </button>
          <button
            className="node-action-button"
            onClick={onResetApp}
            title="Reset app"
          >
            <ResetIcon className="icon action-icon" stroke={"var(--icon)"} />
          </button>
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
