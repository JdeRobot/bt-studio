import React, { useEffect, useState } from 'react';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';
import axios from 'axios';

import logo_img from './img/logo.png'

import './HeaderMenu.css'
import change_project_img from './img/change_project.svg'
import save_project_img from './img/save_project.svg'
import universes_img from './img/universes.svg'
import ProjectModal from './modals/ProjectModal';

var dropdown_shown = false;

const HeaderMenu = ( {setCurrentProjectname, currentProjectname, modelJson, projectChanges, setProjectChanges, openError} ) => {

  const [isProjectModalOpen, setProjectModalOpen] = useState(true);
  const [existingProjects, setExistingProjects] = useState("");

  const createProject = (projectName) => {
  
    if (projectName === null || projectName === '') {
      // User pressed cancel or entered an empty string
      return;
    }
  
    const apiUrl = `/tree_api/create_project?project_name=${encodeURIComponent(projectName)}`;
    axios.get(apiUrl)
      .then(response => {
        if (response.data.success) {
          // Successfully created the project
          setCurrentProjectname(projectName);
          console.log('Project created successfully');
        } 
      })
      .catch(error => {
        if (error.response) {
          // The request was made and the server responded with a status code
          // that falls out of the range of 2xx
          if (error.response.status === 409) {
            openError(`The project ${projectName} already exists`);
          } else {
            // Handle other statuses or general API errors
            openError('Unable to connect with the backend server. Please check the backend status.');
          }
        }
      });
  };

  const changeProject = function(projectName) {
    
    if (projectName === null || projectName === '') {
      // User pressed cancel or entered an empty string
      return;
    }

    if (existingProjects.includes(projectName)) {
      // Project exists, proceed to change
      setCurrentProjectname(projectName);
      console.log(`Switched to project ${projectName}`);
    } else {
      // Project doesn't exist
      openError(`The project ${projectName} does not exist`);
    }
  }

  const openProjectView = (e) => {
    setProjectModalOpen(true)
    saveProject();
  };

  const saveProject = () => {
  
    // Assuming modelJson and currentProjectname are correctly populated
    if (!modelJson || !currentProjectname) {
      console.error('Either modelJson or currentProjectname is not set.');
      openError("Please, select or create a project to store changes")
      return;
    }

    setProjectChanges(false);

    axios.post('/tree_api/save_project/', {
      project_name: currentProjectname,
      graph_json: modelJson
    })
    .then(response => {
      if (response.data.success) {
        console.log('Project saved successfully.');
      } else {
        console.error('Error saving project:', response.data.message || 'Unknown error');
      }
    })
    .catch(error => {
      console.error('Axios Error:', error);
    });

  };

  const handleCloseProjectModal = (project) => {
    if (project) {
      changeProject(project)
    }
    setProjectModalOpen(false);
  };

  const handleFormSubmit = (data) => {
  }
  
  return (
    <AppBar position="static">
      <Toolbar>
        <img src={logo_img} className="jde-icon" alt="JdeRobot logo"></img>
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

        <div className='header-button-container'>
          {currentProjectname && (
              <span className="project-name-box">
                <div className='project-name'>{currentProjectname}</div>
                {projectChanges && <div className="small-text">Unsaved</div>}
              </span>
          )}
          
          <button className="header-button" onClick={openProjectView} title="Change project">
            <img className="header-icon" src={change_project_img}></img>
          </button>
          <button className="header-button" onClick={openProjectView} title="Universe menu">
            <img className="header-icon" src={universes_img}></img>
          </button>
          <button className="header-button" onClick={saveProject} title="Save project">
            <img className="header-icon" src={save_project_img}></img>
          </button>
        </div>


      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
