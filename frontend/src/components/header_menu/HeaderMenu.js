import React, { useEffect, useState } from 'react';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';
import axios from 'axios';

import logo_img from './img/logo.png'

import './HeaderMenu.css'
import add_project_img from './img/add_project.svg'

const HeaderMenu = ( {setCurrentProjectname, currentProjectname} ) => {

  const createProject = () => {

    const projectName = window.prompt('Enter the name for the new project:');

    if (projectName === null || projectName === '') {
      // User pressed cancel or entered an empty string
      return;
    }

    const apiUrl = `/tree_api/create_project?project_name=${projectName}`;

    axios.get(apiUrl)
      .then(response => {

        if (response.data.success) {
          // Successfully created the project
          setCurrentProjectname(projectName);
          localStorage.setItem('project_name', projectName);
          console.log('Project created successfully');
        } 
      })
      .catch(error => {
        // Failed to create the project
        window.alert(`The project ${projectName} already exists`);
      });
  };

  const changeProject = () => {

    // API call to get the list of existing projects
    const listApiUrl = `/tree_api/get_project_list`;
  
    axios.get(listApiUrl)
      .then(response => {

        const existingProjects = response.data.project_list;
        const projectName = window.prompt('Enter the name of the project you want to switch to:');
  
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
          window.alert(`The project ${projectName} does not exist`);
        }
      })
      .catch(error => {
        console.error('Error while fetching project list:', error);
        window.alert(`An error occurred while fetching the project list`);
      });
  };
  
  return (
    <AppBar position="static" sx={{ backgroundColor: '#12494c' }}>
      <Toolbar>
        <img src={logo_img} className="jde-icon" alt="JdeRobot logo"></img>
        <h1 className="Header-text">BT Studio IDE</h1>
        <h2>{currentProjectname}</h2>
        <button className="node-action-button" onClick={createProject}>
          <img className="header-icon" src={add_project_img}></img>
        </button>
        <button className="node-action-button" onClick={changeProject}>
          <img className="header-icon" src={add_project_img}></img>
        </button>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
