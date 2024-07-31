import React, { useEffect, useState } from 'react';
import axios from 'axios';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import './NodeHeader.css';

import del_img from './img/del_node.svg'
import edit_action_img from './img/edit_action.svg'
import help_img from './img/help.svg'
import download_img from './img/download.svg'
import run_img from './img/run.svg'
import stop_img from './img/stop.svg'
import zoom_to_fit_img from './img/zoom_to_fit.svg'

const NodeHeader = ({ onNodeTypeSelected, onDeleteNode, 
  onEditAction, onGenerateApp, onRunApp, isAppRunning, currentProjectname, zoomToFit }) => {

  const [anchorEl, setAnchorEl] = useState(null);
  const [menuLabel, setMenuLabel] = useState("");

  const handleClick = (event, label) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
    if (label === "Actions") {
      fetchActionList();
    }
  };

  const handleClose = () => {
    setAnchorEl(null);
  };

  const handleSelect = (nodeType) => {
    if (onNodeTypeSelected) {
      onNodeTypeSelected(nodeType);
    }
    handleClose();
  };

  // Initialize a state variable to hold the list of action names
  const [actionList, setActionList] = useState([]);
  
  // Fetch the file list and update actionList
  const fetchActionList = () => {

    axios.get(`/tree_api/get_file_list?project_name=${currentProjectname}`)
      .then(response => {
        const files = response.data.file_list;
        if (Array.isArray(files)) {
          const actions = files.map(file => file.replace('.py', ''));
          setActionList(actions);
        } else {
          console.error('API response is not an array:', files);
        }
      })
      .catch(error => {
        console.error('Error fetching files:', error);
      });
  };

  const getMenuItems = () => {
    if (menuLabel === "Sequences") {
      return ["Sequence", "ReactiveSequence", "SequenceWithMemory"];
    } else if (menuLabel === "Fallbacks") {
      return ["Fallback", "ReactiveFallback"];
    } else if (menuLabel === "Decorators") {
      return ["RetryUntilSuccessful", "Inverter",
              "ForceSuccess", "ForceFailure", "KeepRunningUntilFailure", "Repeat",
              "RunOnce", "Delay"];
    } else if (menuLabel === "Actions") {
      return actionList; // Use the action names fetched from the API
    } else if (menuLabel === "Port values") {
      return ["Input port value", "Output port value"]
    }
    return [];
  };

  const openInNewTab = (url) => {
    window.open(url, '_blank').focus();
  }

  return (
    <div className='node-header-container'>

        <h2>Tree Editor</h2>

        <div className='button-container'>
          <button className='node-button' onClick={(e) => handleClick(e, "Sequences")}>
              Sequences
          </button>
          <button className='node-button' onClick={(e) => handleClick(e, "Fallbacks")}>
              Fallbacks
          </button>
          <button className='node-button' onClick={(e) => handleClick(e, "Decorators")}>
              Decorators
          </button>
          <button className='node-button' onClick={(e) => handleClick(e, "Actions")}>
              Actions
          </button>
          <button className='node-button' onClick={(e) => handleClick(e, "Port values")}>
              Port value
          </button>
        </div>

        <Menu
          anchorEl={anchorEl}
          open={Boolean(anchorEl)}
          onClose={handleClose}
        >
          {getMenuItems().map((item) => (
            <MenuItem key={item} onClick={() => handleSelect(item)}>
              {item}
            </MenuItem>
          ))}
        </Menu>

        <div className='action-buttons'>
          <button id='node-action-delete-button' className="node-action-button" onClick={onDeleteNode} title='Delete'>
            <img className="icon action-icon" src={del_img}></img>
          </button>
          <button id='node-action-edit-button' className="node-action-button" onClick={onEditAction} title='Edit'>
            <img className="icon action-icon" src={edit_action_img}></img>
          </button>
          <button id='node-action-zoom-button' className="node-action-button" onClick={zoomToFit} title='Zoom To Fit'>
            <img className="icon action-icon" src={zoom_to_fit_img}></img>
          </button>
          <button id='node-action-help-button' className="node-action-button" onClick={() => {openInNewTab('https://github.com/JdeRobot/bt-studio/tree/unibotics-devel/documentation')}} title='Help'>
            <img className="icon action-icon" src={help_img}></img>
          </button>
          <button className="node-action-button" onClick={onGenerateApp} title='Download app'>
            <img className="icon action-icon" src={download_img}></img>
          </button>
          <button className="node-action-button" onClick={onRunApp} title='Run app'>
            <img className="icon action-icon" src={(isAppRunning) ? stop_img : run_img}></img>
          </button>
        </div>

    </div>
  );
  
};

export default NodeHeader;
