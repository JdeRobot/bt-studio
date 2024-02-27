import React, { useEffect, useState } from 'react';
import axios from 'axios';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import Button from '@mui/material/Button';
import './NodeHeader.css';

import del_img from './img/del_node.svg'
import add_input_img from './img/add_input.svg'
import add_output_img from './img/add_output.svg'
import download_img from './img/download.svg'
import run_img from './img/run.svg'

const NodeHeader = ({ onNodeTypeSelected, onDeleteNode, 
  onAddInputPort, onAddOutputPort, onGenerateApp, onRunApp, currentProjectname }) => {

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
          <button className="node-action-button" onClick={onDeleteNode} title='Delete node'>
            <img className="icon" src={del_img}></img>
          </button>
          <button className="node-action-button" onClick={onAddInputPort} title='Add input port'>
            <img className="icon" src={add_input_img}></img>
          </button>
          <button className="node-action-button" onClick={onAddOutputPort} title='Add output port'>
            <img className="icon" src={add_output_img}></img>
          </button>
          <button className="node-action-button" onClick={onGenerateApp} title='Download app'>
            <img className="icon" src={download_img}></img>
          </button>
          <button className="node-action-button" onClick={onRunApp} title='Run app'>
            <img className="icon" src={run_img}></img>
          </button>
        </div>

    </div>
  );
  
};

export default NodeHeader;
