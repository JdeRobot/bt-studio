import React, { useEffect, useState } from 'react';
import axios from 'axios';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Menu from '@mui/material/Menu';
import MenuItem from '@mui/material/MenuItem';
import Button from '@mui/material/Button';


const NodeHeader = ({ onNodeTypeSelected }) => {

  const modernButtonStyle = {
    color: '#ffffff',
    width: "130px",
    backgroundColor: 'white',
    color: 'black',
    border: 'none',
    borderRadius: '5px',
    margin: '10px',
    fontSize: '14px',
    marginBottom: '5px',
    '&:hover': {
      backgroundColor: 'black',
    }
  };

  const textStyle = {
    marginBottom: "-3px",
  }

  const [anchorEl, setAnchorEl] = useState(null);
  const [menuLabel, setMenuLabel] = useState("");

  const handleClick = (event, label) => {
    setAnchorEl(event.currentTarget);
    setMenuLabel(label);
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
    useEffect(() => {
      axios.get('/tree_api/get_file_list')
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
    }, []);

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
    }
    return [];
  };

  return (
    <AppBar position="static" sx={{ backgroundColor: '#000000' }}>
      <Toolbar>
        <h2>Tree Editor</h2>
        <Button style={modernButtonStyle} onClick={(e) => handleClick(e, "Sequences")}>
          <div style={textStyle}>
            Sequences
          </div>
        </Button>
        <Button style={modernButtonStyle} onClick={(e) => handleClick(e, "Fallbacks")}>
          <div style={textStyle}>
            Fallbacks
          </div>
        </Button>
        <Button style={modernButtonStyle} onClick={(e) => handleClick(e, "Decorators")}>
          <div style={textStyle}>
            Decorators
          </div>
        </Button>
        <Button style={modernButtonStyle} onClick={(e) => handleClick(e, "Actions")}>
          <div style={textStyle}>
            Actions
          </div>
        </Button>
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
      </Toolbar>
    </AppBar>
  );
  
};

export default NodeHeader;
