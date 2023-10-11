import React from 'react';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';
import './common.css'

const HeaderMenu = () => {
  return (
    <AppBar position="static" sx={{ backgroundColor: '#12494c' }}>
      <Toolbar>
        <h2 className="App-text">Behavior Tree Web Editor</h2>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
