import React from 'react';
import AppBar from '@mui/material/AppBar';
import Toolbar from '@mui/material/Toolbar';
import Typography from '@mui/material/Typography';

import './HeaderMenu.css'

const HeaderMenu = () => {
  return (
    <AppBar position="static" sx={{ backgroundColor: '#12494c' }}>
      <Toolbar>
        <h1 className="Header-text">Behavior Tree Web Editor</h1>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
