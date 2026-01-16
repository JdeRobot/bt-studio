import React from "react";
import { Outlet } from "react-router-dom";
import { ErrorProvider } from "jderobot-ide-interface";
import { BtThemeProvider } from "BtContexts/BtThemeContext";

const App = () => {
  return (
    <BtThemeProvider>
      <ErrorProvider>
        <Outlet />
      </ErrorProvider>
    </BtThemeProvider>
  );
};

export default App;
