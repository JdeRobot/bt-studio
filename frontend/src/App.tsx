import React from "react";
import { BrowserRouter, Routes, Route} from "react-router-dom";

import { OptionsProvider } from "BtComponents/options/Options";
import { ErrorProvider } from "jderobot-ide-interface";
import { BtThemeProvider } from "BtContexts/BtThemeContext";
import { BtStudio, Home } from "BtRoutes";

const App = () => {
  return (
    <BtThemeProvider>
      <OptionsProvider>
        <ErrorProvider>
          <BrowserRouter>
            <Routes>
              <Route path="/home" element={<Home />} />
              <Route path="/studio/:proj_id" element={<BtStudio />} />
            </Routes>
          </BrowserRouter>
        </ErrorProvider>
      </OptionsProvider>
    </BtThemeProvider>
  );
};

export default App;
