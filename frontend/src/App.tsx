import React from "react";
import { BrowserRouter, Routes, Route } from "react-router-dom";

import { ErrorProvider } from "jderobot-ide-interface";
import { BtThemeProvider } from "BtContexts/BtThemeContext";
import { BtStudio, CreatePage, EditPage, Home } from "BtRoutes";

const App = () => {
  return (
    <BtThemeProvider>
      <ErrorProvider>
        <BrowserRouter>
          <Routes>
            <Route path="/projects">
              <Route index element={<Home />} />
              <Route path="create_project/*" element={<CreatePage />} />
              <Route path="edit/:proj_id" element={<EditPage />} />
              <Route path="studio/:proj_id" element={<BtStudio />} />
            </Route>
          </Routes>
        </BrowserRouter>
      </ErrorProvider>
    </BtThemeProvider>
  );
};

export default App;
