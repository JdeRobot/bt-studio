import React from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { RouterProvider, createBrowserRouter } from "react-router-dom";

import App from "./App";
import { BtStudio, CreatePage, EditPage, Home } from "BtRoutes";

// Clear stored state on app startup to ensure fresh state on every launch
// This removes theme preferences, search filters, and other cached UI state
const clearStoredState = () => {
  // Clear localStorage items (theme preferences)
  window.localStorage.removeItem("themeType");
};

// Execute immediately before React mounts
clearStoredState();
const router = createBrowserRouter([
  {
    path: "projects",
    Component: App,
    children: [
      { index: true, Component: Home },
      { path: "create_project/*", Component: CreatePage },
      { path: "edit/:proj_id", Component: EditPage },
      { path: "studio/:proj_id", Component: BtStudio },
    ],
  },
]);

const container = document.getElementById("root");
const root = createRoot(container!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>,
);
