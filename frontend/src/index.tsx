import React from "react";
import { createRoot } from "react-dom/client";
import "./index.css";
import { RouterProvider, createBrowserRouter } from "react-router";
import BtRoutes from "./routes";

// Clear stored state on app startup to ensure fresh state on every launch
// This removes theme preferences, search filters, and other cached UI state
const clearStoredState = () => {
  // Clear localStorage items (theme preferences)
  window.localStorage.removeItem("themeType");
};

// Execute immediately before React mounts
clearStoredState();
const router = createBrowserRouter([BtRoutes]);

const container = document.getElementById("root");
const root = createRoot(container!);
root.render(
  <React.StrictMode>
    <RouterProvider router={router} />
  </React.StrictMode>,
);
