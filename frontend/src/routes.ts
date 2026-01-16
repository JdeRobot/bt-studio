import App from "./App";
import { createBrowserRouter } from "react-router";
import { CreatePage, EditPage, Home } from "BtRoutes";

const router = createBrowserRouter([
  {
    path: "/projects",
    Component: App,
    children: [
      { index: true, Component: Home },
      { path: "create_project/*", Component: CreatePage },
      { path: "edit/:proj_id", Component: EditPage },
      { path: "studio/:proj_id", Component: EditPage },
    ],
  },
]);

export default router