import App from "./App";
import { BtStudio, CreatePage, EditPage, Home } from "BtRoutes";

const routes = {
  path: "projects",
  Component: App,
  children: [
    { index: true, Component: Home },
    { path: "create_project/*", Component: CreatePage },
    { path: "edit/:proj_id", Component: EditPage },
    { path: "studio/:proj_id", Component: BtStudio },
  ],
};

export default routes;
