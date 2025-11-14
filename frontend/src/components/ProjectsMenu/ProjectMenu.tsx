import React, { useEffect, useState } from "react";
import { listProjects } from "BtApi/TreeWrapper";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledSectionName } from "BtStyles/ProjectMenu/ProjectMenu.styles";
import ProjectEntry from "./ProjectEntry";

const Menu = () => {
  const theme = useBtTheme();

  const [projects, setProjects] = useState([]);

  const getProjects = async () => {
    try {
      const response = await listProjects();
      setProjects(response);
    } catch (e) {
      setProjects([]);
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        // error("Error while fetching project list: " + e.message);
      }
    }
  };

  useEffect(() => {
    getProjects();
  }, []);

  return (
    <>
      <StyledSectionName color={theme.palette.text}>
        My projects
      </StyledSectionName>
      <ProjectEntry projects={projects}/>
    </>
  );
};

export const LoadingMenu = () => {
  return (
    <h1>
      Loading ...
    </h1>
  );
}

export default Menu;
