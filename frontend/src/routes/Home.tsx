import React, { useEffect, useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { ModalEditableList, ModalRow, useError } from "jderobot-ide-interface";
import { deleteProject, listProjects } from "BtApi/TreeWrapper";
import { StyledAppContainer } from "BtStyles/App.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

const App = () => {
  const { error } = useError();
  const theme = useBtTheme();
  const [projects, setProjects] = useState([]);

  const navigate = useNavigate();

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

  const deleteProjectFunc = async (project: string) => {
    // Delete and update
    try {
      await deleteProject(project);
      await getProjects();
      console.log("Project deleted successfully");
    } catch (e) {
      if (e instanceof Error) {
        console.error("Error while fetching project list: " + e.message);
        error("Error while fetching project list: " + e.message);
      }
    }
  };

  return (
    <StyledAppContainer bg={theme.palette.background}>
      <HomeHeader />
      <h1>Home Page</h1>
      <Link to="/create_project">New project</Link>
      <ModalRow type="all">
        <ModalEditableList
          list={Object.values(projects)}
          onSelect={(e: any, entry: string) => {
            navigate("/studio/" + entry);
          }}
          onDelete={(e: any, entry: string) => {
            deleteProjectFunc(entry);
            e.stopPropagation();
          }}
        />
      </ModalRow>
    </StyledAppContainer>
  );
};

export default App;
