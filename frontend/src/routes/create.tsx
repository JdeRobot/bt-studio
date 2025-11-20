import React, { useEffect } from "react";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledAppContainer } from "BtStyles/App.styles";
import {
  StyledAction,
  StyledActionsContainer,
  StyledHomeContainer,
  StyledHomeContent,
} from "BtStyles/Pages/Home.styles";
import { CreateProjectMenu } from "BtComponents/CreateProject";
import { useParams } from "react-router-dom";

const App = () => {
  const theme = useBtTheme();
  const { "*": projId } = useParams();

  return (
    <StyledAppContainer bg={theme.palette.bg} hoverStyle={theme.hoverStyle}>
      <HomeHeader section="Create new project" />
      <StyledHomeContainer bg={theme.palette.bg}>
        <StyledActionsContainer>
        </StyledActionsContainer>
        <StyledHomeContent bg={theme.palette.bgDark}>
          <CreateProjectMenu projId={projId === "" ? undefined : projId} />
        </StyledHomeContent>
      </StyledHomeContainer>
    </StyledAppContainer>
  );
};

export default App;
