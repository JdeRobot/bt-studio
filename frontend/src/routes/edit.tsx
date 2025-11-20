import React from "react";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledAppContainer } from "BtStyles/App.styles";
import {
  StyledAction,
  StyledActionsContainer,
  StyledHomeContainer,
  StyledHomeContent,
} from "BtStyles/Pages/Home.styles";
import { useParams } from "react-router-dom";
import { EditProjectMenu } from "BtComponents/EditProject";

const App = () => {
  const theme = useBtTheme();
  const { proj_id } = useParams();

  if (proj_id === undefined) {
    return <></>;
  }

  return (
    <StyledAppContainer bg={theme.palette.bg} hoverStyle={theme.hoverStyle}>
      <HomeHeader section={`Edit ${proj_id} project`} />
      <StyledHomeContainer bg={theme.palette.bg}>
        <StyledActionsContainer>
          <StyledAction
            bg={theme.palette.primary}
            color={theme.palette.text}
            roundness={theme.roundness}
            to="/create_project"
          >
            Another action
          </StyledAction>
        </StyledActionsContainer>
        <StyledHomeContent bg={theme.palette.bgDark}>
          <EditProjectMenu projId={proj_id} />
        </StyledHomeContent>
      </StyledHomeContainer>
    </StyledAppContainer>
  );
};

export default App;
