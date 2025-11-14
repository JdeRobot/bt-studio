import React from "react";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { StyledAppContainer } from "BtStyles/App.styles";
import { StyledAction, StyledActionsContainer, StyledHomeContainer, StyledHomeContent } from "BtStyles/Pages/Home.styles";

const App = () => {
  const theme = useBtTheme();
  
  return (
    <StyledAppContainer bg={theme.palette.bg} hoverStyle={theme.hoverStyle}>
      <HomeHeader section="Create new project"/>
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
        </StyledHomeContent>
      </StyledHomeContainer>
    </StyledAppContainer>
  );
};

export default App;
