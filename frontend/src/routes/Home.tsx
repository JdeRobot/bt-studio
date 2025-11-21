import React, { lazy, Suspense, useState } from "react";
import { HomeHeader } from "BtComponents/HeaderMenu";
import { StyledAppContainer } from "BtStyles/App.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import {
  StyledAction,
  StyledActionsContainer,
  StyledActionsSection,
  StyledHomeContainer,
  StyledHomeContent,
} from "BtStyles/Pages/Home.styles";
import { LoadingMenu } from "BtComponents/ProjectsMenu";
import { StyledVertToggleGroup } from "BtStyles/Pages/HomeMui.styles";
import { ToggleButton } from "@mui/material";
import { StyledSpacer } from "BtStyles/ProjectMenu/ProjectEntry.styles";

const App = () => {
  const theme = useBtTheme();
  const [filter, setFilter] = useState<string | undefined>(undefined);

  const ProjectsMenu = lazy(() => import("BtComponents/ProjectsMenu"));

  const handleUserFilter = (
    event: React.MouseEvent<HTMLElement>,
    newFilter: string,
  ) => {
    setFilter(newFilter);
  };

  return (
    <StyledAppContainer bg={theme.palette.bg} hoverStyle={theme.hoverStyle}>
      <HomeHeader />
      <StyledHomeContainer bg={theme.palette.bg}>
        <StyledActionsContainer>
          <StyledAction
            bg={theme.palette.primary}
            color={theme.palette.text}
            roundness={theme.roundness}
            to="create_project"
          >
            New project
          </StyledAction>
          <StyledActionsSection>
            <StyledVertToggleGroup
              value={filter}
              onChange={handleUserFilter}
              exclusive
              bg={theme.palette.bg!}
              selBg={theme.palette.secondary!}
              text={theme.palette.text!}
              roundness={theme.roundness ?? 1}
            >
              <ToggleButton value="">All projects</ToggleButton>
              <ToggleButton value="You">My projects</ToggleButton>
              <ToggleButton value="shared" disabled>
                Shared projects
              </ToggleButton>
              <StyledSpacer bg={theme.palette.bgLight} />
            </StyledVertToggleGroup>
          </StyledActionsSection>
        </StyledActionsContainer>
        <StyledHomeContent bg={theme.palette.bgDark}>
          <Suspense fallback={<LoadingMenu />}>
            <ProjectsMenu userFilter={filter} />
          </Suspense>
        </StyledHomeContent>
      </StyledHomeContainer>
    </StyledAppContainer>
  );
};

export default App;
