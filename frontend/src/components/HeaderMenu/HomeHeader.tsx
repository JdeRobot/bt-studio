import React from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";

import { ReactComponent as LogoIcon } from "BtComponents/icons/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "BtComponents/icons/logo_unibotics_monocolor.svg";
import {
  DocumentationButton,
  HomeButton,
  ThemeButton,
} from "BtComponents/Buttons";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledSection,
} from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

const Header = ({ section }: { section?: string }) => {
  const theme = useBtTheme();
  const isUnibotics = window.location.href.includes("unibotics");
  const isHome = window.location.href.includes("home");

  return (
    <AppBar position="static">
      <Toolbar
        style={{
          backgroundColor: theme.palette.primary,
          height: "50px",
          minHeight: "50px",
        }}
      >
        {isUnibotics ? (
          <a href="/apps">
            <LogoUniboticsIcon
              style={{ width: "40px", height: "40px", marginRight: "10px" }}
              fill={theme.palette.text}
            />
          </a>
        ) : (
          <LogoIcon
            style={{ width: "40px", height: "40px", marginRight: "10px" }}
            fill={theme.palette.text}
          />
        )}
        <StyledHeaderText color={theme.palette.text}>
          {isUnibotics ? "Projects" : "BT Studio IDE"}
        </StyledHeaderText>
        {section && (
          <StyledSection color={theme.palette.text}>
            <div>{section}</div>
          </StyledSection>
        )}
        <StyledHeaderButtonContainer>
          {!isHome && (
            <HomeButton
              project={""}
              manager={null}
              setProject={() => {}}
              setAppRunning={() => {}}
            />
          )}
          <ThemeButton />
          <DocumentationButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default Header;
