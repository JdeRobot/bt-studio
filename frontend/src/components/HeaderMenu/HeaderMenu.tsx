import React from "react";
import { useEffect, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { CommsManager, states } from "jderobot-commsmanager";

import { ReactComponent as LogoIcon } from "BtIcons/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "BtIcons/logo_unibotics_monocolor.svg";
import { subscribe, unsubscribe } from "../helper/TreeEditorHelper";
import {
  DocumentationButton,
  DownloadButton,
  HomeButton,
  LayoutButton,
  PlayPauseButton,
  ResetButton,
  SettingsButton,
  TerminateUniverseButton,
  ThemeButton,
} from "../Buttons";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledProject,
} from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";

const HeaderMenu = ({
  currentProjectname,
  setCurrentProjectname,
  manager,
  isUnibotics,
  setLayout,
}: {
  currentProjectname: string;
  setCurrentProjectname: Function;
  manager: CommsManager | null;
  isUnibotics: boolean;
  setLayout: Function;
}) => {
  const theme = useBtTheme();

  // App state
  const [appRunning, setAppRunning] = useState(false);

  const updateState = (e: any) => {
    setAppRunning(e.detail.state === states.RUNNING);
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

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
        <StyledProject color={theme.palette.text}>
          <div>{currentProjectname}</div>
        </StyledProject>
        <StyledHeaderButtonContainer>
          <HomeButton
            project={currentProjectname}
            manager={null}
            setProject={setCurrentProjectname}
            setAppRunning={setAppRunning}
          />
          <ThemeButton />
          <DownloadButton project={currentProjectname} />
          <LayoutButton setLayout={setLayout} />
          <SettingsButton project={currentProjectname} />
          <PlayPauseButton
            project={currentProjectname}
            manager={manager}
            appRunning={appRunning}
            setAppRunning={setAppRunning}
          />
          <ResetButton manager={manager} setAppRunning={setAppRunning} />
          <TerminateUniverseButton
            manager={manager}
            setAppRunning={setAppRunning}
          />
          <DocumentationButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
