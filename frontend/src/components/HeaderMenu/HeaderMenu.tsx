import React from "react";
import { useEffect, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";

import { ReactComponent as LogoIcon } from "BtComponents/icons/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "BtComponents/icons/logo_unibotics_monocolor.svg";
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
import { getProjectInfo } from "BtApi/TreeWrapper";

const HeaderMenu = ({
  projectId,
  connectManager,
  setLayout,
}: {
  projectId: string;
  connectManager: (
    desiredState?: string,
    callback?: () => void,
  ) => Promise<void>;
  setLayout: Function;
}) => {
  const theme = useBtTheme();
  const [name, setName] = useState<string | undefined>(undefined);
  const isUnibotics = window.location.href.includes("unibotics");

  const getInfo = async (id: string) => {
    const info = await getProjectInfo(id);
    setName(info.name);
  };

  useEffect(() => {
    if (projectId) {
      getInfo(projectId);
    }
  }, []);

  return (
    <AppBar position="static">
      <Toolbar
        style={{
          backgroundColor: theme.palette.bg,
          height: "50px",
          minHeight: "50px",
        }}
      >
        {isUnibotics ? (
          <a href="/apps">
            <LogoUniboticsIcon
              viewBox="0 0 400 400"
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
          <div>{name}</div>
        </StyledProject>
        <StyledHeaderButtonContainer>
          <HomeButton />
          <ThemeButton />
          <DownloadButton project={projectId} />
          <LayoutButton setLayout={setLayout} />
          <SettingsButton project={projectId} />
          <PlayPauseButton
            project={projectId}
            connectManager={connectManager}
          />
          <ResetButton />
          <TerminateUniverseButton />
          <DocumentationButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
