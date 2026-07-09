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
  TerminateWorldButton,
  ThemeButton,
  ExportButton,
} from "../Buttons";
import {
  StyledHeaderButtonContainer,
  StyledHeaderText,
  StyledProject,
} from "BtStyles/Header/HeaderMenu.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { getProjectInfo } from "BtApi/TreeWrapper";
import { Layout } from "jderobot-ide-interface";
import { CommsManager } from "jderobot-commsmanager";
import { subscribe, unsubscribe } from "BtHelpers/utils";
import ConnectButton from "BtComponents/Buttons/Connect";

const HeaderMenu = ({
  project,
  setLayout,
  connectManager,
  commsManager,
}: {
  project: string;
  setLayout: (layout: Layout) => void;
  connectManager: (
    desiredState?: string,
    callback?: () => void,
  ) => Promise<void>;
  commsManager: CommsManager | null;
}) => {
  const theme = useBtTheme();
  const [name, setName] = useState<string | undefined>(undefined);
  const isUnibotics = window.location.href.includes("unibotics");

  const getInfo = async (id: string) => {
    const info = await getProjectInfo(id);
    setName(info.name);
  };

  useEffect(() => {
    if (project) {
      getInfo(project);
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
          <ExportButton project={project} />
          <DownloadButton project={project} />
          <LayoutButton setLayout={setLayout} />
          {/* <SettingsButton project={project} /> */}
          <ExecutionControl
            project={project}
            commsManager={commsManager}
            connectManager={connectManager}
          />
          <DocumentationButton />
        </StyledHeaderButtonContainer>
      </Toolbar>
    </AppBar>
  );
};

const ExecutionControl = ({
  project,
  commsManager,
  connectManager,
}: {
  project: string;
  commsManager: CommsManager | null;
  connectManager: (
    desiredState?: string,
    callback?: () => void,
  ) => Promise<void>;
}) => {
  const [state, setState] = useState<string | undefined>(
    commsManager?.getState(),
  );

  const updateState = (e: unknown) => {
    const T = CustomEvent<{ detail: unknown }>;
    if (e instanceof T) {
      setState(e.detail.state);
    }
  };

  useEffect(() => {
    subscribe("CommsManagerStateChange", updateState);

    return () => {
      unsubscribe("CommsManagerStateChange", () => {});
    };
  }, []);

  const viewConnect = state === undefined || state === "idle";

  return (
    <>
      {viewConnect ? (
        <ConnectButton connectManager={connectManager} />
      ) : (
        <>
          <PlayPauseButton project={project} />
          <ResetButton />
          <TerminateWorldButton />
        </>
      )}
    </>
  );
};

export default HeaderMenu;
