import { useEffect, useRef, useState } from "react";
import AppBar from "@mui/material/AppBar";
import Toolbar from "@mui/material/Toolbar";
import { CommsManager, states } from "jderobot-commsmanager";

import { ReactComponent as LogoIcon } from "../icons/logo_jderobot_monocolor.svg";
import { ReactComponent as LogoUniboticsIcon } from "../icons/logo_unibotics_monocolor.svg";

import "./HeaderMenu.css";
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
          backgroundColor: "var(--header)",
          height: "50px",
          minHeight: "50px",
        }}
      >
        {isUnibotics ? (
          <a href="/apps">
            <LogoUniboticsIcon className="bt-jde-icon" fill="var(--icon)" />
          </a>
        ) : (
          <LogoIcon className="bt-jde-icon" fill="var(--icon)" />
        )}
        <h1 className="bt-Header-text">
          {isUnibotics ? "Projects" : "BT Studio IDE"}
        </h1>
        <span className="bt-project-name-box">
          <div className="bt-project-name">{currentProjectname}</div>
        </span>
        <div className="bt-header-button-container">
          <HomeButton
            project={currentProjectname}
            manager={null}
            setProject={setCurrentProjectname}
            setAppRunning={setAppRunning}
          />
          <ThemeButton/>
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
        </div>
      </Toolbar>
    </AppBar>
  );
};

export default HeaderMenu;
