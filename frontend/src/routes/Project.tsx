import React, { useState, useEffect, useRef } from "react";
import HeaderMenu from "BtComponents/HeaderMenu";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectConfig, saveProjectConfig } from "BtApi/TreeWrapper";

import IdeInterface, { VncViewer, useError } from "jderobot-ide-interface";
import TreeEditorContainer, {
  AddSubtreeButton,
  BTSelectorButtons,
  OtherButtons,
} from "BtComponents/TreeEditor";
import TreeMonitorContainer from "BtComponents/TreeMonitor";
import { explorers } from "BtComponents/Explorers";
import { statusBar } from "BtComponents/StatusBar";
import { editorApi } from "BtComponents/Editors";

import { useParams } from "react-router-dom";
import { StyledAppContainer } from "BtStyles/App.styles";
import { useBtTheme } from "BtContexts/BtThemeContext";
import { GazeboIcon, TerminalIcon, TreeMonitorIcon } from "BtIcons";
import {
  ProjectSettingsProvider,
  useProjectSettings,
} from "BtContexts/ProjectSettingsContext";

const Wrapper = () => {
  const { proj_id } = useParams();
  const { error_critical } = useError();
  const hasUserLimit = window.location.href.includes("unibotics");
  const maxUsers = 15;
  const currentUsers = useRef<number>(0);
  const btAtMaxCapacity = useRef<boolean>(false);

  if (proj_id === undefined) {
    return <></>;
  }

  //Only needed in Unibotics

  useEffect(() => {
    console.log("Current number of users connected: " + currentUsers.current);
    addUser();
    console.log(
      "Now the updated value of users connected is: ",
      currentUsers.current
    );
    console.log(
      "Current value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current
    );
    updateBtAtMaxCapacity(currentUsers.current);
    console.log(
      "Updated value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current
    );

    return () => {
      substractUser();
    };
  }, []);

  /////////////////////////////Functions only used in Unibotics/////////////////

  const addUser = () => {
    currentUsers.current += 1;
  };

  const substractUser = () => {
    currentUsers.current -= 1;
  };

  const updateBtAtMaxCapacity = (currentUserCount: number) => {
    console.log("Entering update of MaxCapacity");
    if (currentUserCount > maxUsers && hasUserLimit) {
      console.log("Too much users!");
      btAtMaxCapacity.current = true;
      error_critical(
        "There's not enough room for you to enter Studio. Please try again later.",
        "../.."
      );
    } else {
      console.log("The user can go in");
      btAtMaxCapacity.current = false;
    }
  };

  return (
    <ProjectSettingsProvider>
      <App projectId={proj_id} />
    </ProjectSettingsProvider>
  );
};

const App = ({ projectId }: { projectId: string }) => {
  const theme = useBtTheme();
  const settings = useProjectSettings();
  const connected = useRef<boolean>(false);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [showSim, setSimVisible] = useState<boolean>(false);
  const [showMonitor, setMonitorVisible] = useState<boolean>(false);
  const [showTerminal, setTerminalVisible] = useState<boolean>(false);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );

  const saveSettings = async (project: string) => {
    const json_settings: { name: string; config: { [id: string]: any } } = {
      name: project,
      config: {},
    };

    Object.entries(settings).map(([key, setting]) => {
      json_settings.config[key] = setting.value;
    });

    console.log("Save: ", project);

    try {
      await saveProjectConfig(project, JSON.stringify(json_settings));
    } catch (e) {
      console.error("Error saving config:", e);
    }
  };

  // RB manager setup

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void
  ) => {
    if (!manager || manager?.getState() != "idle") {
      return;
    }
    try {
      const currManager = CommsManager.getInstance();
      await currManager.connect();
      console.log("Connected!", currManager.getState());
      connected.current = true;
      setManager(currManager);
      if (callback) {
        waitManagerState(desiredState ? desiredState : "connected", callback);
      }
    } catch (e: unknown) {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 2000, desiredState, callback);
    }
  };

  const waitManagerState = async (state: string, callback: any) => {
    if (manager?.getState() === state) {
      callback();
    } else {
      return setTimeout(waitManagerState, 100, state, callback);
    }
  };

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
    getProjectConfig(projectId, settings);

    return () => {
      const currManager = CommsManager.getInstance();
      if (currManager) {
        currManager.disconnect();
        console.log(currManager);
      }
      saveSettings(projectId);
    };
  }, []);

  // useUnload((event: any) => {
  //   event.preventDefault();
  //   if (manager) {
  //     manager.disconnect();
  //     connected.current = false;
  //   }
  //   // saveSettings(projectToSave.current);
  //   return (event.returnValue = "");
  // });

  const treeMonitor = {
    component: (
      <TreeMonitorContainer commsManager={manager} project={projectId} />
    ),
    icon: <TreeMonitorIcon />,
    name: "Tree Monitor",
    active: showMonitor,
    activate: setMonitorVisible,
  };

  const gazeboViewer = {
    component: (
      <VncViewer
        commsManager={manager}
        port={6080}
        message={"Click Play to connect to the Robotics Backend"}
      />
    ),
    icon: <GazeboIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: (
      <VncViewer
        commsManager={manager}
        port={6082}
        message={"Click Play to connect to the Robotics Backend"}
      />
    ),
    icon: <TerminalIcon />,
    name: "Terminal",
    active: showTerminal,
    activate: setTerminalVisible,
  };

  const treeEditor = {
    component: TreeEditorContainer,
    buttons: [
      <BTSelectorButtons key="BTSelectorButtons" project={projectId} />,
      <AddSubtreeButton key="AddSubtreeButton" project={projectId} />,
      <OtherButtons key="OtherButtons" />,
    ],
    name: "Tree editor",
    language: "custom_tree_editor",
    trigger: [{ group: "Trees", extension: "json" }],
  };

  return (
    <StyledAppContainer bg={theme.palette.bg}>
      <HeaderMenu
        projectId={projectId}
        connectManager={connectWithRetry}
        setLayout={setLayout}
      />
      <IdeInterface
        commsManager={manager}
        connectManager={connectWithRetry}
        project={projectId}
        explorers={explorers}
        api={editorApi}
        extraEditors={[treeEditor]}
        viewers={[treeMonitor, gazeboViewer, terminalViewer]}
        options={[]}
        layout={layout}
        statusBarComponents={statusBar}
      />
    </StyledAppContainer>
  );
};

export default Wrapper;
