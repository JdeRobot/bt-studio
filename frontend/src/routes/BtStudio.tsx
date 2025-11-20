import React, { useState, useEffect, useRef } from "react";
import { useUnload } from "BtHooks/useUnload";
import HeaderMenu from "BtComponents/HeaderMenu";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectConfig, saveProjectConfig } from "BtApi/TreeWrapper";

import { OptionsContext } from "BtComponents/options/Options";
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

const App = () => {
  const theme = useBtTheme();
  const isUnibotics = window.location.href.includes("unibotics");
  const { proj_id } = useParams();
  const projectId = proj_id;

  if (projectId === undefined) {
    return <></>;
  }

  // const [projectToSave, setProjectToSave] = useState(currentProjectname);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [showSim, setSimVisible] = useState<boolean>(false);
  const [showMonitor, setMonitorVisible] = useState<boolean>(false);
  const [showTerminal, setTerminalVisible] = useState<boolean>(false);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both"
  );

  //Only needed in Unibotics
  const maxUsers = 15;
  const currentUsers = React.useRef<number>(0);
  const btAtMaxCapacity = React.useRef<boolean>(false);
  const projectToSave = React.useRef<string>(projectId);
  const { error_critical } = useError();

  const settings = React.useContext(OptionsContext);
  const settingsToSave = React.useRef<object>(settings);

  const saveSettings = async (project: string) => {
    const json_settings: { name: string; config: { [id: string]: any } } = {
      name: project,
      config: {},
    };

    Object.entries(settingsToSave.current).map(([key, setting]) => {
      json_settings.config[key] = setting.value;
    });

    console.log("Save: ", projectToSave.current);

    try {
      await saveProjectConfig(project, JSON.stringify(json_settings));
    } catch (e) {
      console.error("Error saving config:", e);
    }
  };
  //////////////////////////s////////////////////////////

  // RB manager setup
  const connected = useRef<boolean>(false);

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

    const manager = CommsManager.getInstance();
    if (btAtMaxCapacity.current === false) {
      setManager(manager);
    }

    return () => {
      substractUser();
    };
  }, []);

  /////////////////////////////Functions only used in Unibotics///////////////////////////////////////////////////

  const addUser = () => {
    currentUsers.current += 1;
  };

  const substractUser = () => {
    currentUsers.current -= 1;
  };

  const updateBtAtMaxCapacity = (currentUserCount: number) => {
    console.log("Entering update of MaxCapacity");
    if (currentUserCount > maxUsers && isUnibotics) {
      console.log("Too much users!");
      btAtMaxCapacity.current = true;
      error_critical(
        "There's not enough room for you to enter BT-studio. Please try again later."
      );
    } else {
      console.log("The user can go in");
      btAtMaxCapacity.current = false;
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  const connectWithRetry = async (
    desiredState?: string,
    callback?: () => void
  ) => {
    if (!manager || connected.current) {
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
    if (manager) {
      console.log("The manager is up!");
      connectWithRetry();
    }
  }, [manager]);

  useUnload((event: any) => {
    event.preventDefault();
    if (manager) {
      manager.disconnect();
      connected.current = false;
    }
    // saveSettings(projectToSave.current);
    return (event.returnValue = "");
  });

  useEffect(() => {
    const func = async () => {
      if (projectId !== "") {
        getProjectConfig(projectId, settings);
      }
      await saveSettings(projectToSave.current);
      projectToSave.current = projectId;
    };
    func();
  }, [projectId]); // Reload project configuration

  useEffect(() => {
    console.log("change settings");
    settingsToSave.current = settings;
  }, [settings]); // Reload project configuration

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
    component: <VncViewer commsManager={manager} port={6080} />,
    icon: <GazeboIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: <VncViewer commsManager={manager} port={6082} />,
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
      <OtherButtons key="OtherButtons" project={projectId} />,
    ],
    name: "Tree editor",
    language: "custom_tree_editor",
    trigger: [{ group: "Trees", extension: "json" }],
  };

  return (
    <StyledAppContainer bg={theme.palette.bg}>
      <HeaderMenu
        currentProjectname={projectId}
        manager={manager}
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

export default App;
