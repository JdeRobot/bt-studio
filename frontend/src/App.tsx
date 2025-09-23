import React, { useState, useEffect, useRef } from "react";
import { useUnload } from "./hooks/useUnload";
import HeaderMenu from "./components/HeaderMenu";
import "./App.css";
import { SimulatorIcon, TerminalIcon } from "./components/icons";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectConfig, saveProjectConfig } from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import IdeInterface, { VncViewer, useError } from "jderobot-ide-interface";
import TreeEditorContainer, {
  AddSubtreeButton,
  BTSelectorButtons,
  OtherButtons,
} from "./components/TreeEditor";
import TreeMonitorContainer from "./components/TreeMonitor";
import { explorers } from "./components/Explorers";
import { statusBar } from "./components/StatusBar";
import { editorApi } from "./components/Editors";

import TerminalRoundedIcon from "@mui/icons-material/TerminalRounded";
import VideoCameraBackRoundedIcon from "@mui/icons-material/VideoCameraBackRounded";
import AccountTreeRoundedIcon from "@mui/icons-material/AccountTreeRounded";

const App = ({ isUnibotics }: { isUnibotics: boolean }) => {
  const [currentProjectname, setCurrentProjectname] = useState<string>("");
  // const [projectToSave, setProjectToSave] = useState(currentProjectname);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [showSim, setSimVisible] = useState<boolean>(false);
  const [showMonitor, setMonitorVisible] = useState<boolean>(false);
  const [showTerminal, setTerminalVisible] = useState<boolean>(false);
  const [layout, setLayout] = useState<"only-editor" | "only-viewers" | "both">(
    "both",
  );

  //Only needed in Unibotics
  const maxUsers = 15;
  const currentUsers = React.useRef<number>(0);
  const btAtMaxCapacity = React.useRef<boolean>(false);
  const projectToSave = React.useRef<string>(currentProjectname);
  const { error_critical } = useError();

  const settings = React.useContext(OptionsContext);
  const settingsToSave = React.useRef<Object>(settings);

  const saveSettings = async (project: string) => {
    let json_settings: { name: string; config: { [id: string]: any } } = {
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
      currentUsers.current,
    );
    console.log(
      "Current value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current,
    );
    updateBtAtMaxCapacity(currentUsers.current);
    console.log(
      "Updated value of UsersAtMaxCapacity: ",
      btAtMaxCapacity.current,
    );

    const manager = CommsManager.getInstance();
    if (btAtMaxCapacity.current === false) {
      setManager(manager);
    }

    return () => {
      substractUser();
    };
  }, []);

  const resetManager = () => {
    CommsManager.deleteInstance();
    const manager = CommsManager.getInstance();
    setManager(manager);
  };

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
        "There's not enough room for you to enter BT-studio. Please try again later.",
      );
    } else {
      console.log("The user can go in");
      btAtMaxCapacity.current = false;
    }
  };

  ////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  const connectWithRetry = async () => {
    if (!manager || connected.current) {
      return;
    }
    try {
      await manager.connect();
      console.log("Connected!", manager.getState());
      connected.current = true;
    } catch (error) {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 1000);
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
    saveSettings(projectToSave.current);
    return (event.returnValue = "");
  });

  useEffect(() => {
    const func = async () => {
      if (currentProjectname !== "") {
        getProjectConfig(currentProjectname, settings);
      }
      await saveSettings(projectToSave.current);
      projectToSave.current = currentProjectname;
    };
    func();
  }, [currentProjectname]); // Reload project configuration

  useEffect(() => {
    console.log("change settings");
    settingsToSave.current = settings;
  }, [settings]); // Reload project configuration

  const treeMonitor = {
    component: (
      <TreeMonitorContainer
        commsManager={manager}
        project={currentProjectname}
      />
    ),
    icon: <AccountTreeRoundedIcon />,
    name: "Tree Monitor",
    active: showMonitor,
    activate: setMonitorVisible,
  };

  const gazeboViewer = {
    component: <VncViewer commsManager={manager} port={6080} />,
    icon: <VideoCameraBackRoundedIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: <VncViewer commsManager={manager} port={1108} />,
    icon: <TerminalRoundedIcon />,
    name: "Terminal",
    active: showTerminal,
    activate: setTerminalVisible,
  };

  const treeEditor = {
    component: TreeEditorContainer,
    buttons: [
      <BTSelectorButtons project={currentProjectname} />,
      <AddSubtreeButton project={currentProjectname} />,
      <OtherButtons project={currentProjectname} />,
    ],
    name: "Tree editor",
    language: "custom_tree_editor",
    trigger: [{ group: "Trees", extension: "json" }],
  };

  return (
    <div
      className="bt-App"
      style={{ display: "flex" }}
    >
      <HeaderMenu
        currentProjectname={currentProjectname}
        setCurrentProjectname={setCurrentProjectname}
        manager={manager}
        isUnibotics={isUnibotics}
        setLayout={setLayout}
      />
      <IdeInterface
        commsManager={manager}
        resetManager={resetManager}
        project={currentProjectname}
        explorers={explorers}
        api={editorApi}
        extraEditors={[treeEditor]}
        viewers={[treeMonitor, gazeboViewer, terminalViewer]}
        options={[]}
        layout={layout}
        statusBarComponents={statusBar}
      />
    </div>
  );
};

export default App;
