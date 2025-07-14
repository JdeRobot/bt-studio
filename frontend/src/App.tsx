import React, { useState, useEffect, useRef } from "react";
import { useUnload } from "./hooks/useUnload";
import HeaderMenu from "./components/HeaderMenu";
import "./App.css";
import { SimulatorIcon, TerminalIcon } from "./components/icons";
import { CommsManager } from "jderobot-commsmanager";
import { getProjectConfig } from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import IdeInterface, {
  Theme,
  ThemeProvider,
  VncViewer,
  useError,
} from "jderobot-ide-interface";
import TreeEditorContainer, {
  AddSubtreeButton,
  BTSelectorButtons,
  OtherButtons,
} from "./components/TreeEditor";
import TreeMonitorContainer from "./components/TreeMonitor";
import { explorers } from "./components/Explorers";
import { statusBar } from "./components/StatusBar";
import { editorApi } from "./components/Editors";

const App = ({ isUnibotics }: { isUnibotics: boolean }) => {
  const [currentProjectname, setCurrentProjectname] = useState<string>("");
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
  const { error_critical } = useError();

  const settings = React.useContext(OptionsContext);
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

  useUnload(() => {
    if (manager) {
      manager.disconnect();
      connected.current = false;
    }
  });

  useEffect(() => {
    if (currentProjectname !== "") {
      getProjectConfig(currentProjectname, settings);
    }
  }, [currentProjectname]); // Reload project configuration

  const treeMonitor = {
    component: (
      <TreeMonitorContainer
        commsManager={manager}
        project={currentProjectname}
      />
    ),
    icon: <SimulatorIcon />,
    name: "Tree Monitor",
    active: showMonitor,
    activate: setMonitorVisible,
  };

  const gazeboViewer = {
    component: <VncViewer commsManager={manager} port={6080} />,
    icon: <SimulatorIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: <VncViewer commsManager={manager} port={1108} />,
    icon: <TerminalIcon />,
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

  const darkTheme: Theme = {
    palette: {
      text: "#ededf2",
      darkText: "#000000",
      placeholderText: "#a6a6bf",
      success: "#29ac29",
      warning: "#f9e86d",
      error: "#802626",
      background: "#16161d",
      primary: "#444444ff",
      secondary: "#666666ff",
      scrollbar: "#6f6f90",
      border: {
        warning: "#ffe100",
        error: "#772222",
        info: "#134f53",
      },
      progressBar: {
        background: "#134f53",
        color: "#1d777c",
      },
      button: {
        error: "#9e2e2e",
        success: "#29ac29",
        warning: "#ffe100",
        info: "#134f53",
        hoverError: "#c63939",
        hoverSuccess: "#29ac29",
        hoverWarning: "#ccb400",
        hoverInfo: "#1d777c",
      },
      selectedGradient:
        "linear-gradient( -45deg, #12494c 0%, #584f42 50%, #909c7b 100%)",
    },
    roundness: 5,
    monacoTheme: "dark",
  };

  return (
    <div
      className="bt-App"
      data-theme={settings.theme.value}
      style={{ display: "flex" }}
    >
      <HeaderMenu
        currentProjectname={currentProjectname}
        setCurrentProjectname={setCurrentProjectname}
        manager={manager}
        isUnibotics={isUnibotics}
        setLayout={setLayout}
      />
      <ThemeProvider>
        {/* <ThemeProvider theme={darkTheme}> */}
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
      </ThemeProvider>
    </div>
  );
};

export default App;
