import React, { useState, useEffect, useRef } from "react";
import { useUnload } from "./hooks/useUnload";
import HeaderMenu from "./components/header_menu/HeaderMenu";
import "./App.css";
import ErrorModal from "./components/error_popup/ErrorModal";
import { useError } from "./components/error_popup/ErrorModal";
import { ReactComponent as SimulatorIcon } from "./components/icons/gazebo.svg";
import { ReactComponent as TerminalIcon } from "./components/icons/terminal.svg";
import { CommsManager } from "jderobot-commsmanager";
import {
  createAction,
  createFile,
  createFolder,
  deleteFile,
  deleteFolder,
  getFile,
  getFileList,
  getProjectConfig,
  getRoboticsBackendUniverse,
  getUniverseConfig,
  listUniverses,
  renameFile,
  renameFolder,
  saveFile,
  uploadFile,
} from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import IdeInterface, {
  VncViewer,
  Entry,
  newFileData,
  StatusBarComponents,
  ExtraApi,
} from "jderobot-ide-interface";
import { publish } from "./components/helper/TreeEditorHelper";
import TreeEditor from "./components/tree_editor/TreeEditorContainer";
import {
  AddSubtreeButton,
  BTSelectorButtons,
  OtherButtons,
} from "./components/tree_editor/TreeEditorMenu";
import TreeMonitor from "./components/tree_monitor/TreeMonitorContainer";
import { createCustomUniverseConfig } from "./components/helper/customUniverseHelper";
import UniverseModal from "./components/UniverseModal/UniverseModal";

const App = ({ isUnibotics }: { isUnibotics: boolean }) => {
  const [currentProjectname, setCurrentProjectname] = useState<string>("");
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
        "There's not enough room for you to enter BT-studio. Please try again later."
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

  const fileExplorer = {
    name: "Code",
    list: (project: string) => {
      return getFileList(project);
    },
    file: {
      create: (project: string, location: string, data: newFileData) => {
        if (data.fileType === "actions") {
          publish("updateActionList");
          return createAction(project, data.fileName, data.templateType);
        } else {
          return createFile(project, data.fileName, location);
        }
      },
      get: (project: string, path: string) => {
        return getFile(project, path);
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFile(project, oldPath, newPath);
      },
      delete: (project: string, path: string) => {
        return deleteFile(project, path);
      },
      upload: (
        project: string,
        path: string,
        name: string,
        content: string
      ) => {
        return uploadFile(project, name, path, content);
      },
    },
    folder: {
      create: (project: string, location: string, name: string) => {
        return createFolder(project, name, location);
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFolder(project, oldPath, newPath);
      },
      delete: (project: string, path: string) => {
        return deleteFolder(project, path);
      },
    },
  };

  const universeExplorer = {
    name: "Universes",
    list: (project: string) => {
      return getFileList(project, "");
    },
    file: {
      create: (project: string, location: string, data: newFileData) => {
        return createFile(project, data.fileName, location, "");
      },
      get: (project: string, path: string) => {
        return getFile(project, path, "");
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFile(project, oldPath, newPath, "");
      },
      delete: (project: string, path: string) => {
        return deleteFile(project, path, "");
      },
      upload: (
        project: string,
        path: string,
        name: string,
        content: string
      ) => {
        return uploadFile(project, name, path, content, "");
      },
    },
    folder: {
      create: (project: string, location: string, name: string) => {
        return createFolder(project, name, location, "");
      },
      rename: (project: string, oldPath: string, newPath: string) => {
        return renameFolder(project, oldPath, newPath, "");
      },
      delete: (project: string, path: string) => {
        return deleteFolder(project, path, "");
      },
    },
  };

  const treeMonitor = {
    component: (
      <TreeMonitor commsManager={manager} project={currentProjectname} />
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
    component: TreeEditor,
    buttons: [
      <BTSelectorButtons project={currentProjectname} />,
      <AddSubtreeButton project={currentProjectname} />,
      <OtherButtons project={currentProjectname} />,
    ],
    name: "Tree editor",
    language: "custom_tree_editor",
    trigger: [{ group: "Trees", extension: "json" }],
  };

  const api: ExtraApi = {
    file: {
      get: (project: string, file: Entry) => {
        if (file.group === "Universes") {
          return getFile(project, file.path, "");
        }
        return getFile(project, file.path);
      },
      save: (project: string, file: Entry, content: string) => {
        if (file.group === "Universes") {
          return saveFile(project, file.path, content, "");
        }
        return saveFile(project, file.path, content);
      },
    },
    universes: {
      list: (project: string) => {
        return listUniverses(project);
      },
      get_config: async (project: string, universe: string) => {
        const config = await getUniverseConfig(universe, project);
        const configJson = JSON.parse(config);

        if (configJson.type === "robotics_backend") {
          return getRoboticsBackendUniverse(configJson.id);
        }
        // Get custom universe config
        return createCustomUniverseConfig(project, configJson);
      },
    },
  };

  const statusBar: StatusBarComponents = {
    universeSelector: UniverseModal,
    extras: [],
  };

  return (
    <div
      className="bt-App"
      data-theme={settings.theme.value}
      style={{ display: "flex" }}
    >
      <ErrorModal />
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
        explorers={[fileExplorer, universeExplorer]}
        api={api}
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
