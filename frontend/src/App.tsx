import React, { useState, useEffect, useRef } from "react";
// import useLocalStorage from 'use-local-storage'
import { useUnload } from "./hooks/useUnload";
import { Resizable } from "react-resizable";
import HeaderMenu from "./components/header_menu/HeaderMenu";
import FileBrowser from "./components/file_browser/FileBrowser";
import FileEditor from "./components/file_editor/FileEditor";
import "./App.css";
import VncViewer from "./components/vnc_viewer/VncViewer";
import ErrorModal, { ErrorProvider } from "./components/error_popup/ErrorModal";
import { useError } from "./components/error_popup/ErrorModal";
import MainTreeEditorContainer from "./components/tree_editor/MainTreeEditorContainer";
import { ReactComponent as SimulatorIcon } from "./components/status_bar/img/gazebo.svg";
import { ReactComponent as TerminalIcon } from "./components/status_bar/img/terminal.svg";
import CommsManager from "./api_helper/CommsManager";
import {
  createAction,
  createFile,
  createFolder,
  deleteFile,
  deleteFolder,
  getFile,
  getFileList,
  getProjectConfig,
  renameFile,
  renameFolder,
  saveFile,
} from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import TerminalViewer from "./components/vnc_viewer/TerminalViewer";
import StatusBar from "./components/status_bar/StatusBar";
import UniverseBrowser from "./components/file_browser/UniverseBrowser";
import EditorComponent from "./components/editor_component/EditorComponent";
import { newFileModalData } from "./components/editor_component/explorer/modals/NewFileModal";
import { publish } from "./components/helper/TreeEditorHelper";
import { Entry } from "./components/editor_component/explorer/Explorer";

const App = ({ isUnibotics }: { isUnibotics: boolean }) => {
  const [fileBrowserWidth, setFileBrowserWidth] = useState<number>(300);
  const [editorWidth, setEditorWidth] = useState<number>(800);
  const [currentFilename, setCurrentFilename] = useState<string>("");
  const [autosaveEnabled, setAutosave] = useState<boolean>(true);
  const [forceSaveCurrent, setForcedSaveCurrent] = useState<boolean>(false);
  const [currentProjectname, setCurrentProjectname] = useState<string>("");
  const [currentUniverseName, setCurrentUniverseName] = useState<string>("");
  const [saveCurrentDiagram, setSaveCurrentDiagram] = useState<boolean>(false);
  const [updateFileExplorer, setUpdateFileExplorer] = useState<boolean>(false);
  const [projectChanges, setProjectChanges] = useState<boolean>(false);
  const [gazeboEnabled, setGazeboEnabled] = useState<boolean>(false);
  const [manager, setManager] = useState<CommsManager | null>(null);
  const [showSim, setSimVisible] = useState<boolean>(false);
  const [showTerminal, setTerminalVisible] = useState<boolean>(false);

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
    if (btAtMaxCapacity.current == false) {
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
      console.log("Connected!");
      connected.current = true;
    } catch (error) {
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 1000);
    }
  };

  useEffect(() => {
    if (manager) {
      console.log("The manager is up!");
      manager.subscribeOnce("introspection", () => {});
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

  const onResize = (key: string, size: { width: number; height: number }) => {
    switch (key) {
      case "editorWidth":
        setEditorWidth(size.width);
        break;
      case "fileBrowserWidth":
        setFileBrowserWidth(size.width);
        break;
      default:
        break;
    }
  };

  // Show VNC viewers
  const showVNCViewer = () => {
    setSimVisible(true);
    setTerminalVisible(true);
  };

  const showVNCTerminal = (show: boolean) => {
    if (gazeboEnabled) {
      setTerminalVisible(show);
    }
  };

  const showVNCSim = (show: boolean) => {
    if (gazeboEnabled) {
      setSimVisible(show);
    }
  };

  const fileExplorer = {
    name: "Code",
    list: (project: string) => {
      return getFileList(project);
    },
    file: {
      create: (project: string, location: string, data: newFileModalData) => {
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
      create: (project: string, location: string, data: newFileModalData) => {
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

  const editorApi = {
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
  };

  const gazeboViewer = {
    component: <VncViewer gazeboEnabled={false} />,
    icon: <SimulatorIcon />,
    name: "Gazebo",
    active: showSim,
    activate: setSimVisible,
  };

  const terminalViewer = {
    component: <TerminalViewer gazeboEnabled={false} />,
    icon: <TerminalIcon />,
    name: "Terminal",
    active: showTerminal,
    activate: setTerminalVisible,
  };

  return (
    <div
      className="bt-App"
      data-theme={settings.theme.value}
      style={{ display: "flex" }}
    >
      <ErrorModal/>
      <HeaderMenu
        currentProjectname={currentProjectname}
        setCurrentProjectname={setCurrentProjectname}
        currentUniverseName={currentUniverseName}
        setCurrentUniverseName={setCurrentUniverseName}
        setSaveCurrentDiagram={setSaveCurrentDiagram}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
        gazeboEnabled={gazeboEnabled}
        setGazeboEnabled={setGazeboEnabled}
        manager={manager}
        showVNCViewer={showVNCViewer}
        isUnibotics={isUnibotics}
      />

      <EditorComponent
        commsManager={manager}
        project={currentProjectname}
        explorers={[fileExplorer, universeExplorer]}
        editorApi={editorApi}
        extra_editors={[]}
        viewers={[gazeboViewer, terminalViewer]}
        options={[]}
        layout="both"
      />
    </div>
  );

  // return (
  //   <div
  //     className="bt-App"
  //     data-theme={settings.theme.value}
  //     style={{ display: "flex" }}
  //   >
  //     <ErrorModal />
  //     <>
  //       <HeaderMenu
  //         currentProjectname={currentProjectname}
  //         setCurrentProjectname={setCurrentProjectname}
  //         currentUniverseName={currentUniverseName}
  //         setCurrentUniverseName={setCurrentUniverseName}
  //         setSaveCurrentDiagram={setSaveCurrentDiagram}
  //         projectChanges={projectChanges}
  //         setProjectChanges={setProjectChanges}
  //         gazeboEnabled={gazeboEnabled}
  //         setGazeboEnabled={setGazeboEnabled}
  //         manager={manager}
  //         showVNCViewer={showVNCViewer}
  //         isUnibotics={isUnibotics}
  //       />

  //       <div className="bt-App-main">
  //         <Resizable
  //           width={fileBrowserWidth}
  //           height={0}
  //           onResize={(e, { size }) => onResize("fileBrowserWidth", size)}
  //           minConstraints={[200, 200]}
  //           maxConstraints={[400, 400]}
  //         >
  //           <div
  //             style={{
  //               width: `${fileBrowserWidth}px`,
  //               display: "flex",
  //               flexDirection: "column",
  //             }}
  //           >
  //             <div className="bt-sidebar">
  //               <FileBrowser
  //                 setCurrentFilename={setCurrentFilename}
  //                 currentFilename={currentFilename}
  //                 currentProjectname={currentProjectname}
  //                 setProjectChanges={setProjectChanges}
  //                 setAutosave={setAutosave}
  //                 forceSaveCurrent={forceSaveCurrent}
  //                 setForcedSaveCurrent={setForcedSaveCurrent}
  //                 forceUpdate={{
  //                   value: updateFileExplorer,
  //                   callback: setUpdateFileExplorer,
  //                 }}
  //                 setSaveCurrentDiagram={setSaveCurrentDiagram}
  //               />
  //               <UniverseBrowser
  //                 setCurrentFilename={setCurrentFilename}
  //                 currentFilename={currentFilename}
  //                 currentProjectname={currentProjectname}
  //                 setProjectChanges={setProjectChanges}
  //                 setAutosave={setAutosave}
  //                 forceSaveCurrent={forceSaveCurrent}
  //                 setForcedSaveCurrent={setForcedSaveCurrent}
  //                 forceUpdate={{
  //                   value: updateFileExplorer,
  //                   callback: setUpdateFileExplorer,
  //                 }}
  //                 setSaveCurrentDiagram={setSaveCurrentDiagram}
  //               />
  //             </div>
  //           </div>
  //         </Resizable>

  //         <Resizable
  //           width={editorWidth}
  //           height={0}
  //           onResize={(e, { size }) => onResize("editorWidth", size)}
  //           minConstraints={[400, 400]}
  //           maxConstraints={[800, 900]}
  //         >
  //           <div
  //             style={{
  //               width: `${editorWidth}px`,
  //               display: "flex",
  //               flexDirection: "column",
  //               gap: "5px",
  //               backgroundColor: "var(--control-bar)",
  //             }}
  //           >
  //             <FileEditor
  //               currentFilename={currentFilename}
  //               currentProjectname={currentProjectname}
  //               setProjectChanges={setProjectChanges}
  //               isUnibotics={isUnibotics}
  //               autosaveEnabled={autosaveEnabled}
  //               setAutosave={setAutosave}
  //               forceSaveCurrent={forceSaveCurrent}
  //               manager={manager}
  //             />
  //             {showTerminal && <TerminalViewer gazeboEnabled={gazeboEnabled} />}
  //           </div>
  //         </Resizable>

  //         <div
  //           style={{
  //             flex: "1 1 0%",
  //             display: "flex",
  //             flexDirection: "column",
  //             flexWrap: "nowrap",
  //             gap: "5px",
  //             backgroundColor: "var(--control-bar)",
  //           }}
  //         >
  //           {currentProjectname ? (
  //             <MainTreeEditorContainer
  //               projectName={currentProjectname}
  //               setProjectEdited={setProjectChanges}
  //               saveCurrentDiagram={saveCurrentDiagram}
  //               setSaveCurrentDiagram={setSaveCurrentDiagram}
  //               updateFileExplorer={setUpdateFileExplorer}
  //             />
  //           ) : (
  //             <p>Loading...</p>
  //           )}
  //           {showSim && <VncViewer gazeboEnabled={gazeboEnabled} />}
  //         </div>
  //       </div>
  //       <StatusBar
  //         showSim={showSim}
  //         setSimVisible={showVNCSim}
  //         showTerminal={showTerminal}
  //         setTerminalVisible={showVNCTerminal}
  //         dockerData={dockerData}
  //         resetManager={resetManager}
  //       />
  //     </>
  //   </div>
  // );
};

export default App;
