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
import MainTreeEditorContainer from "./components/tree_editor/MainTreeEditorContainer";
import CommsManager from "./api_helper/CommsManager";
import { loadProjectConfig } from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import TerminalViewer from "./components/vnc_viewer/TerminalViewer";
import StatusBar from "./components/status_bar/StatusBar";

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
  var currentUsers = 0;
  var maxUsers = 1;

  const [dockerData, setDockerData] = useState<{
    gpu_avaliable: string;
    robotics_backend_version: string;
    ros_version: string;
  } | null>(null);

  const settings = React.useContext(OptionsContext);
  //////////////////////////s////////////////////////////

  // RB manager setup
  const connected = useRef<boolean>(false);

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
  }, []);

  const resetManager = () => {
    CommsManager.deleteInstance();
    const manager = CommsManager.getInstance();
    setManager(manager);
  };

  const introspectionCallback = (msg: any) => {
    setDockerData(msg.data);
  };

  const connectWithRetry = async () => {
    if (!manager || connected.current) {
      return;
    }

      try {
      console.log("Current number of users connected: " + currentUsers);
      currentUsers += 1;
      console.log("Is currents users equal or less to maximum?" + (currentUsers <= maxUsers))
        if (currentUsers <= maxUsers) {
            await manager.connect();
            console.log("Connected!");
            connected.current = true;
        }
        else {
          console.log("Exceeded users capacity.")
        }
    } catch (error) {
      // Connection failed, try again after a delay
      console.log("Connection failed, trying again!");
      setTimeout(connectWithRetry, 1000);
    }
  };

  useEffect(() => {
    if (manager) {
      console.log("The manager is up!");
      manager.subscribeOnce("introspection", introspectionCallback);
      connectWithRetry();
    }
  }, [manager]);

  useUnload(() => {
      if (manager) {
      currentUsers -= 1;
      manager.disconnect();
      connected.current = false;
    }
  });

  useEffect(() => {
    if (currentProjectname !== "") {
      loadProjectConfig(currentProjectname, settings);
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

  // Error modal
  // const openError = (err: unknown) => {
  //   if (err instanceof Error) {
  //     (document.getElementById("errorMsg") as HTMLElement).innerText =
  //       err.message;
  //     setErrorModalOpen(true);
  //   }
  // };

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

  return (
    <div
      className="bt-App"
      data-theme={settings.theme.value}
      style={{ display: "flex" }}
    >
      <ErrorProvider>
        <ErrorModal />
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

        <div className="bt-App-main">
          <Resizable
            width={fileBrowserWidth}
            height={0}
            onResize={(e, { size }) => onResize("fileBrowserWidth", size)}
            minConstraints={[200, 200]}
            maxConstraints={[400, 400]}
          >
            <div
              style={{
                width: `${fileBrowserWidth}px`,
                display: "flex",
                flexDirection: "column",
              }}
            >
              <div className="bt-sideBar">
                <FileBrowser
                  setCurrentFilename={setCurrentFilename}
                  currentFilename={currentFilename}
                  currentProjectname={currentProjectname}
                  setProjectChanges={setProjectChanges}
                  setAutosave={setAutosave}
                  forceSaveCurrent={forceSaveCurrent}
                  setForcedSaveCurrent={setForcedSaveCurrent}
                  forceUpdate={{
                    value: updateFileExplorer,
                    callback: setUpdateFileExplorer,
                  }}
                  setSaveCurrentDiagram={setSaveCurrentDiagram}
                />
              </div>
            </div>
          </Resizable>

          <Resizable
            width={editorWidth}
            height={0}
            onResize={(e, { size }) => onResize("editorWidth", size)}
            minConstraints={[400, 400]}
            maxConstraints={[800, 900]}
          >
            <div
              style={{
                width: `${editorWidth}px`,
                display: "flex",
                flexDirection: "column",
                gap: "5px",
                backgroundColor: "var(--control-bar)",
              }}
            >
              <FileEditor
                currentFilename={currentFilename}
                currentProjectname={currentProjectname}
                setProjectChanges={setProjectChanges}
                isUnibotics={isUnibotics}
                autosaveEnabled={autosaveEnabled}
                setAutosave={setAutosave}
                forceSaveCurrent={forceSaveCurrent}
              />
              {showTerminal && <TerminalViewer gazeboEnabled={gazeboEnabled} />}
            </div>
          </Resizable>

          <div
            style={{
              flex: "1 1 0%",
              display: "flex",
              flexDirection: "column",
              flexWrap: "nowrap",
              gap: "5px",
              backgroundColor: "var(--control-bar)",
            }}
          >
            {currentProjectname ? (
              <MainTreeEditorContainer
                projectName={currentProjectname}
                setProjectEdited={setProjectChanges}
                saveCurrentDiagram={saveCurrentDiagram}
                setSaveCurrentDiagram={setSaveCurrentDiagram}
                updateFileExplorer={setUpdateFileExplorer}
              />
            ) : (
              <p>Loading...</p>
            )}
            {showSim && <VncViewer gazeboEnabled={gazeboEnabled} />}
          </div>
        </div>
        <StatusBar
          showSim={showSim}
          setSimVisible={showVNCSim}
          showTerminal={showTerminal}
          setTerminalVisible={showVNCTerminal}
          dockerData={dockerData}
          resetManager={resetManager}
        />
      </ErrorProvider>
    </div>
  );
};

export default App;
