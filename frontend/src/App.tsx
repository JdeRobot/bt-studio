// App.js
import React, { useMemo, useState, useEffect } from "react";
// import useLocalStorage from 'use-local-storage'
import { useUnload } from "./hooks/useUnload";
import { Resizable } from "react-resizable";
import HeaderMenu from "./components/header_menu/HeaderMenu";
import FileBrowser from "./components/file_browser/FileBrowser";
import FileEditor from "./components/file_editor/FileEditor";
import "./App.css";
import VncViewer from "./components/vnc_viewer/VncViewer";
import ErrorModal from "./components/error_popup/ErrorModal";
import MainTreeEditorContainer from "./components/tree_editor/MainTreeEditorContainer";
import CommsManager from "./api_helper/CommsManager";
import { loadProjectConfig } from "./api_helper/TreeWrapper";

import { OptionsContext } from "./components/options/Options";
import TerminalViewer from "./components/vnc_viewer/TerminalViewer";
import StatusBar from "./components/status_bar/StatusBar";

const App = () => {
  const [fileBrowserWidth, setFileBrowserWidth] = useState<number>(300);
  const [editorWidth, setEditorWidth] = useState<number>(800);
  const [currentFilename, setCurrentFilename] = useState<string>("");
  const [currentProjectname, setCurrentProjectname] = useState<string>("");
  const [currentUniverseName, setCurrentUniverseName] = useState<string>("");
  const [actionNodesData, setActionNodesData] = useState<Record<string, any>>(
    {},
  );
  const [modelJson, setModelJson] = useState<string>("");
  const [isErrorModalOpen, setErrorModalOpen] = useState<boolean>(false);
  const [projectChanges, setProjectChanges] = useState<boolean>(false);
  const [gazeboEnabled, setGazeboEnabled] = useState<boolean>(false);
  const [manager, setManager] = useState<any>(null);
  const [diagramEditorReady, setDiagramEditorReady] = useState<boolean>(false);
  const [showSim, setSimVisible] = useState<boolean>(false);
  const [showTerminal, setTerminalVisible] = useState<boolean>(false);

  const settings = React.useContext(OptionsContext);
  //////////////////////////s////////////////////////////

  // RB manager setup

  useEffect(() => {
    const manager = CommsManager.getInstance();
    setManager(manager);
  }, []);

  const connectWithRetry = async () => {
    try {
      await manager.connect();
      console.log("Connected!");
    } catch (error) {
      // Connection failed, try again after a delay
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
    manager.disconnect();
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
  const openError = (err: unknown) => {
    if (err instanceof Error) {
      (document.getElementById("errorMsg") as HTMLElement).innerText =
        err.message;
      setErrorModalOpen(true);
    }
  };

  const closeError = () => {
    setErrorModalOpen(false);
  };

  // Show VNC viewers
  const showVNCViewer = () => {
    setSimVisible(true);
    setTerminalVisible(true);
  }

  const showVNCTerminal = (show: boolean) => {
    if (gazeboEnabled) {
      setTerminalVisible(show)
    }
  }

  const showVNCSim = (show: boolean) => {
    if (gazeboEnabled) {
      setSimVisible(show)
    }
  }

  return (
    <div
      className="App"
      data-theme={settings.theme.value}
      style={{ display: "flex" }}
    >
      {/* <ErrorModal isOpen={isErrorModalOpen} onClose={closeError} /> */}

      <HeaderMenu
        currentProjectname={currentProjectname}
        setCurrentProjectname={setCurrentProjectname}
        currentUniverseName={currentUniverseName}
        setCurrentUniverseName={setCurrentUniverseName}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
        gazeboEnabled={gazeboEnabled}
        setGazeboEnabled={setGazeboEnabled}
        // onSetShowExecStatus={onSetShowExecStatus}
        manager={manager}
        showVNCViewer={showVNCViewer}
      />

      <div className="App-main">
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
            <div className="sideBar">
              <FileBrowser
                setCurrentFilename={setCurrentFilename}
                currentFilename={currentFilename}
                currentProjectname={currentProjectname}
                setProjectChanges={setProjectChanges}
                actionNodesData={actionNodesData}
                showAccentColor={"editorShowAccentColors"}
                diagramEditorReady={diagramEditorReady}
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
            }}
          >
            <FileEditor
              currentFilename={currentFilename}
              currentProjectname={currentProjectname}
              setProjectChanges={setProjectChanges}
            />
            {showTerminal && (
              <TerminalViewer gazeboEnabled={gazeboEnabled} />
            )}
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
              setGlobalJson={setModelJson}
              modelJson={modelJson}
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
      />
    </div>
  );
};

export default App;
