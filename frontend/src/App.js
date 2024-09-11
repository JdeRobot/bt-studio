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
import axios from "axios";
import EditorContainer from "./components/diagram_editor/EditorContainer";
import CommsManager from "./api_helper/CommsManager";

const App = () => {
  const [editorWidth, setEditorWidth] = useState(600);
  const [currentFilename, setCurrentFilename] = useState("");
  const [currentProjectname, setCurrentProjectname] = useState("");
  const [currentUniverseName, setCurrentUniverseName] = useState(null);
  const [actionNodesData, setActionNodesData] = useState({});
  const [modelJson, setModelJson] = useState("");
  const [isErrorModalOpen, setErrorModalOpen] = useState(false);
  const [projectChanges, setProjectChanges] = useState(false);
  const [gazeboEnabled, setGazeboEnabled] = useState(false);
  const [manager, setManager] = useState(null);
  const [diagramEditorReady, setDiagramEditorReady] = useState(false);
  const [appRunning, setAppRunning] = useState(false);

  // const defaultDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  // const [theme, setTheme] = useLocalStorage('theme', defaultDark ? 'dark' : 'light');
  ////////////////////// SETTINGS //////////////////////

  // TODO: try to not repeat the default values
  const [editorShowAccentColors, setEditorShowAccentColors] = useState(true);
  const [theme, setTheme] = useState("dark");
  const [btOrder, setBtOrder] = useState("bottom-to-top");

  // Setting => name: {setter: function, value: name, default_value: default_value}
  const settings = {
    editorShowAccentColors: {
      setter: setEditorShowAccentColors,
      value: editorShowAccentColors,
      default_value: true,
    },
    theme: { setter: setTheme, value: theme, default_value: "dark" },
    btOrder: {
      setter: setBtOrder,
      value: btOrder,
      default_value: "bottom-to-top",
    },
  };
  //////////////////////////////////////////////////////

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

  const loadProjectConfig = async () => {
    try {
      const apiUrl = `/tree_api/get_project_configuration?project_name=${currentProjectname}`;
      const response = await axios.get(apiUrl);
      let raw_config = JSON.parse(response.data);
      let project_settings = raw_config.config;

      // Load all the settings
      Object.entries(settings).map(([key, value]) => {
        value.setter(
          project_settings[key] ? project_settings[key] : value.default_value
        );
      });
    } catch (error) {
      console.log(error);
      if (error.response) {
        if (error.response.status === 404) {
          openError(
            `The project ${currentProjectname} has no configuration available`
          );
        } else {
          openError("Failed to load configuration");
        }
      }

      console.log("Loading default settings");
      Object.entries(settings).map(([key, value]) => {
        value.setter(value.default_value);
      });
    }
  };

  useEffect(() => {
    if (currentProjectname !== "") {
      loadProjectConfig();
    }
  }, [currentProjectname]); // Reload project configuration

  const onResize = (key, size) => {
    switch (key) {
      case "editorWidth":
        setEditorWidth(size.width);
        break;
      default:
        break;
    }
  };

  const openError = (err) => {
    document.getElementById("errorMsg").innerText = err;
    setErrorModalOpen(true);
  };

  const closeError = () => {
    setErrorModalOpen(false);
  };

  return (
    <div className="App" data-theme={theme}>
      <ErrorModal isOpen={isErrorModalOpen} onClose={closeError} />

      <HeaderMenu
        currentProjectname={currentProjectname}
        setCurrentProjectname={setCurrentProjectname}
        currentUniverseName={currentUniverseName}
        setCurrentUniverseName={setCurrentUniverseName}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
        openError={openError}
        settingsProps={settings}
        gazeboEnabled={gazeboEnabled}
        setGazeboEnabled={setGazeboEnabled}
        manager={manager}
      />

      <div className="App-main" style={{ display: "flex" }}>
        <div style={{ width: "200px", paddingLeft: "1vw" }}>
          <FileBrowser
            setCurrentFilename={setCurrentFilename}
            currentFilename={currentFilename}
            currentProjectname={currentProjectname}
            setProjectChanges={setProjectChanges}
            actionNodesData={actionNodesData}
            showAccentColor={editorShowAccentColors}
            diagramEditorReady={diagramEditorReady}
          />
        </div>

        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize("editorWidth", size)}
          minConstraints={[400, 400]}
          maxConstraints={[800, 900]}
        >
          <div style={{ width: `${editorWidth}px` }}>
            <FileEditor
              currentFilename={currentFilename}
              currentProjectname={currentProjectname}
              setProjectChanges={setProjectChanges}
            />
          </div>
        </Resizable>

        <Resizable
          width={100 % -editorWidth}
          height={0}
          onResize={(e, { size }) => onResize("sidebarWidth", size)}
          minConstraints={[300, 300]}
          maxConstraints={[300, 900]}
        >
          <div style={{ flex: 1 }}>
            {currentProjectname ? (
              <EditorContainer
                projectName={currentProjectname}
                setProjectEdited={setProjectChanges}
                setGlobalJson={setModelJson}
              />
            ) : (
              <p>Loading...</p>
            )}
            <VncViewer gazeboEnabled={gazeboEnabled} />
          </div>
        </Resizable>
      </div>
    </div>
  );
};

export default App;
