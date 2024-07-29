// App.js
import React, { useMemo, useState, useEffect } from "react";
// import useLocalStorage from 'use-local-storage'
import { useUnload } from "./components/comms_manager/useUnload";
import { Resizable } from "react-resizable";
import HeaderMenu from "./components/header_menu/HeaderMenu";
import FileBrowser from "./components/file_browser/FileBrowser";
import FileEditor from "./components/file_editor/FileEditor";
import "./App.css";
import DiagramEditor from "./components/diagram_editor/DiagramEditor";
import VncViewer from "./components/vnc_viewer/VncViewer";
import CommsManager from "./components/comms_manager/CommsManager";
import ErrorModal from "./components/error_popup/ErrorModal";

import axios from "axios";

const App = () => {
  const [editorWidth, setEditorWidth] = useState(807);
  const [currentFilename, setCurrentFilename] = useState("");
  const [currentProjectname, setCurrentProjectname] = useState("");
  const [currentUniverseName, setCurrentUniverseName] = useState(null);
  const [actionNodesData, setActionNodesData] = useState({});
  const [modelJson, setModelJson] = useState("");
  const [isErrorModalOpen, setErrorModalOpen] = useState(false);
  const [projectChanges, setProjectChanges] = useState(false);
  const [gazeboEnabled, setGazeboEnabled] = useState(false);
  const [manager, setManager] = useState(null);

  // const defaultDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
  // const [theme, setTheme] = useLocalStorage('theme', defaultDark ? 'dark' : 'light');

  useEffect(() => {
    const newManager = CommsManager("ws://127.0.0.1:7163");
    setManager(newManager);
  }, []);

  useUnload(() => {
    manager.disconnect();
  });

  const connectWithRetry = () => {
    console.log("The manager is up!");
    manager
      .connect()
      .then(() => {
        console.log("Connected!");
      })
      .catch((e) => {
        // Connection failed, try again after a delay
        console.log("Connection failed, trying again!");
        setTimeout(connectWithRetry, 1000);
      });
  };

  const launchUniverse = (universe_name) => {
    const apiUrl = `/tree_api/get_universe_configuration?project_name=${encodeURIComponent(
      currentProjectname
    )}&universe_name=${encodeURIComponent(universe_name)}`;

    axios.get(apiUrl).then((response) => {
      console.log("Stored universe config: " + response.data);
      const stored_cfg = JSON.parse(response.data);

      if (stored_cfg.type === "robotics_backend") {
        const universe_config = {
          name: stored_cfg.name,
          launch_file_path: stored_cfg.config.launch_file_path,
          ros_version: "ROS2",
          visualization: "gazebo_rae",
          world: "gazebo",
          exercise_id: stored_cfg.id,
        };
        manager.launchWorld(universe_config).then(() => {
          console.log("World launched!");
          manager
            .prepareVisualization(universe_config.visualization)
            .then(() => {
              console.log("Viz ready!");
              setGazeboEnabled(true);
            });
        });
      } else {
        console.log("Launching a zip universe: " + universe_name);

        fetch("/tree_api/get_universe_zip/", {
          method: "POST",
          headers: {
            "Content-Type": "application/json",
          },
          body: JSON.stringify({
            app_name: currentProjectname,
            universe_name: universe_name,
          }),
        })
          .then((response) => {
            if (!response.ok) {
              return response.json().then((data) => {
                throw new Error(data.message || "An error occurred.");
              });
            }
            return response.blob();
          })
          .then((blob) => {
            var reader = new FileReader();
            reader.readAsDataURL(blob);
            reader.onloadend = function () {
              // Get the zip in base64
              var base64data = reader.result;

              // Prepare the config
              const universe_cfg = {
                name: stored_cfg.name,
                launch_file_path: stored_cfg.config.launch_file_path,
                ros_version: stored_cfg.config.ros_version,
                world: stored_cfg.config.world,
                zip: base64data,
              };

              // Launch the world
              manager.launchWorld(universe_cfg).then(() => {
                console.log("World launched!");
                manager.prepareVisualization("gazebo_rae").then(() => {
                  console.log("Viz ready!");
                  setGazeboEnabled(true);
                });
              });
            };
          });
      }
    });
  };

  const terminateUniverse = () => {
    if (gazeboEnabled) {
      manager.terminate_application().then(() => {
        manager.terminate_visualization().then(() => {
          manager.terminate_universe().then(() => {
            setGazeboEnabled(false);
            setCurrentUniverseName(null);
          });
        });
      });
    }
  };

  const changeUniverse = (universe_name) => {
    if (gazeboEnabled) {
      const apiUrl = `/tree_api/get_universe_configuration?project_name=${encodeURIComponent(
        currentProjectname
      )}&universe_name=${encodeURIComponent(universe_name)}`;
      axios.get(apiUrl).then((response) => {
        console.log(response.data);
        const universe_raw_config = JSON.parse(response.data);

        if (universe_raw_config.type === "robotics_backend") {
          const universe_config = {
            name: universe_raw_config.name,
            launch_file_path: universe_raw_config.config.launch_file_path,
            ros_version: "ROS2",
            visualization: "gazebo_rae",
            world: "gazebo",
            exercise_id: universe_raw_config.id,
          };
          manager.terminate_application().then(() => {
            manager.terminate_visualization().then(() => {
              manager.terminate_universe().then(() => {
                manager.launchWorld(universe_config).then(() => {
                  console.log("World launched!");
                  manager
                    .prepareVisualization(universe_config.visualization)
                    .then(() => {
                      console.log("Viz ready!");
                      setGazeboEnabled(true);
                    });
                });
              });
            });
          });
        }
      });
    }
  };

  useEffect(() => {
    if (manager) {
      connectWithRetry();
    }
  }, [manager]); // Re-run if the manager instance changes

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
    <div className="App" data-theme={"dark"}>
      <ErrorModal isOpen={isErrorModalOpen} onClose={closeError} />

      <HeaderMenu
        setCurrentProjectname={setCurrentProjectname}
        currentProjectname={currentProjectname}
        setCurrentUniverseName={setCurrentUniverseName}
        currentUniverseName={currentUniverseName}
        launchUniverse={launchUniverse}
        terminateUniverse={terminateUniverse}
        changeUniverse={changeUniverse}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
        openError={openError}
      />

      <div className="App-main" style={{ display: "flex" }}>
        <div style={{ width: "200px", paddingLeft: "1vw" }}>
          <FileBrowser
            setCurrentFilename={setCurrentFilename}
            currentFilename={currentFilename}
            currentProjectname={currentProjectname}
            setProjectChanges={setProjectChanges}
            actionNodesData={actionNodesData}
          />
        </div>

        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize("editorWidth", size)}
          minConstraints={[400, 400]}
          maxConstraints={[900, 900]}
        >
          <div style={{ width: `${editorWidth}px` }}>
            <FileEditor
              currentFilename={currentFilename}
              currentProjectname={currentProjectname}
              setProjectChanges={setProjectChanges}
            />
          </div>
        </Resizable>

        <div>
          <DiagramEditor
            currentProjectname={currentProjectname}
            setModelJson={setModelJson}
            setProjectChanges={setProjectChanges}
            gazeboEnabled={gazeboEnabled}
            manager={manager}
            actionNodesData={actionNodesData}
            openError={openError}
          />
          <VncViewer gazeboEnabled={gazeboEnabled} />
        </div>
      </div>
    </div>
  );
};

export default App;
