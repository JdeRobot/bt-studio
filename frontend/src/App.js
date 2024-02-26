// App.js
import React, { useMemo, useState, useEffect } from 'react';
import { useUnload } from './components/comms_manager/useUnload';
import { Resizable } from 'react-resizable';
import HeaderMenu from './components/header_menu/HeaderMenu';
import FileBrowser from './components/file_browser/FileBrowser';
import FileEditor from './components/file_editor/FileEditor';
import './App.css';
import DiagramEditor from './components/diagram_editor/DiagramEditor';
import VncViewer from './components/vnc_viewer/VncViewer'
import CommsManager from './components/comms_manager/CommsManager';

const App = () => {

  const [editorWidth, setEditorWidth] = useState(700);
  const [currentFilename, setCurrentFilename] = useState('');
  const [currentProjectname, setCurrentProjectname] = useState('');
  const [modelJson, setModelJson] = useState('');
  const [projectChanges, setProjectChanges] = useState(false);
  const [gazeboEnabled, setGazeboEnabled] = useState(false);
  const [manager, setManager] = useState(null);

  var universe_config = {
    "name": "follow_person_ros2",
    "launch_file_path": "/opt/jderobot/Launchers/follow_person.launch.py",
    "ros_version": "ROS2",
    "visualization": "gazebo_rae",
    "world": "gazebo",
    "template": "RoboticsAcademy/exercises/static/exercises/follow_person_newmanager/python_template/",
    "exercise_id": "follow_person_newmanager"
  }

  useEffect(() => {
    const newManager = CommsManager("ws://127.0.0.1:7163");
    setManager(newManager);
  }, []);

  useUnload(() => {
    manager.disconnect();
  });

  const connectWithRetry = () => {
    console.log("The manager is up!");
    manager.connect()
    .then(() => {
      console.log("Connected!");
      manager.launchWorld(universe_config)
      .then(() => {
        console.log("World launched!")
        manager.prepareVisualization(universe_config.visualization)
        .then(() => {
          console.log("Viz ready!")
          setGazeboEnabled(true);
        })
      })
    })
    .catch((e) => {
      // Connection failed, try again after a delay
      console.log("Connection failed, trying again!")
      setTimeout(connectWithRetry, 1000);
    });
  }

  useEffect(() => {
    if (manager) {
      connectWithRetry();
    }
  }, [manager]); // Re-run if the manager instance changes

  const onResize = (key, size) => {
    switch (key) {
      case 'editorWidth':
        setEditorWidth(size.width);
        break;
      default:
        break;
    }
  };

  return (
    <div className="App">

      <HeaderMenu 
        setCurrentProjectname={setCurrentProjectname} 
        currentProjectname={currentProjectname}
        modelJson={modelJson}
        projectChanges={projectChanges}
        setProjectChanges={setProjectChanges}
      />

      <div className="App-main" style={{ display: 'flex' }}>

        <div style={{ width: '200px'}}>
          <FileBrowser 
            setCurrentFilename={setCurrentFilename} 
            currentFilename={currentFilename}
            currentProjectname={currentProjectname}
            setProjectChanges={setProjectChanges}
          />
        </div>
        
        <Resizable
          width={editorWidth}
          height={0}
          onResize={(e, { size }) => onResize('editorWidth', size)}
          minConstraints={[400, 400]}
          maxConstraints={[900, 900]}
        >
          <div style={{ width: `${editorWidth}px` }}>
            <FileEditor 
              currentFilename = {currentFilename} 
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
          />
          <VncViewer
            gazeboEnabled={gazeboEnabled}
          />
        </div>
        
      </div>

    </div>
  );
};

export default App;
