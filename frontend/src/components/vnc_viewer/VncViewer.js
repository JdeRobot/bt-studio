import React, { useEffect, useState } from 'react';
import { useUnload } from './useUnload';
import 'ace-builds/src-noconflict/mode-python';
import 'ace-builds/src-noconflict/theme-monokai';

import './VncViewer.css';
import CommsManager from './comms_manager'; // Adjust this import to where your CommsManager is defined
import BounceLoader from "react-spinners/BounceLoader";

const VncViewer = () => {

  const [enableGazebo, setEnableGazebo] = useState(true);
  const [manager, setManager] = useState(null);

  var patata = {
    "name": "follow_person_ros2",
    "launch_file_path": "/opt/jderobot/Launchers/follow_person.launch.py",
    "ros_version": "ROS2",
    "visualization": "gazebo_rae",
    "world": "gazebo",
    "template": "RoboticsAcademy/exercises/static/exercises/follow_person_newmanager/python_template/",
    "exercise_id": "follow_person_newmanager"
  }

  const connectWithRetry = () => {
    console.log("The manager is up!");
    manager.connect()
    .then(() => {
      console.log("Connected!");
      manager.launchWorld(patata)
      .then(() => {
        console.log("World launched!")
        manager.prepareVisualization(patata.visualization)
        .then(() => {
          console.log("Viz ready!")
          setEnableGazebo(false);
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
  }, [manager]); // Re-run this effect if the manager instance changes

  useEffect(() => {
    const newManager = CommsManager("ws://127.0.0.1:7163");
    setManager(newManager);
  }, [])

  useUnload(() => {
    manager.disconnect();
  });

  return (
    <div className="viewer">
      {enableGazebo ? (
        <div className='loader'>
          <BounceLoader color="#36d7b7" size={80} speedMultiplier={0.7}/>
        </div>
      ) :
      (
        <iframe
          id={"iframe"}
          style={{
            width: "100%",
            height: "100%",
          }}
          src={
            "http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"
          }
        />
      )}
    </div>
  );
};

export default VncViewer;
