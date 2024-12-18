import { useContext, useEffect } from "react";
import { OptionsContext } from "../options/Options";

import "./StatusBar.css";

import { ReactComponent as TerminalIcon } from "./img/terminal.svg";
import { ReactComponent as SimulatorIcon } from "./img/gazebo.svg";
import { ReactComponent as ResetIcon } from "./img/reset.svg";

const StatusBar = ({
  showSim,
  setSimVisible,
  showTerminal,
  setTerminalVisible,
  resetManager,
  dockerData,
}: {
  showSim: boolean;
  setSimVisible: Function;
  showTerminal: boolean;
  setTerminalVisible: Function;
  resetManager: Function;
  dockerData: {
    gpu_avaliable: string;
    robotics_backend_version: string;
    ros_version: string;
  } | null;
}) => {
  // Settings
  const settings = useContext(OptionsContext);

  return (
    <div className="bt-status-bar-container">
      <button
        className={
          showTerminal ? `bt-status-bar-button-active` : `bt-status-bar-button`
        }
        onClick={() => {
          setTerminalVisible(!showTerminal);
        }}
        title="Toggle console"
      >
        <TerminalIcon className="bt-status-bar-icon" stroke={"var(--icon)"} />
      </button>
      {dockerData !== null ? (
        <>
          <div className="bt-status-bar-div" title="ROS 2 version">
            <label className="bt-status-bar-label">{`ROS 2: ${dockerData.ros_version}`}</label>
          </div>
          <div className="bt-status-bar-div" title="GPU status">
            <label className="bt-status-bar-label">{`GPU: ${dockerData.gpu_avaliable}`}</label>
          </div>
          <div className="bt-status-bar-div" title="Robotics Backend version">
            <label className="bt-status-bar-label">{`Robotics Backend: ${dockerData.robotics_backend_version}`}</label>
          </div>
        </>
      ) : (
        <button
          className={`bt-status-bar-button`}
          id={`reset-connection`}
          onClick={() => {
            resetManager();
          }}
          title="Reconnect with Robotics Backend"
        >
          <ResetIcon className="bt-status-bar-icon" stroke={"var(--icon)"} />
          <label className="bt-status-bar-label">Reconnect</label>
        </button>
      )}
      <button
        className={showSim ? `bt-status-bar-button-active` : `bt-status-bar-button`}
        onClick={() => {
          setSimVisible(!showSim);
        }}
        title="Toggle Simulator"
        style={{ marginLeft: "auto", width: "300px" }}
      >
        <SimulatorIcon className="bt-status-bar-icon" stroke={"var(--icon)"} />
        <label className="bt-status-bar-label">Simulator</label>
      </button>
    </div>
  );
};

export default StatusBar;
