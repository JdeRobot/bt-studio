import { useContext } from "react";
import { OptionsContext } from "../options/Options";

import "./StatusBar.css";

import { ReactComponent as TerminalIcon } from "./img/terminal.svg";
import { ReactComponent as SimulatorIcon } from "./img/gazebo.svg";

const StatusBar = ({
  showSim,
  setSimVisible,
  showTerminal,
  setTerminalVisible,
}: {
  showSim: boolean;
  setSimVisible: Function;
  showTerminal: boolean;
  setTerminalVisible: Function;
}) => {
  // Settings
  const settings = useContext(OptionsContext);

  return (
    <div className="status-bar-container">
      <button
        className="status-bar-button"
        onClick={() => {
          setTerminalVisible(!showTerminal);
        }}
        title="Toggle console"
      >
        <TerminalIcon className="status-bar-icon" stroke={"var(--icon)"} />
      </button>
      <button
        className="status-bar-button"
        onClick={() => console.log("a")}
        title="Toggle console"
      >
        <TerminalIcon className="status-bar-icon" stroke={"var(--icon)"} />
      </button>
      <button
        className="status-bar-button"
        onClick={() => {
          setSimVisible(!showSim);
        }}
        title="Toggle Simulator"
        style={{ marginLeft: "auto", width: "300px" }}
      >
        <SimulatorIcon className="status-bar-icon" stroke={"var(--icon)"} />
        <label className="status-bar-label">Simulator</label>
      </button>
    </div>
  );
};

export default StatusBar;
