import { useContext } from "react";
import { OptionsContext } from "../options/Options";

import "./StatusBar.css";

import { ReactComponent as TerminalIcon } from "./img/terminal.svg";

const StatusBar = ({
}: {
}) => {
  // Settings
  const settings = useContext(OptionsContext);

  return (
    <div className="status-bar-container">
      <button
        className="status-bar-button"
        onClick={() => console.log("a")}
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
        onClick={() => console.log("a")}
        title="Toggle Gazebo"
        style={{marginLeft:"auto"}}
      >
        <TerminalIcon className="status-bar-icon" stroke={"var(--icon)"} />
      </button>
    </div>
  );
};

export default StatusBar;
