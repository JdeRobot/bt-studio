import { useContext } from "react";
import { OptionsContext } from "../options/Options";

import "./StatusBar.css";

const StatusBar = ({
}: {
}) => {
  // Settings
  const settings = useContext(OptionsContext);

  return (
    <div className="status-bar-container">
    </div>
  );
};

export default StatusBar;
