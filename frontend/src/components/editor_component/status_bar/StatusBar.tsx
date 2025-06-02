import "./StatusBar.css";

import { ReactComponent as ResetIcon } from "./img/reset.svg";
import CommsManager from "../../../api_helper/CommsManager";

const StatusBar = ({
  commsManager,
  resetManager,
}: {
  commsManager: CommsManager | null;
  resetManager: Function;
}) => {
  var dockerData = commsManager?.getHostData()

  return (
    <div className="bt-status-bar-container">
      {dockerData ? (
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
    </div>
  );
};

export default StatusBar;
