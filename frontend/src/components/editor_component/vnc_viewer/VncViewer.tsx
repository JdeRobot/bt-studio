import CommsManager from "../../../api_helper/CommsManager";
import "./VncViewer.css";
import BounceLoader from "react-spinners/BounceLoader";

const enabled = (state?: string): boolean => {
  if (
    state === "visualization_ready" ||
    state === "application_running" ||
    state === "paused"
  ) {
    return true;
  }

  return false;
};

const VncViewer = ({
  commsManager,
  port,
}: {
  commsManager: CommsManager | null;
  port: number;
}) => {
  var state = commsManager?.getState();

  return (
    <div className="bt-viewer">
      {enabled(state) ? (
        <iframe
          title="Gazebo"
          id={"iframe"}
          style={{
            width: "100%",
            height: "100%",
            border: 0,
          }}
          src={`http://127.0.0.1:${port}/vnc.html?resize=remote&autoconnect=true`}
        />
      ) : (
        <div className="bt-loader">
          <BounceLoader color="var(--header)" size={80} speedMultiplier={0.7} />
        </div>
      )}
    </div>
  );
};

export default VncViewer;
