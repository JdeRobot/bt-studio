import "./VncViewer.css";
import BounceLoader from "react-spinners/BounceLoader";

const VncViewer = ({ gazeboEnabled }: { gazeboEnabled: boolean }) => {
  return (
    <div className="bt-viewer">
      {gazeboEnabled ? (
        <iframe
          title="Gazebo"
          id={"iframe"}
          style={{
            width: "100%",
            height: "100%",
            border: 0,
          }}
          src={"http://127.0.0.1:6080/vnc.html?resize=remote&autoconnect=true"}
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
