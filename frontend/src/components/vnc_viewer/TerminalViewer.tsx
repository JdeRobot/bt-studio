import "./VncViewer.css";
import BounceLoader from "react-spinners/BounceLoader";

const TerminalViewer = ({ gazeboEnabled }: { gazeboEnabled: boolean }) => {
  return (
    <div className="bt-terminal">
      {gazeboEnabled ? (
        <iframe
          title="Terminal"
          id={"iframe"}
          style={{
            width: "100%",
            height: "100%",
            border: 0,
          }}
          src={"http://127.0.0.1:1108/vnc.html?resize=remote&autoconnect=true"}
        />
      ) : (
        <div className="bt-loader-terminal">
          <BounceLoader color="var(--header)" size={80} speedMultiplier={0.7} />
        </div>
      )}
    </div>
  );
};

export default TerminalViewer;
