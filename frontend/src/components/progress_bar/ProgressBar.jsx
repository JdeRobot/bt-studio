import "./ProgressBar.css";

const ProgressBar = ({ completed }) => {
  return (
    <div className="progress-bar-container">
      <div className="progress-bar-filler" style={{ width: `${completed}%` }}>
        <span className="progress-bar-percentage">{`${completed}%`}</span>
      </div>
    </div>
  );
};

export default ProgressBar;
