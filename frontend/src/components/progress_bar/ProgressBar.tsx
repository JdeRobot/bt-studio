import "./ProgressBar.css";

const ProgressBar = ({ completed }: { completed: number }) => {
  return (
    <div className="bt-progress-bar-container">
      <div
        className="bt-progress-bar-filler"
        style={{ width: `${completed}%` }}
      >
        <span className="bt-progress-bar-percentage">{`${completed}%`}</span>
      </div>
    </div>
  );
};

export default ProgressBar;
