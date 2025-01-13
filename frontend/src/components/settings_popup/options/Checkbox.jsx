import "./Checkbox.css";

const Checkbox = ({ setting }) => {
  return (
    <div className="bt-setting-checkbox-wrapper">
      <input
        type="checkbox"
        checked={setting.value}
        onChange={() => setting.setter(!setting.value)}
      />
    </div>
  );
};

export default Checkbox;
