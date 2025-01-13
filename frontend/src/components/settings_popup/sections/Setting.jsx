const Setting = ({ title, children }) => {
  return (
    <div className="bt-setting-setting">
      <label className="bt-setting-setting-title">{title}</label>
      {/* Add settings info */}
      {children}
    </div>
  );
};

export default Setting;
