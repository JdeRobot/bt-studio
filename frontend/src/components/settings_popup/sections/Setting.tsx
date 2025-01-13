const Setting = ({ title, children }: { title:string, children:any }) => {
  return (
    <div className="bt-setting-setting">
      <label className="bt-setting-setting-title">{title}</label>
      {/* Add settings info */}
      {children}
    </div>
  );
};

export default Setting;
