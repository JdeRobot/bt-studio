const Section = ({ title, children }) => {
  return (
    <div className="bt-setting-section">
      <label className="bt-setting-section-title">{title}</label>
      {children}
    </div>
  );
};

export default Section;
