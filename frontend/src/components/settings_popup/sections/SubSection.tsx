const SubSection = ({ title, children }: { title: string; children: any }) => {
  return (
    <div className="bt-setting-subsection">
      <label className="bt-setting-subsection-title">{title}</label>
      {children}
    </div>
  );
};

export default SubSection;
