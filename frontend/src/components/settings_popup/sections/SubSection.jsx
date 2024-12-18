import React, { useMemo, useState, useEffect } from "react";

const SubSection = ({ title, children }) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className="bt-setting-subsection">
      <label className="bt-setting-subsection-title">{title}</label>
      {children}
    </div>
  );
};

export default SubSection;
