import React, { useMemo, useState, useEffect } from "react";

const Section = ({ title, children }) => {
  // const [open, setOpen] = useState(false);

  return (
    <div className="bt-setting-section">
      <label className="bt-setting-section-title">{title}</label>
      {children}
    </div>
  );
};

export default Section;
